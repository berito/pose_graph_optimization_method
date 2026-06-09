#include "metric_eval.hpp"

#define LINESIZE 81920

double getAngle(const Eigen::Matrix3d& R) 
{
  // to compute the angular error we refer to the axis-angle representation of the error
  // rotation matrix from that we can extract the sin and cos of the angle and then use the
  // atan2 (see Alessandro De Luca slides for the Robotics 1 course)
  const Eigen::Matrix3d skew_R = R - R.transpose();
  // sin of the error angle
  double sin_angle = 0.5 * std::sqrt(skew_R(0, 1) * skew_R(0, 1) + skew_R(0, 2) * skew_R(0, 2) +
                                     skew_R(1, 2) * skew_R(1, 2));
  // cos of the error angle
  double cos_angle = 0.5 * (R.trace() - 1.0);

  double angle = std::atan2(sin_angle, cos_angle);
  return angle;
}

/**/
void fixRotation(Eigen::Matrix3d& R) 
{
  Eigen::Matrix3d E = R.transpose() * R;
  for ( size_t idx = 0; idx < 3; ++idx ) E(idx, idx) -= 1.0;
  R -= R * E * 0.5;

  return;
}
/**/

void computeAlignmentTransform(const std::vector<Eigen::Isometry3d>& graph,
                               const std::vector<Eigen::Isometry3d>& target_graph,
                               Eigen::Isometry3d& delta) 
{
  std::vector<Eigen::Vector3d> target, query;
  target.reserve(graph.size());
  query.reserve(graph.size());

  // Vector of positions
  for ( size_t i = 0 ; i < target_graph.size(); ++i )
  {
    query.emplace_back(graph[i].translation());
    target.emplace_back(target_graph[i].translation());
  }

  // Computing mean
  Eigen::Vector3d mean_query = Eigen::Vector3d::Zero(), mean_target = Eigen::Vector3d::Zero();
  for (size_t i = 0; i < query.size(); ++i) 
  {
    mean_query += query.at(i);
    mean_target += target.at(i);
  }

  const double inv_size = 1.0 / (double) query.size();
  mean_query *= inv_size;
  mean_target *= inv_size;
  Eigen::Matrix3d sigma = Eigen::Matrix3d::Zero();
  for (size_t i = 0; i < query.size(); ++i)
    sigma += (target.at(i) - mean_target) * (query.at(i) - mean_query).transpose();
  
  sigma *= inv_size;
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(sigma, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d W = Eigen::Matrix3d::Identity();
  if (svd.matrixU().determinant() * svd.matrixV().determinant() < 0)
    W(2, 2) = -1;

  Eigen::Matrix3d R = svd.matrixU() * W * svd.matrixV().transpose();
  
  fixRotation(R);
  delta.translation() = mean_target - R * mean_query;
  delta.linear()      = R;

  return;
}

void align(std::vector<Eigen::Isometry3d>& sol, const Eigen::Isometry3d& T) 
{
  for ( size_t i = 0 ; i < sol.size() ; ++i )
  {
    Eigen::Isometry3d estimate = sol[i];
    Eigen::Isometry3d aligned_estimate = T * estimate;
    sol[i] = aligned_estimate;  
  }

  return;
}



void evaluateAte(std::vector<Eigen::Isometry3d>& sol,
                 const std::vector<Eigen::Isometry3d>& gt,
                 double& ate_rotation,
                 double& ate_translation,
                 double& rmse_translation) 
{
  auto& gt_variables    = gt;
  auto& query_variables = sol;

  // tg check it the two graph are same number of variables
  if (gt_variables.size() != query_variables.size())
  {
    std::cout << "GT_SIZE = " << gt_variables.size() << std::endl;
    std::cout << "QUERY_SIZE = " << query_variables.size() << std::endl;
    throw std::runtime_error("|number of variables mismatch between query and ground truth");
  }

  Eigen::Isometry3d delta_alignment = Eigen::Isometry3d::Identity();
  computeAlignmentTransform(query_variables, gt_variables, delta_alignment);
  //delta_alignment = query_variables[0].inverse();

  align(query_variables, delta_alignment);

  // tg counter for skipped variables (due to nans) and processed
  size_t skipped   = 0;
  size_t processed = 0;
  // tg for each ground truth variable
  for ( size_t id = 0; id < gt_variables.size(); ++id )
  {
    // Corresponding posese
    const Eigen::Isometry3d target = gt_variables[id];
    const Eigen::Isometry3d query  = query_variables[id];

    // extract rotation and translation
    const Eigen::Matrix3d Ri = target.linear();
    const Eigen::Matrix3d Rj = query.linear();
    const Eigen::Vector3d ti = target.translation();
    const Eigen::Vector3d tj = query.translation();

    // compute rotation error
    Eigen::Matrix3d R_error = Ri * Rj.transpose();
    
    // fix round off due to the multiplication
    fixRotation(R_error);
  
    // compute rotation error
    double angle_error           = getAngle(R_error);
    Eigen::Vector3d t_error_ATE = ti - R_error * tj;
    Eigen::Vector3d t_error     = ti - tj;

    // check for nans
    if (std::isnan(t_error_ATE.norm()) || std::isnan(angle_error)) 
    {
      std::cout << "WARNING, detected nan, skipping vertex [ " << id << " ]\n";
      ++skipped;
      continue;
    }

    // accumulate squared errors
    ate_translation  += t_error_ATE.transpose() * t_error_ATE;
    ate_rotation     += angle_error * angle_error;
    rmse_translation += t_error.transpose() * t_error;
    ++processed;

  }

  std::cout << "processed variables [ " << processed << "/" << gt_variables.size() << " ]\n";
  std::cout << "skipped variables [ " << skipped << "/" << gt_variables.size() << " ]\n";
  if (!processed)
    throw std::runtime_error("invalid types");

  // get number of variables processed
  const double inverse_num_variables = 1.0 / (double) (processed);
  // compute final ATE
  ate_translation  *= inverse_num_variables;
  ate_rotation     *= inverse_num_variables;
  rmse_translation *= inverse_num_variables;

  ate_translation    = std::sqrt(ate_translation);
  ate_rotation       = std::sqrt(ate_rotation);
  rmse_translation   = std::sqrt(rmse_translation);

  return;
}

void evaluateRpe(const std::vector<Eigen::Isometry3d>& sol,
                 const std::vector<Eigen::Isometry3d>& gt,
                 double& rpe_rotation,
                 double& rpe_translation) 
{
  auto& gt_variables    = gt;
  auto& query_variables = sol;

  // tg check it the two graph are same number of variables
  if (gt_variables.size() != query_variables.size())
  {
    std::cout << "GT_SIZE = " << gt_variables.size() << std::endl;
    std::cout << "QUERY_SIZE = " << query_variables.size() << std::endl;
    throw std::runtime_error("|number of variables mismatch between query and ground truth");
  }

  // tg counter for skipped variables (due to nans) and processed
  size_t skipped   = 0;
  size_t processed = 0;
  // tg for each ground truth variable
  for ( size_t id = 0; id < gt_variables.size() - 1; ++id )
  {
    // Corresponding posese
    const Eigen::Isometry3d target = gt_variables[id].inverse() * gt_variables[id + 1];
    const Eigen::Isometry3d query  = query_variables[id].inverse() * query_variables[id + 1];

    // extract rotation and translation
    const Eigen::Isometry3d delta = target.inverse() * query;
    
    // compute rotation error
    Eigen::Matrix3d R_error = delta.linear().matrix();
    
    // compute rotation error
    double angle_error           = getAngle(R_error);

    //Eigen::Vector3d t_error = delta.translation();
    Eigen::Vector3d t_error = target.translation() - R_error.transpose() * query.translation();


    // check for nans
    if (std::isnan(t_error.norm()) || std::isnan(angle_error)) 
    {
      std::cout << "WARNING, detected nan, skipping vertex [ " << id << " ]\n";
      ++skipped;
      continue;
    }

    // accumulate squared errors
    rpe_translation  += t_error.transpose() * t_error;
    rpe_rotation     += angle_error * angle_error;
    ++processed;

  }

  std::cout << "processed variables [ " << processed << "/" << gt_variables.size() - 1 << " ]\n";
  std::cout << "skipped variables [ " << skipped << "/" << gt_variables.size() - 1<< " ]\n";
  if (!processed)
    throw std::runtime_error("invalid types");

  // compute final ATE
  const double inverse_num_variables = 1.f / (double) (processed);
  rpe_translation  *= inverse_num_variables;
  rpe_rotation     *= inverse_num_variables;

  rpe_translation    = std::sqrt(rpe_translation);
  rpe_rotation       = std::sqrt(rpe_rotation);

  return;
}


void readSolutionFile2D(std::vector<Eigen::Isometry3d>& poses, const std::string& path)
{
  std::ifstream in_data(path.c_str());
  if (!in_data )
    throw std::invalid_argument("Cannot find file : " + path);

  // Keep going until the file has been read
  double x, y, yaw;
  while ( !in_data.eof() )
  {
    in_data >> x >> y >> yaw;

    Eigen::Isometry3d p = Eigen::Isometry3d::Identity();
    p.translation() = Eigen::Vector3d(x, y, 0.0);
    p.linear() = (Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX()) *
                  Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
                  Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ())).toRotationMatrix();

    poses.push_back(p);

    in_data.ignore(LINESIZE, '\n');
  }

  return;
}

void readSolutionFile3D(std::vector<Eigen::Isometry3d>& poses, const std::string& path)
{
  std::ifstream in_data(path.c_str());
  if (!in_data ) throw std::invalid_argument("Cannot find file : " + path);

  // Keep going until the file has been read
  //double x, y, z, qx, qy, qz, qw;
  double x, y, z, roll, pitch, yaw;
  while ( !in_data.eof() )
  {
    //in_data >> x >> y >> z >> qx >> qy >> qz >> qw;
    in_data >> x >> y >> z >> roll >> pitch >> yaw;

    Eigen::Isometry3d p = Eigen::Isometry3d::Identity();
    p.translation() = Eigen::Vector3d(x, y, z);
    /**/
    p.linear() = (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
                  Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                  Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX())).toRotationMatrix();
    /**/
    /*
    p.linear() = Eigen::Quaterniond(qw, qx, qy, qz).toRotationMatrix();
    /**/

    poses.push_back(p);

    in_data.ignore(LINESIZE, '\n');
  }

  return;
}

void evaluateSectionRpe(std::vector<Eigen::Isometry3d>& sol,
                        const std::vector<Eigen::Isometry3d>& gt,
                        const g2o::SparseOptimizer& graph,
                        double& rmse_translation,
                        double& energy_local_def,
                        double& valid_trajectory)
{
  auto& gt_variables    = gt;
  auto& query_variables = sol;
  energy_local_def = 0.0;
  rmse_translation = 0.0;

  // tg check it the two graph are same number of variables
  if (gt_variables.size() != query_variables.size())
  {
    std::cout << "GT_SIZE = " << gt_variables.size() << std::endl;
    std::cout << "QUERY_SIZE = " << query_variables.size() << std::endl;
    throw std::runtime_error("|number of variables mismatch between query and ground truth");
  }

  // tg counter for skipped variables (due to nans) and processed
  size_t skipped   = 0;
  size_t processed = 0;
  
  int valid_moves = 0;
  for ( auto it_e = graph.edges().begin(); it_e != graph.edges().end(); ++it_e )
  {
    g2o::EdgeSE3* edge = dynamic_cast<g2o::EdgeSE3*>(*it_e);
    if (!edge) continue;

    int id1 = edge->vertices()[0]->id();
    int id2 = edge->vertices()[1]->id();

    int diff = std::abs(id2 - id1);

    if (diff != 1) continue;

    // Corresponding posese
    const Eigen::Isometry3d target = gt_variables[id1].inverse() * gt_variables[id2];
    const Eigen::Isometry3d query  = query_variables[id1].inverse() * query_variables[id2];

    const Eigen::Isometry3d delta = target.inverse() * query;
    const Eigen::Vector2d t_error(delta.translation()[0], delta.translation()[1]);

    const Eigen::Matrix2d info = edge->information().block<2, 2>(0, 0);

    double mah_dist = t_error.transpose() * info * t_error;
    double chi2_90 = 0.21;
    double chi2_95 = 0.1;

    double err_mag = t_error.norm();

    ++valid_moves;

    if (mah_dist < chi2_95)
    {
      ++processed;
      rmse_translation += t_error.transpose() * t_error;
    }
    else
    {
      ++skipped;
      energy_local_def += err_mag;
      //std::cout << "-----\n";
      //std::cout << "Error = " << err_mag << std::endl;
      //std::cout << "Ref = " << ref_mag << std::endl;
      //std::cout << "Malahanobis Dist = " << mah_dist << std::endl;
    }

  }

  double perc_total_section = (1.0 * processed)/(valid_moves);
  double section_rpe = processed > 0 ? std::sqrt(rmse_translation / processed) : 0.0;

  skipped += 1;
  std::cout << "Section RPE = " << section_rpe << std::endl;
  std::cout << "Total Section = " << processed << std::endl;
  std::cout << "Local Energy = " << energy_local_def << std::endl;
  std::cout << "Percentage valid trajectory = " << perc_total_section << std::endl;
  
  valid_trajectory = perc_total_section;
  rmse_translation = section_rpe;
  //energy_local_def = skipped > 0 ? energy_local_def / skipped : 0.0;


  return; 
}



void evaluateFPSRpe(std::vector<Eigen::Isometry3d>& sol,
                    const std::vector<Eigen::Isometry3d>& gt,
                    const double percentage,
                    double& rmse_translation)
{

  auto& gt_variables    = gt;
  auto& query_variables = sol;

  // tg check it the two graph are same number of variables
  if (gt_variables.size() != query_variables.size())
  {
    std::cout << "GT_SIZE = " << gt_variables.size() << std::endl;
    std::cout << "QUERY_SIZE = " << query_variables.size() << std::endl;
    throw std::runtime_error("|number of variables mismatch between query and ground truth");
  }

  // tg counter for skipped variables (due to nans) and processed
  size_t skipped   = 0;
  size_t processed = 0;
  
  int samples = percentage * gt_variables.size();
  int step = gt_variables.size() / samples;

  double tot_error = 0.0;
  std::vector<int> bucket(gt_variables.size(), 0);
  for ( int iter = 0; iter < samples; iter+=step )
  {

    int id1 = iter;

    // Find farthest point from id1
    double max_dist = 0.0;
    int id2 = 0;
    const Eigen::Isometry3d start_pt_inv = gt_variables[id1].inverse();
    for (int fd_idx = 0 ; fd_idx < gt_variables.size(); ++fd_idx)
    {
      double dist = (start_pt_inv * gt_variables[fd_idx]).translation().norm();
      if (dist < max_dist && !bucket[fd_idx]) continue;

      max_dist = dist;
      id2 = fd_idx;
    }

    bucket[id2] = 1;

    // Corresponding posese
    const Eigen::Isometry3d target = gt_variables[id1].inverse() * gt_variables[id2];
    const Eigen::Isometry3d query  = query_variables[id1].inverse() * query_variables[id2];

    const Eigen::Isometry3d delta = target.inverse() * query;
    const Eigen::Vector3d t_error = delta.translation();
    
    double err_sq = t_error.transpose() * t_error;
    tot_error += err_sq;
  }

  rmse_translation = std::sqrt(tot_error / samples);

  std::cout << "FPS RPE = " << rmse_translation << std::endl;
  
  return;  
}


void evaluateEnergyDeformation(std::vector<Eigen::Isometry3d>& sol,
                               const std::vector<Eigen::Isometry3d>& gt,
                               const g2o::SparseOptimizer& graph,
                               double& energy_drift)
{

  energy_drift = 0.0;
  auto& gt_variables    = gt;
  auto& query_variables = sol;

  // tg for each ground truth variable
  size_t processed = 0;
  int valid_moves = 0;
  for ( auto it_e = graph.edges().begin(); it_e != graph.edges().end(); ++it_e )
  {
    g2o::EdgeSE3* edge = dynamic_cast<g2o::EdgeSE3*>(*it_e);
    if (!edge) continue;

    auto v1 = edge->vertices()[0];
    auto v2 = edge->vertices()[1];

    int diff = std::abs(v2->id() - v1->id());

    if (diff == 1) continue;

    int id1 = v1->id();
    int id2 = v2->id();

    // Corresponding posese
    const Eigen::Isometry3d target = gt_variables[id1].inverse() * gt_variables[id2];
    const Eigen::Isometry3d query  = query_variables[id1].inverse() * query_variables[id2];

    
    const Eigen::Isometry3d delta = target.inverse() * query;
    const Eigen::Vector2d t_error(delta.translation()[0], delta.translation()[1]);

    const Eigen::Matrix2d info = edge->information().block<2, 2>(0, 0);

    double mah_dist = t_error.transpose() * info * t_error;
    double chi2_90 = 0.21;
    double chi2_95 = 0.1;
    if (mah_dist < chi2_95) continue;

    double err_mag = t_error.norm();
    energy_drift += err_mag;
    ++processed;

  }

  //energy_drift = processed > 0 ? energy_drift / processed : 0.0;
  std::cout << "Global Energy = " << energy_drift << std::endl;

  return;
}

void evaluateFPSRpeVanilla(std::vector<Eigen::Isometry3d>& sol,
                           const std::vector<Eigen::Isometry3d>& gt,
                           const int samples,
                           double& rmse_translation)
{

  auto& gt_variables    = gt;
  auto& query_variables = sol;

  // tg check it the two graph are same number of variables
  if (gt_variables.size() != query_variables.size())
  {
    std::cout << "GT_SIZE = " << gt_variables.size() << std::endl;
    std::cout << "QUERY_SIZE = " << query_variables.size() << std::endl;
    throw std::runtime_error("|number of variables mismatch between query and ground truth");
  }

  // tg counter for skipped variables (due to nans) and processed
  size_t skipped   = 0;
  size_t processed = 0;
  
  // Choose the indices of the samples based on actual FPS
  std::vector<int> indices(samples, 0);
  std::vector<std::pair<int, Eigen::Vector3d>> pt_cloud;
  for ( size_t iter = 0; iter < gt_variables.size(); ++iter )
    pt_cloud.push_back(std::make_pair(0, gt_variables[iter].translation()));
  indices[0] = 0;
  pt_cloud[0].first = 1;

  // FPS algorithm
  for ( size_t s_idx = 1; s_idx < samples; ++s_idx )
  {
    double max_dist = 0;
    int max_idx = -1;
    for ( size_t pt_idx = 0; pt_idx < pt_cloud.size(); ++pt_idx )
    {
      // Already selected 
      if (pt_cloud[pt_idx].first) continue;

      double min_dist = std::numeric_limits<double>::max();
      int min_idx = -1;
      Eigen::Vector3d pt = pt_cloud[pt_idx].second;
      for ( size_t cand_idx = 0; cand_idx < s_idx; ++cand_idx )
      {
        Eigen::Vector3d selected_pt = pt_cloud[indices[cand_idx]].second;
        double dist = (pt - selected_pt).norm();

        if (dist < min_dist)
        {
          min_dist = dist;
          min_idx = cand_idx;
        }
      }

      if (min_dist > max_dist)
      {
        max_dist = min_dist;
        max_idx = pt_idx;
      }
    
    }

    indices[s_idx] = max_idx;
    pt_cloud[max_idx].first = 1;
  }

  /* Sorting and visualization (DEBUGGING)
  std::sort(indices.begin(), indices.end());
  std::cout << "Selected indices = ";
  for ( size_t idx = 0; idx < indices.size(); ++idx )
    std::cout << indices[idx] << " ";
  std::cout << std::endl;
  */

  double tot_error = 0.0;
  std::vector<int> bucket(gt_variables.size(), 0);
  for ( size_t iter = 0; iter < samples; ++iter )
  {

    int id1 = iter;
    // Find farthest point from id1
    double max_dist = 0.0;
    int id2 = 0;
    const Eigen::Isometry3d start_pt_inv = gt_variables[id1].inverse();
    for (int fd_idx = 0 ; fd_idx < gt_variables.size(); ++fd_idx)
    {
      double dist = (start_pt_inv * gt_variables[fd_idx]).translation().norm();
      if (dist < max_dist && !bucket[fd_idx]) continue;

      max_dist = dist;
      id2 = fd_idx;
    }

    bucket[id2] = 1;

    // Corresponding posese
    const Eigen::Isometry3d target = gt_variables[id1].inverse() * gt_variables[id2];
    const Eigen::Isometry3d query  = query_variables[id1].inverse() * query_variables[id2];

    const Eigen::Isometry3d delta = target.inverse() * query;
    const Eigen::Vector3d t_error = delta.translation();
    
    double err_sq = t_error.transpose() * t_error;
    tot_error += err_sq;
  }

  rmse_translation = std::sqrt(tot_error / samples);

  
  return;  
}
