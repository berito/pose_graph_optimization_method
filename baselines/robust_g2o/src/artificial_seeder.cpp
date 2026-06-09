#include "utils.hpp"

using namespace std;
using namespace g2o;

// we use the 2D and 3D SLAM types here
G2O_USE_TYPE_GROUP(slam2d);
G2O_USE_TYPE_GROUP(slam3d);
G2O_USE_OPTIMIZATION_LIBRARY(eigen);

int main(int argc, char** argv) 
{

  // Command line parsing
  int maxIterations = 100;
  string outputFilename;
  string open_loop_trajectory;
  string seeded_trajectory;
  string inputFilename;
  CommandArgs arg;
  
  arg.param("st", seeded_trajectory, "",
            "Trajectory with few loops used for seeding");
  arg.param("o", outputFilename, "seed_traj.g2o",
            "G2o file with ehnanced loops");
  arg.parseArgs(argc, argv);

  typedef Eigen::Isometry3d PoseType;
  typedef EdgeSE3 EdgeType;
  typedef VertexSE3 VertexType;
  
  vector<PoseType> init_poses;
  vector<VertexType*> v_poses;

  std::cout << "Reading file: " << seeded_trajectory << std::endl;


  // Creating GT using the optimized trajectory
  bool rescale = true;
  double scale = 0.05;
  SparseOptimizer optimizer;
  setProblem<PoseType, EdgeType, VertexType>(seeded_trajectory, optimizer, init_poses, v_poses);
  opencv2XYZ(optimizer);
  if ( rescale ) scaleTrajectory(optimizer, scale);
  /**
  optimizer.setVerbose(false);
  optimizer.vertex(0)->setFixed(true);
  std::cout << "Starting optimization : " << std::endl;
  optimizer.initializeOptimization();
  chrono::steady_clock::time_point begin = chrono::steady_clock::now();
  optimizer.optimize(maxIterations);
  chrono::steady_clock::time_point end = chrono::steady_clock::now();
  chrono::microseconds delta_time = chrono::duration_cast<chrono::microseconds>(end - begin);

  int neigh = 4;
  int lower_id_limit = v_poses.size() / 2;
  int n_loops = 3;
  int loop_counter = 0;

  int start_id = v_poses.size() - 1;
  int reverse_id = 0;
  vector<pair<int, int>> friends;
  while (start_id > lower_id_limit && loop_counter < n_loops)
  {
    // Find the closest vertex from the beginning
    int closest_id = -1;
    double closest_dist = std::numeric_limits<double>::max();
    bool decreasing = true;
    for (int cls_id = reverse_id; cls_id < lower_id_limit && decreasing; ++cls_id)
    {
      Eigen::Isometry3d pose1 = v_poses[start_id]->estimate();
      Eigen::Isometry3d pose2 = v_poses[cls_id]->estimate();
      Eigen::Isometry3d diff = pose1.inverse() * pose2;
      double dist = diff.translation().norm();
      decreasing = dist < closest_dist;

      if (!decreasing) continue;

      closest_dist = dist;
      closest_id = cls_id;
    }

    friends.push_back(make_pair(start_id, closest_id));
    start_id -= neigh;
    reverse_id += neigh;
    loop_counter++;
  }

  Eigen::Matrix<double, 6, 6> info = Eigen::Matrix<double, 6, 6>::Identity();
  info = dynamic_cast<EdgeType*>(*optimizer.edges().begin())->information();
  for (size_t ide = 0; ide < friends.size(); ++ide)
  {
      int id1 = friends[ide].first;
      int id2 = friends[ide].second;
      // Creating the edge
      EdgeType* edge = new EdgeType();
      edge->vertices()[0] = v_poses[id1];
      edge->vertices()[1] = v_poses[id2];
      edge->setMeasurement(v_poses[id1]->estimate().inverse() * v_poses[id2]->estimate());
      edge->setInformation(info);
      optimizer.addEdge(edge);
  }

  for (size_t it = 0; it < optimizer.vertices().size(); ++it )
  {
      VertexType* v = dynamic_cast<VertexType*>(optimizer.vertex(it));
      v->setEstimate(init_poses[v->id()]);
  }
  /**/
  optimizer.save(outputFilename.c_str());
  

  return 0;
}
