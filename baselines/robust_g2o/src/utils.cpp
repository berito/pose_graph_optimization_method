#include "utils.hpp"

using namespace std;
using namespace g2o;
using namespace Eigen;

#include <string>
#include <iostream>
#include <fstream>

#define PBSTR "||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||"
#define PBWIDTH 60

/*-------------------------------------------------------------------*/

void betterInitializationPoint(SparseOptimizer& optimizer)
{
    // Iterating trough the edges
    for ( auto it_e = optimizer.edges().begin(); it_e != optimizer.edges().end(); ++it_e )
    {
        auto edge_odom = dynamic_cast<EdgeSE2*>(*it_e);
        if ( edge_odom != nullptr )
        {
            auto v_fn = dynamic_cast<VertexSE2*>(edge_odom->vertices()[1]);
            auto v_st = dynamic_cast<VertexSE2*>(edge_odom->vertices()[0]);

            int id_st = v_st->id();
            int id_fn = v_fn->id();

            auto est = id_fn - id_st == 1 ? v_st->estimate() * edge_odom->measurement() : v_fn->estimate();
            v_fn->setEstimate(est);
        }
    }

    return;
}


/*-------------------------------------------------------------------*/

template <class T, class EDGE, class VERTEX>
void setProblem(const string& problem_file, 
                SparseOptimizer& optimizer,
                vector<T>& init_poses,
                vector<VERTEX*>& v_poses)
{
    optimizer.setVerbose(false);
    
    // allocate the solver
    OptimizationAlgorithmProperty solverProperty;
    //optimizer.setAlgorithm(OptimizationAlgorithmFactory::instance()->construct("dl_var", solverProperty));
    optimizer.setAlgorithm(OptimizationAlgorithmFactory::instance()->construct("gn_var", solverProperty));
    
    // Loading the g2o file
    ifstream ifs(problem_file.c_str());
    if (!ifs) 
    {
        cerr << "unable to open " << problem_file << endl;
        return;
    }
    optimizer.load(ifs);
    //odometryInitialization<EDGE, VERTEX>(optimizer);
    init_poses.resize(optimizer.vertices().size());
    v_poses.resize(optimizer.vertices().size());
    for ( auto it = optimizer.vertices().begin() ; it != optimizer.vertices().end() ; ++it )
    {
        auto v1 = dynamic_cast<VERTEX*>(it->second);
        v_poses[v1->id()] = v1;
        init_poses[v1->id()]= v1->estimate();
    }

    return;
}

template void setProblem<SE2, EdgeSE2, VertexSE2>(const string& problem_file, 
                                                  SparseOptimizer& optimizer,
                                                  vector<SE2>& init_poses,
                                                  vector<VertexSE2*>& v_poses);
template void setProblem<Isometry3d, EdgeSE3, VertexSE3>(const string& problem_file, 
                                                         SparseOptimizer& optimizer,
                                                         vector<Isometry3d>& init_poses,
                                                         vector<VertexSE3*>& v_poses);

/*-------------------------------------------------------------------*/

template <class EDGE, class VERTEX>
void odometryInitialization(SparseOptimizer& optimizer)
{
    // Iterating trough the edges
    for ( auto it_e = optimizer.edges().begin(); it_e != optimizer.edges().end(); ++it_e )
    {
        auto edge_odom = dynamic_cast<EDGE*>(*it_e);
        if ( edge_odom != nullptr )
        {
            auto v_fn = dynamic_cast<VERTEX*>(edge_odom->vertices()[1]);
            auto v_st = dynamic_cast<VERTEX*>(edge_odom->vertices()[0]);

            int id_st = v_st->id();
            int id_fn = v_fn->id();

            auto est = id_fn - id_st == 1 ? v_st->estimate() * edge_odom->measurement(): v_fn->estimate();
            v_fn->setEstimate(est);
        }
    }

    return;
}

template void odometryInitialization<EdgeSE2, VertexSE2>(SparseOptimizer& optimizer);
template void odometryInitialization<EdgeSE3, VertexSE3>(SparseOptimizer& optimizer);

/*-------------------------------------------------------------------*/

template <class EDGE, class VERTEX>
void propagateCurrentGuess(SparseOptimizer& optimizer, int id_start, const vector<OptimizableGraph::Edge*>& odom)
{ 
    for ( size_t i = id_start + 1; i <= odom.size() ; ++i )
    {
        auto eo = dynamic_cast<EDGE*>(odom[i - 1]);
        auto v1 = dynamic_cast<VERTEX*>(optimizer.vertex(i - 1));
        auto v2 = dynamic_cast<VERTEX*>(optimizer.vertex(i));
        v2->setEstimate(v1->estimate() * eo->measurement());
    }

    return;
}

template void propagateCurrentGuess<EdgeSE2, VertexSE2>(SparseOptimizer& optimizer, int id_start, const vector<OptimizableGraph::Edge*>& odom);
template void propagateCurrentGuess<EdgeSE3, VertexSE3>(SparseOptimizer& optimizer, int id_start, const vector<OptimizableGraph::Edge*>& odom);

/*-------------------------------------------------------------------*/


template <class EDGE, class VERTEX>
void getOdometryEdges(const SparseOptimizer& optimizer, OptimizableGraph::EdgeContainer& odometry_edges)
{
    // Clearing any previous data
    odometry_edges.clear();

    // Iterating trough the edges
    for ( auto it_e = optimizer.edges().begin(); it_e != optimizer.edges().end(); ++it_e )
    {
        auto edge = dynamic_cast<EDGE*>(*it_e);
        if ( edge == nullptr ) continue;
        
        int id_st = dynamic_cast<VERTEX*>(edge->vertices()[1])->id();
        int id_fn = dynamic_cast<VERTEX*>(edge->vertices()[0])->id();

        if (abs(id_fn - id_st) != 1) continue;

        odometry_edges.push_back(edge);
    }

    return;
}

template void getOdometryEdges<EdgeSE2, VertexSE2>(const SparseOptimizer& optimizer, OptimizableGraph::EdgeContainer& odometry_edges);
template void getOdometryEdges<EdgeSE3, VertexSE3>(const SparseOptimizer& optimizer, OptimizableGraph::EdgeContainer& odometry_edges);

/*-------------------------------------------------------------------*/

template <class EDGE, class VERTEX>
void getLoopEdges(const SparseOptimizer& optimizer, OptimizableGraph::EdgeContainer& loop_edges)
{
    // Clearing any previous data
    loop_edges.clear();

    // Iterating trough the edges
    for ( auto it_e = optimizer.edges().begin(); it_e != optimizer.edges().end(); ++it_e )
    {
        auto edge = dynamic_cast<EDGE*>(*it_e);
        if ( edge == nullptr ) continue;
        
        int id_st = dynamic_cast<VERTEX*>(edge->vertices()[1])->id();
        int id_fn = dynamic_cast<VERTEX*>(edge->vertices()[0])->id();

        if (abs(id_fn - id_st) == 1) continue;

        loop_edges.push_back(edge);
    }

    return;
}

template void getLoopEdges<EdgeSE2, VertexSE2>(const SparseOptimizer& optimizer, OptimizableGraph::EdgeContainer& loop_edges);
template void getLoopEdges<EdgeSE3, VertexSE3>(const SparseOptimizer& optimizer, OptimizableGraph::EdgeContainer& loop_edges);

/*-------------------------------------------------------------------*/

void wishartPrior(const Eigen::MatrixXd& sigma_0, const double w_prior, const int n_measurements,
                  Eigen::MatrixXd& v_matrix, double& v)
{
    
    //int mat_dof = v_matrix.cols() * v_matrix.rows();
    int mat_dof = 6;
    v_matrix.setZero();

    v_matrix = w_prior * n_measurements * sigma_0.inverse();
    v = w_prior * n_measurements + mat_dof + 1;

    return;
}

/*-------------------------------------------------------------------*/

template <class EDGE>
Eigen::MatrixXd computeSampleCovariance(OptimizableGraph::EdgeContainer& edges)
{
    EDGE* e0 = dynamic_cast<EDGE*>(edges[0]);
    int dim_measure = e0->measurementDimension();
    Eigen::MatrixXd sample_cov(dim_measure, dim_measure);
    sample_cov.setZero();

    for (size_t idx = 0; idx < edges.size(); ++idx )
    {
        EDGE* e = dynamic_cast<EDGE*>(edges[idx]);
        e->computeError();
        sample_cov += e->error() * e->error().transpose();
    } 
    
    double factor = 1.0 / static_cast<double>(edges.size());

    return factor * sample_cov;
}

template Eigen::MatrixXd computeSampleCovariance<EdgeSE2>(OptimizableGraph::EdgeContainer& edges);
template Eigen::MatrixXd computeSampleCovariance<EdgeSE3>(OptimizableGraph::EdgeContainer& edges);

/*-------------------------------------------------------------------*/



template <class T>
void readSolutionFile(vector<T>& poses, const string& path)
{
    const int LINESIZE = 81920;

    ifstream in_data(path.c_str());
    if (!in_data )
        throw invalid_argument("Cannot find file : " + path);

    // Keep going until the file has been read
    while ( !in_data.eof() )
    {
        T p = T::Identity();
        readLine(in_data, p);
        poses.push_back(p);

        in_data.ignore(LINESIZE, '\n');
    }

    return;
}

template void readSolutionFile<Isometry2d>(vector<Isometry2d>& poses, const string& path);
template void readSolutionFile<Isometry3d>(vector<Isometry3d>& poses, const string& path);

/*-------------------------------------------------------------------*/

void writeVertex(ofstream& out_data, VertexSE2* v)
{
    out_data << v->estimate()[0] << " " 
             << v->estimate()[1] << " " 
             << v->estimate()[2] << endl;
    return;
}


void writeVertex(ofstream& out_data, VertexSE3* v)
{
    Isometry3d pose = v->estimate();
    Quaterniond q(pose.linear());
    Vector3d eul = pose.linear().eulerAngles(0, 1, 2);
    Vector3d t = pose.translation();

    out_data << t[0] << " " << t[1] << " " << t[2] << " "
             << eul[0] << " " <<eul[1] << " " << eul[2] << endl;

    return;
}

/*-------------------------------------------------------------------*/

void readLine(ifstream& in_data, Isometry2d& pose)
{
    double x, y, yaw;
    in_data >> x >> y >> yaw;

    pose = Isometry2d::Identity();
    pose.translation() = Vector2d(x, y);
    pose.linear() = Eigen::Rotation2D<double>(yaw).toRotationMatrix();

    return;
}

void readLine(ifstream& in_data, Isometry3d& pose)
{
    double x, y, z, qx, qy, qz, qw;
    in_data >> x >> y >> z >> qx >> qy >> qz >> qw;

    pose = Isometry3d::Identity();
    pose.translation() = Vector3d(x, y, z);
    pose.linear() = Quaterniond(qw, qx, qy, qz).toRotationMatrix();

    return;
}


/*-------------------------------------------------------------------*/
void readConfig(const std::string& cfg_filepath, Config& out_cfg)
{
    YAML::Node config = YAML::LoadFile(cfg_filepath);

    // Filter parameters
    out_cfg.dataset = config["dataset"].as<std::string>();
    out_cfg.output = config["output"].as<std::string>();
    out_cfg.canonic_inliers = config["canonic_inliers"].as<int>();
    out_cfg.maxiters = config["max_iters"].as<int>();
    out_cfg.inlier_th = config["inlier_th"].as<double>();
    out_cfg.batch_size = config["batch_size"].as<int>();
    
    // Switchable variables
    out_cfg.switch_prior = config["switch_prior"].as<double>();

    // MaxMix parameters
    out_cfg.maxmix_weight = config["maxmix_weight"].as<double>();
    out_cfg.nu_constraints = config["nu_constraint"].as<double>();
    out_cfg.nu_nullHypothesis = config["nu_nullHypothesis"].as<double>();


    return;
}
/*-------------------------------------------------------------------------*/

bool cmpTime(pair<int, OptimizableGraph::Edge*> p1, pair<int, OptimizableGraph::Edge*> p2)
{
    int id1_v1 = p1.second->vertices()[1]->id();
    int id1_v2 = p1.second->vertices()[0]->id();
    int id1_max = id1_v1 > id1_v2 ? id1_v1 : id1_v2;

    int id2_v1 = p2.second->vertices()[1]->id();
    int id2_v2 = p2.second->vertices()[0]->id();
    int id2_max = id2_v1 > id2_v2 ? id2_v1 : id2_v2;

    return (id1_max < id2_max);
}

/*-----------------------------------------------------------------------*/

void opencv2XYZ(SparseOptimizer& optimizer)
{
    // Trabsforming from OpenCV to XYZ convention
    Eigen::Matrix3d R_cv;
    R_cv << 0.0, -1.0,  0.0,
            0.0,  0.0, -1.0,
            1.0,  0.0,  0.0;
    Eigen::Vector3d t_cv = Eigen::Vector3d::Zero();
    Eigen::Isometry3d T_cv = Eigen::Isometry3d::Identity();
    T_cv.linear() = R_cv;
    T_cv.translation() = t_cv;

    for ( auto it = optimizer.vertices().begin() ; it != optimizer.vertices().end() ; ++it )
    {
        auto v = dynamic_cast<VertexSE3*>(it->second);
        Eigen::Isometry3d pose = v->estimate();
        Eigen::Isometry3d new_pose = T_cv.inverse() * pose * T_cv;
        v->setEstimate(new_pose);
    }

    for ( auto it_e = optimizer.edges().begin(); it_e != optimizer.edges().end(); ++it_e )
    {
        auto edge = dynamic_cast<EdgeSE3*>(*it_e);
        Eigen::Isometry3d meas = edge->measurement();
        Eigen::Isometry3d new_meas = T_cv.inverse() * meas * T_cv;
        edge->setMeasurement(new_meas);

        Eigen::Matrix<double, 6, 6> info = edge->information();
        Eigen::Matrix<double, 6, 6> rotated_info = Eigen::Matrix<double, 6, 6>::Zero();
        rotated_info.block<3,3>(0,0) = R_cv.transpose() * info.block<3,3>(0,0) * R_cv;
        rotated_info.block<3,3>(0,3) = R_cv.transpose() * info.block<3,3>(0,3) * R_cv;
        rotated_info.block<3,3>(3,0) = R_cv.transpose() * info.block<3,3>(3,0) * R_cv;
        rotated_info.block<3,3>(3,3) = R_cv.transpose() * info.block<3,3>(3,3) * R_cv;
        edge->setInformation(rotated_info);
    }  


    return;
}

void correctedInformationMatrices(g2o::SparseOptimizer& optimizer)
{
    for ( auto it_e = optimizer.edges().begin(); it_e != optimizer.edges().end(); ++it_e )
    {
        auto edge = dynamic_cast<EdgeSE3*>(*it_e);
        if ( edge == nullptr ) continue;

        Eigen::Matrix<double, 6, 6> info = Eigen::Matrix<double, 6, 6>::Zero();

        // Correcting translation part
        info(0, 0) = 800.0;  // TUM: 300.0   | KITTI_05: 1000.0 | KITTI_00: 800.0
        info(1, 1) = 650.0;  // TUM: 260.0   | KITTI_05: 800.0  | KITTI_00: 650.0
        info(2, 2) = 5000.0; // TUM: 2500.0  | KITTI_05: 1000.0 | KITTI_00: 5000.0
        // Correcting rotation part
        info(3, 3) = 5000.0; // TUM: 2500.0 | KITTI_05: 1000.0 | KITTI_00: 5000.0
        info(4, 4) = 5000.0; // TUM: 2500.0 | KITTI_05: 1000.0 | KITTI_00: 5000.0
        info(5, 5) = 700.0;  // TUM: 200.0  | KITTI_05: 900.0  | KITTI_00: 700.0
        edge->setInformation(info);
    }  

    return;
}


void scaleTrajectory(SparseOptimizer& optimizer, const double scale)
{
    for ( auto it = optimizer.vertices().begin() ; it != optimizer.vertices().end() ; ++it )
    {
        auto v = dynamic_cast<VertexSE3*>(it->second);
        Eigen::Isometry3d pose = v->estimate();
        pose.translation() *= scale;
        v->setEstimate(pose);
    }

    for ( auto it_e = optimizer.edges().begin(); it_e != optimizer.edges().end(); ++it_e )
    {
        auto edge = dynamic_cast<EdgeSE3*>(*it_e);
        Eigen::Isometry3d meas = edge->measurement();
        meas.translation() *= scale;
        edge->setMeasurement(meas);
    }

    return;
}

void printProgress(double percentage) 
{
    int val = (int) (percentage * 100);
    int lpad = (int) (percentage * PBWIDTH);
    int rpad = PBWIDTH - lpad;
    printf("\r%3d%% [%.*s%*s]", val, lpad, PBSTR, rpad, "");
    fflush(stdout);
}