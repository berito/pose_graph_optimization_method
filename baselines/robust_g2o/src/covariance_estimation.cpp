#include "utils.hpp"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"

using namespace std;
using namespace g2o;

// we use the 2D and 3D SLAM types here
G2O_USE_TYPE_GROUP(slam2d);
G2O_USE_TYPE_GROUP(slam3d);
G2O_USE_OPTIMIZATION_LIBRARY(eigen);

int main(int argc, char** argv) 
{

  // Command line parsing
  int maxIterations = 2;
  string refined_dataset;
  string open_loop_trajectory;
  string dataset;
  CommandArgs arg;
  
  arg.param("d", dataset, "",
            "Trajectory with few loops used for seeding");
  arg.param("o", refined_dataset, "res.g2o",
            "G2o file with ehnanced loops");
  arg.parseArgs(argc, argv);

  typedef Eigen::Isometry3d PoseType;
  typedef EdgeSE3 EdgeType;
  typedef VertexSE3 VertexType;
  
  vector<PoseType> init_poses;
  vector<VertexType*> v_poses;

  std::cout << "Reading file: " << dataset << std::endl;
  
  // GAUSS-NEWTON CREATION AND SOLVER INITIALIZATION
  SparseOptimizer optimizer;
  optimizer.load(dataset.c_str());
  auto linearSolver = std::make_unique<LinearSolverEigen<BlockSolverX::PoseMatrixType>>();
  linearSolver->setBlockOrdering(false);
  auto blockSolver = std::make_unique<BlockSolverX>(std::move(linearSolver));
  OptimizationAlgorithmGaussNewton *solverGauss = new OptimizationAlgorithmGaussNewton(std::move(blockSolver));
  optimizer.setAlgorithm(solverGauss);
  odometryInitialization<EdgeType, VertexType>(optimizer);

  // GETTING INLIER AND OUTLIER LABELS + SETTING EXPERIMENTS AS IF IT WAS INCREMENTAL EXPERIMENT
  OptimizableGraph::EdgeContainer loop_edges, odom_edges;
  getLoopEdges<EdgeType, VertexType>(optimizer, loop_edges);
  getOdometryEdges<EdgeType, VertexType>(optimizer, odom_edges);
  optimizer.vertex(0)->setFixed(true);
  optimizer.initializeOptimization();

  init_poses.resize(optimizer.vertices().size());
  for ( auto it = optimizer.vertices().begin() ; it != optimizer.vertices().end() ; ++it )
  {
    auto v1 = dynamic_cast<VertexType*>(it->second);
    init_poses[v1->id()]= v1->estimate();
  }

  double w_prior = 0.99;
  Eigen::MatrixXd info0_odom(6, 6); Eigen::MatrixXd sigma0_odom(6, 6); Eigen::MatrixXd vmat_odom(6, 6);
  info0_odom.setZero(); sigma0_odom.setZero(); vmat_odom.setZero();
  info0_odom(0, 0) = 450.0;  // TUM: 100.0  | KITTI_05: 450.0
  info0_odom(1, 1) = 400.0;   // TUM: 80.0  | KITTI_05: 200.0
  info0_odom(2, 2) = 2000.0; // TUM: 2000.0 | KITTI_05: 2000.0
  // Correcting rotation part
  info0_odom(3, 3) = 2000.0; // TUM: 1000.0 | KITTI_05: 2000.0
  info0_odom(4, 4) = 2000.0; // TUM: 1000.0 | KITTI_05: 2000.0
  info0_odom(5, 5) = 250.0;  // TUM: 100.0  | KITTI_05: 100.0
  sigma0_odom = info0_odom.inverse(); 

  Eigen::MatrixXd info0_loop(6, 6); Eigen::MatrixXd sigma0_loop(6, 6); Eigen::MatrixXd vmat_loop(6, 6);
  info0_loop.setZero(); sigma0_loop.setZero(); vmat_loop.setZero();
  info0_loop(0, 0) = 450.0;  // TUM: 100.0  | KITTI_05: 450.0
  info0_loop(1, 1) = 200.0;   // TUM: 80.0  | KITTI_05: 200.0
  info0_loop(2, 2) = 2000.0; // TUM: 2000.0 | KITTI_05: 2000.0
  // Correcting rotation part
  info0_loop(3, 3) = 2000.0; // TUM: 1000.0 | KITTI_05: 2000.0
  info0_loop(4, 4) = 2000.0; // TUM: 1000.0 | KITTI_05: 2000.0
  info0_loop(5, 5) = 100.0;  // TUM: 100.0  | KITTI_05: 100.0
  sigma0_loop = info0_loop.inverse();
  
  double v_odom = 0.0; double v_loop = 0.0;
  wishartPrior(sigma0_odom, w_prior, odom_edges.size(), vmat_odom, v_odom);
  wishartPrior(sigma0_loop, w_prior, loop_edges.size(), vmat_loop, v_loop);
  int m_dof = 6;
    
  for (int it = 0; it < maxIterations; ++it)
  {

    // Step: 1 -> Computing x*
    optimizer.initializeOptimization();

    // Step: 2 -> Computing P*
    Eigen::MatrixXd smat_odom = computeSampleCovariance<EdgeType>(odom_edges);
    Eigen::MatrixXd smat_loop = computeSampleCovariance<EdgeType>(loop_edges);

    Eigen::MatrixXd mmat_odom = (odom_edges.size() * smat_odom + vmat_odom.inverse()) / static_cast<double>(odom_edges.size() + v_odom - m_dof - 1);
    Eigen::MatrixXd mmat_loop = (loop_edges.size() * smat_loop + vmat_loop.inverse()) / static_cast<double>(loop_edges.size() + v_loop - m_dof - 1);

    Eigen::MatrixXd mmat_odom_diag = mmat_odom.diagonal().asDiagonal();
    Eigen::MatrixXd mmat_loop_diag = mmat_loop.diagonal().asDiagonal();

    //std::cout << "TMP =\n" << tmp << std::endl;

    // Step: 3 -> Updating Covariances in Graph
    Eigen::MatrixXd info_odom = mmat_odom_diag.inverse() * 1e-5;
    for (size_t it = 0; it < odom_edges.size(); ++it )
    {
        EdgeType* e = dynamic_cast<EdgeType*>(odom_edges[it]);
        e->setInformation(info_odom);
    }

    Eigen::MatrixXd info_loop = mmat_loop_diag.inverse() * 1e-3; 
    for (size_t it = 0; it < loop_edges.size(); ++it )
    {
        EdgeType* e = dynamic_cast<EdgeType*>(loop_edges[it]);
        e->setInformation(info_loop);
    }

    optimizer.optimize(1);

  }

  auto info_odom = dynamic_cast<EdgeType*>(odom_edges[0])->information();
  auto info_loop = dynamic_cast<EdgeType*>(loop_edges[0])->information();
  std::cout << "Final Estimated Covariance Odom:\n" << info_odom << std::endl; 
  std::cout << "Final Estimated Covariance Loop:\n" << info_loop << std::endl; 

  for (size_t it = 0; it < optimizer.vertices().size(); ++it )
  {
      VertexType* v = dynamic_cast<VertexType*>(optimizer.vertex(it));
      v->setEstimate(init_poses[v->id()]);
  }
  optimizer.save(refined_dataset.c_str());


  return 0;
}
