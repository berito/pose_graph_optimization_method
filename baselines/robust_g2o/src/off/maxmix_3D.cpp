#include "utils.hpp"
#include "maxmixConstraints/include/edge_se3_mixture.hpp"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"

using namespace std;
using namespace g2o;

// we use the 2D and 3D SLAM types here
G2O_USE_TYPE_GROUP(slam3d);
G2O_USE_OPTIMIZATION_LIBRARY(eigen);

void initializeMixture(EdgeSE3Mixture* e_maxmix, EdgeSE3* e_odom);

int main(int argc, char** argv) 
{
  // Command line parsing
  string cfg_file;
  CommandArgs arg;
  
  arg.param("cfg", cfg_file, "",
            "Configuration File(.yaml)");
  arg.parseArgs(argc, argv);

  Config cfg;
  readConfig(cfg_file, cfg);
  string input_dataset = cfg.dataset;
  int maxIterations = cfg.maxiters;
  double inlier_th = cfg.inlier_th;
  int inliers = cfg.canonic_inliers;
  double maxmix_weight = cfg.maxmix_weight;
  double nu_constraints = cfg.nu_constraints;
  double nu_nullHypothesis = cfg.nu_nullHypothesis;

  // create the optimizer to load the data and carry out the optimization
  auto linearSolver = std::make_unique<LinearSolverEigen<BlockSolverX::PoseMatrixType>>();
  linearSolver->setBlockOrdering(false);
  auto blockSolver = std::make_unique<BlockSolverX>(std::move(linearSolver));
  OptimizationAlgorithmLevenberg *solverGauss = new OptimizationAlgorithmLevenberg(std::move(blockSolver));

  // create the optimizer to load the data and carry out the optimization
  OptimizableGraph::EdgeContainer loop_edges;
  SparseOptimizer optimizer;
  optimizer.load(input_dataset.c_str());
  optimizer.setAlgorithm(solverGauss);
  odometryInitialization<EdgeSE3, VertexSE3>(optimizer);
  getLoopEdges<EdgeSE3, VertexSE3>(optimizer, loop_edges);

  // Storing initial guess
  vector<Eigen::Isometry3d> v_poses(optimizer.vertices().size());
  for (size_t it = 0; it < optimizer.vertices().size(); ++it )
  {
      VertexSE3* v = dynamic_cast<VertexSE3*>(optimizer.vertex(it));
      v_poses[it] = Eigen::Isometry3d(v->estimate());
  }

  for ( auto it_e = optimizer.edges().begin(); it_e != optimizer.edges().end(); ++it_e )
  {
    EdgeSE3* edge_odom = dynamic_cast<EdgeSE3*>(*it_e);
    edge_odom->setLevel(0);
  }
  
  /**/
  std::vector<EdgeSE3*> loops_orig;
  std::vector<EdgeSE3Mixture*> loops_mixture;
  for ( auto it_e = loop_edges.begin(); it_e != loop_edges.end(); ++it_e )
  {
    EdgeSE3* edge_loop = dynamic_cast<EdgeSE3*>(*it_e);
    EdgeSE3Mixture* e_maxmix = new EdgeSE3Mixture();
    e_maxmix->setVertex(0, edge_loop->vertices()[0]);
    e_maxmix->setVertex(1, edge_loop->vertices()[1]);
    initializeMixture(e_maxmix, edge_loop);
    e_maxmix->setInformation(edge_loop->information());
    e_maxmix->weight = maxmix_weight;
    e_maxmix->information_constraint = edge_loop->information();
    e_maxmix->nu_constraint = nu_constraints;
    e_maxmix->nu_nullHypothesis = nu_nullHypothesis;

    optimizer.addEdge(e_maxmix);

    loops_mixture.push_back(e_maxmix);
    loops_orig.push_back(edge_loop);
    edge_loop->setLevel(1);
    e_maxmix->setLevel(0);

  }

  std::cout << "Starting optimization : " << std::endl;
  optimizer.initializeOptimization(0);
  optimizer.vertex(0)->setFixed(true);
  chrono::steady_clock::time_point begin = chrono::steady_clock::now();
  optimizer.optimize(maxIterations);
  chrono::steady_clock::time_point end = chrono::steady_clock::now();
  chrono::microseconds delta_time = chrono::duration_cast<chrono::microseconds>(end - begin);

  int tp  = 0; int tn = 0; int fp  = 0; int fn = 0; 
  std::vector<int> inliers_edges; 
  for ( size_t idx = 0; idx < inliers; ++idx)
  {
    if ( !loops_mixture[idx]->isOutlier() ) 
    {
      ++tp;
      inliers_edges.push_back(idx);
    }
    else ++fn;
  }

  for ( size_t idx = inliers; idx < loops_mixture.size(); ++idx)
  {
    if ( loops_mixture[idx]->isOutlier() ) ++tn;
    else
    {
      ++fp;
      inliers_edges.push_back(idx);
    } 
  }
  
  for (size_t i = 0 ; i < inliers_edges.size() ; loops_orig[inliers_edges[i++]]->setLevel(0) );
  for (size_t i = 0 ; i < loops_mixture.size() ; optimizer.removeEdge(loops_mixture[i++]));

  // Restoring initial guess
  for (size_t it = 0; it < optimizer.vertices().size(); ++it )
  {
      VertexSE3* v = dynamic_cast<VertexSE3*>(optimizer.vertex(it));
      v->setEstimate(v_poses[it]);
  }

  optimizer.setVerbose(false);
  std::cout << "Starting Refinment : " << std::endl;
  optimizer.initializeOptimization(0);
  optimizer.computeActiveErrors();
  optimizer.optimize(maxIterations);


  std::cout << "Optimtization Concluded!" << std::endl;

  ofstream outfile;
  string output_file_trj = cfg.output;
  outfile.open(output_file_trj.c_str()); 
  for (size_t it = 0; it < optimizer.vertices().size(); ++it )
  {
      VertexSE3* v = dynamic_cast<VertexSE3*>(optimizer.vertex(it));
      writeVertex(outfile, v);
  }
  outfile.close();

  float precision = tp + fp > 0 ? tp / (float)(tp + fp) : 0.0;
  float recall    = tp + fn > 0 ? tp / (float)(tp + fn) : 0.0; 
  float dt = delta_time.count() / 1000000.0;

  std::cout << "Loops size = " << loops_mixture.size() << std::endl;
  std::cout << "Canonic Inliers = " << cfg.canonic_inliers << std::endl;
  std::cout << "Prec = " << precision << std::endl;
  std::cout << "Rec = " << recall << std::endl;
  std::cout << "TP = " << tp << std::endl;
  std::cout << "TN = " << tn << std::endl;
  std::cout << "FP = " << fp << std::endl;
  std::cout << "FN = " << fn << std::endl;


  string output_file_pr = output_file_trj.substr(0, output_file_trj.size() - 3) + "PR";
  outfile.open(output_file_pr.c_str());
  outfile << precision << " " << recall << endl;
  outfile << dt << endl;
  outfile.close();

  return 0;
}

void initializeMixture(EdgeSE3Mixture* e_maxmix, EdgeSE3* e_odom)
{
  typedef Eigen::Matrix<double, 6, 6> Matrix6d;

  double m[7];
  e_odom->getMeasurementData(m);
  Eigen::Map<const Vector7> v(m);
  e_maxmix->setMeasurement(internal::fromVectorQT(v));
  e_maxmix->information_nullHypothesis = Matrix6d::Identity();

  return;
}