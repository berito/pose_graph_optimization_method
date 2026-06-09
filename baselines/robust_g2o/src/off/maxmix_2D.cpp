#include "utils.hpp"
#include "maxmixConstraints/include/edge_se2_mixture.hpp"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"

using namespace std;
using namespace g2o;

// we use the 2D and 3D SLAM types here
G2O_USE_TYPE_GROUP(slam2d);
G2O_USE_OPTIMIZATION_LIBRARY(eigen);

void initializeMixture(EdgeSE2Mixture* e_maxmix, EdgeSE2* e_odom);

int main(int argc, char** argv) 
{

  // Command line parsing
  string cfg_file;
  CommandArgs arg;
  
  arg.param("cfg", cfg_file, "",
            "Configuration File(.yaml)");
  arg.parseArgs(argc, argv);

  // Storing initial guess and vertices of optimization
  Config cfg;
  readConfig(cfg_file, cfg);
  string input_dataset = cfg.dataset;
  int maxIterations = cfg.maxiters;
  int inliers = cfg.canonic_inliers;
  double maxmix_weight = cfg.maxmix_weight;
  double nu_constraints = cfg.nu_constraints;
  double nu_nullHypothesis = cfg.nu_nullHypothesis;

  auto linearSolver = std::make_unique<LinearSolverEigen<BlockSolverX::PoseMatrixType>>();
  linearSolver->setBlockOrdering(false);
  auto blockSolver = std::make_unique<BlockSolverX>(std::move(linearSolver));
  //OptimizationAlgorithmGaussNewton *solverGauss = new OptimizationAlgorithmGaussNewton(std::move(blockSolver));
  OptimizationAlgorithmLevenberg *solverGauss = new OptimizationAlgorithmLevenberg(std::move(blockSolver));

  // create the optimizer to load the data and carry out the optimization
  OptimizableGraph::EdgeContainer loop_edges;
  SparseOptimizer optimizer;
  optimizer.load(input_dataset.c_str());
  optimizer.setAlgorithm(solverGauss);
  odometryInitialization<EdgeSE2, VertexSE2>(optimizer);
  getLoopEdges<EdgeSE2, VertexSE2>(optimizer, loop_edges);

  // Storing initial guess
  std::vector<SE2> v_poses(optimizer.vertices().size());
  for (size_t it = 0; it < optimizer.vertices().size(); ++it )
  {
      VertexSE2* v = dynamic_cast<VertexSE2*>(optimizer.vertex(it));
      v_poses[it] = SE2(v->estimate());
  }

  for ( auto it_e = optimizer.edges().begin(); it_e != optimizer.edges().end(); ++it_e )
  {
    EdgeSE2* edge_odom = dynamic_cast<EdgeSE2*>(*it_e);
    edge_odom->setLevel(0);
  }

  std::vector<EdgeSE2*> loops_orig;
  std::vector<EdgeSE2Mixture*> loops_mixture;
  for ( auto it_e = loop_edges.begin(); it_e != loop_edges.end(); ++it_e )
  {
    EdgeSE2* edge_loop = static_cast<EdgeSE2*>(*it_e);
    
    EdgeSE2Mixture* e_maxmix = new EdgeSE2Mixture();
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
  optimizer.vertex(0)->setFixed(true);
  optimizer.initializeOptimization(0);
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
      VertexSE2* v = dynamic_cast<VertexSE2*>(optimizer.vertex(it));
      v->setEstimate(v_poses[it]);
  }

  optimizer.setVerbose(false);
  std::cout << "Starting Refinment : " << std::endl;
  optimizer.initializeOptimization(0);
  optimizer.computeActiveErrors();
  optimizer.optimize(maxIterations);


  ofstream outfile;
  string output_file_trj = cfg.output;
  outfile.open(output_file_trj.c_str()); 
  for (size_t it = 0; it < optimizer.vertices().size(); ++it )
  {
      VertexSE2* v = dynamic_cast<VertexSE2*>(optimizer.vertex(it));
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


void initializeMixture(EdgeSE2Mixture* e_maxmix, EdgeSE2* e_odom)
{
  double m[3];
  e_odom->getMeasurementData(m);
  e_maxmix->setMeasurement(SE2(m[0], m[1], m[2]));
  e_maxmix->information_nullHypothesis = Eigen::Matrix3d::Identity();

  return;
}
