#include "utils.hpp"
#include "graduatedNonConvexity/include/gnc_sparse_optimizer.hpp"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"

using namespace std;
using namespace g2o;

// we use the 2D and 3D SLAM types here
G2O_USE_TYPE_GROUP(slam2d);
G2O_USE_OPTIMIZATION_LIBRARY(eigen);

int main(int argc, char** argv) 
{

  // Command line parsing
  string cfg_file;
  CommandArgs arg;
  
  arg.param("cfg", cfg_file, "",
            "Configuration File(.yaml)");
  arg.parseArgs(argc, argv);

  
  // Storing initial guess and vertices of optimization
  vector<SE2> init_poses;
  vector<VertexSE2*> v_poses;

  Config cfg;
  readConfig(cfg_file, cfg);
  string input_dataset = cfg.dataset;
  int maxIterations = cfg.maxiters;
  double inlier_th = cfg.inlier_th;
  int inliers = cfg.canonic_inliers;

  // GNC Specific Parameters
  /**/
  int inner_iterations = 10;
  double mu_step = 1.4;

  // For GNC I use the Gauss-Newton optimizer
  auto linearSolver = std::make_unique<LinearSolverEigen<BlockSolverX::PoseMatrixType>>();
	linearSolver->setBlockOrdering(false);
	auto blockSolver = std::make_unique<BlockSolverX>(std::move(linearSolver));
	OptimizationAlgorithmGaussNewton *solverGauss = new OptimizationAlgorithmGaussNewton(std::move(blockSolver));
	
  // Create GNC optimizer
  GNCSparseOptimizer optimizer(GncLossType::TLS);
  optimizer.setAlgorithm(solverGauss);
  optimizer.load(input_dataset.c_str());
  odometryInitialization<EdgeSE2, VertexSE2>(optimizer);
  optimizer.vertex(0)->setFixed(true);
  optimizer.initializeOptimization();

  /* Set GNC parameters */
  std::cout << "GNC parameters: inlier_th = " << inlier_th 
            << ", inner_iterations = " << inner_iterations
            << ", mu_step = " << mu_step << std::endl;

  optimizer.setAlpha(inlier_th);
  optimizer.setInnerIterations(inner_iterations);
  optimizer.setMuStep(mu_step);

  // Set odometry edges as inliers
  OptimizableGraph::EdgeContainer odometry_edges, loop_edges;
  getOdometryEdges<EdgeSE2, VertexSE2>(optimizer, odometry_edges);
  int n_loops = optimizer.edges().size() - odometry_edges.size();
  optimizer.setKnownInliers(odometry_edges);
  /* Set GNC parameters */

  optimizer.computeActiveErrors();
  double initial_chi2 = optimizer.activeChi2();

  std::cout << "Starting optimization : " << std::endl;
  std::cout << "Initial chi2: " << initial_chi2 << std::endl;
  chrono::steady_clock::time_point begin = chrono::steady_clock::now();
  optimizer.setVerbose(true);
  int total_iterations = optimizer.optimize(maxIterations, false);
  chrono::steady_clock::time_point end = chrono::steady_clock::now();
  chrono::microseconds delta_time = chrono::duration_cast<chrono::microseconds>(end - begin);

  /* Check inliers/outlier state */
  int tp  = 0; int tn = 0; int fp  = 0; int fn = 0; 
  for ( size_t idx = 0; idx < inliers; ++idx)
  {
    if ( optimizer.isEdgeInlier(idx) ) ++tp;
    else ++fn;
  }

  for ( size_t idx = inliers; idx < n_loops; ++idx)
  {
    if ( !optimizer.isEdgeInlier(idx) ) ++tn;
    else ++fp;
  }

  std::cout << "Optimization Concluded in: " << total_iterations << std::endl;
  std::cout << "Time taken for optimization: " << delta_time.count() / 1000000.0 << " seconds" << std::endl;
  std::cout << "Final chi2: " << optimizer.activeRobustChi2() << std::endl;

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

  std::cout << "Loops size = " << n_loops << std::endl;
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
 
  optimizer.clear();
 
  return 0;
}
