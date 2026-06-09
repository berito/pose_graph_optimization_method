#include "utils.hpp"
#include "g2o/core/robust_kernel_impl.h"
#include "dcs_kernel_impl.hpp"
#include <boost/math/distributions/chi_squared.hpp>
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"


// we use the 2D and 3D SLAM types here
G2O_USE_TYPE_GROUP(slam3d);
G2O_USE_OPTIMIZATION_LIBRARY(eigen);

using namespace g2o;
using namespace std;

double Chi2inv(const double alpha, const size_t dofs) 
{
  boost::math::chi_squared_distribution<double> chi2(dofs);
  return boost::math::quantile(chi2, alpha);
}

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
  double information_edge = cfg.switch_prior;
  double inlier_th = cfg.inlier_th;
  int inliers = cfg.canonic_inliers;


  auto linearSolver = std::make_unique<LinearSolverEigen<BlockSolverX::PoseMatrixType>>();
  linearSolver->setBlockOrdering(false);
  auto blockSolver = std::make_unique<BlockSolverX>(std::move(linearSolver));
  OptimizationAlgorithmGaussNewton *solverGauss = new OptimizationAlgorithmGaussNewton(std::move(blockSolver));

  // create the optimizer to load the data and carry out the optimization
  OptimizableGraph::EdgeContainer loop_edges;
  SparseOptimizer optimizer;
  optimizer.load(input_dataset.c_str());
  optimizer.setAlgorithm(solverGauss);
  
  odometryInitialization<EdgeSE3, VertexSE3>(optimizer);
  getLoopEdges<EdgeSE3, VertexSE3>(optimizer, loop_edges);

  double chi2_th = Chi2inv(inlier_th, 6); // 95% quantile of chi2 with 3 dofs
  for ( size_t id_e = 0; id_e < loop_edges.size(); ++id_e )
  {
      EdgeSE3* e = static_cast<EdgeSE3*>(loop_edges[id_e]);

      // Add robust kernel DCS
      RobustDCSX* rk = new RobustDCSX;
      rk->setDelta(chi2_th);
      e->setRobustKernel(rk);
  }

  // Optimize the problem
  cout << "Starting optimization : " << endl;
  optimizer.initializeOptimization();
  optimizer.vertex(0)->setFixed(true);
  chrono::steady_clock::time_point begin = chrono::steady_clock::now();
  optimizer.optimize(maxIterations);
  chrono::steady_clock::time_point end = chrono::steady_clock::now();
  chrono::microseconds delta_time = chrono::duration_cast<chrono::microseconds>(end - begin);

  // Updating values 
  int tp  = 0; int tn = 0;
  int fp  = 0; int fn = 0;
  for (int idx = 0; idx < inliers; ++idx)
  {
    if (loop_edges[idx]->chi2() < chi2_th) ++tp;
    else ++fn;
  }

  for ( int idx = inliers; idx < loop_edges.size(); ++idx)
  {
    if (loop_edges[idx]->chi2() < chi2_th) ++fp;
    else ++tn;  
  }

  float precision = tp + fp > 0 ? tp / (float)(tp + fp) : 0.0;
  float recall    = tp + fn > 0 ? tp / (float)(tp + fn) : 0.0; 
  float dt = delta_time.count() / 1000000.0;

  cout << "Optimization complete in " << dt << " [s]" << endl;
  cout << "Precision  = " << precision << endl;
  cout << "Recall = " << recall << endl;
  cout << "TP = " << tp << endl;
  cout << "TN = " << tn << endl;
  cout << "FP = " << fp << endl;
  cout << "FN = " << fn << endl;
  
  cout << "Optimtization Concluded!" << endl;

  ofstream outfile;
  string output_file_trj = cfg.output;
  outfile.open(output_file_trj.c_str()); 
  for (size_t it = 0; it < optimizer.vertices().size(); ++it )
  { 
      VertexSE3* v = dynamic_cast<VertexSE3*>(optimizer.vertex(it));
      
      if ( v == nullptr ) continue;

      writeVertex(outfile, v);
  }
  outfile.close();

  string output_file_pr = output_file_trj.substr(0, output_file_trj.size() - 3) + "PR";
  outfile.open(output_file_pr.c_str());
  outfile << precision << " " << recall << endl;
  outfile << dt << endl;
  outfile.close();

  return 0;
}