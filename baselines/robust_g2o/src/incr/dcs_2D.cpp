#include "utils.hpp"
#include "dcs_kernel_impl.hpp"

#include "g2o/core/robust_kernel_impl.h"
#include <boost/math/distributions/chi_squared.hpp>
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"


// we use the 2D and 3D SLAM types here
G2O_USE_TYPE_GROUP(slam2d);
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

  // CONFIGURATION PARAMETERS
  Config cfg;
  readConfig(cfg_file, cfg);
  string input_dataset = cfg.dataset;
  int maxIterations = cfg.maxiters;
  double information_edge = cfg.switch_prior;
  double inlier_th = cfg.inlier_th;
  int inliers = cfg.canonic_inliers;
  int batch_size = cfg.batch_size;

  // GAUSS-NEWTON CREATION AND SOLVER INITIALIZATION
  SparseOptimizer optimizer;
  optimizer.load(input_dataset.c_str());
  auto linearSolver = std::make_unique<LinearSolverEigen<BlockSolverX::PoseMatrixType>>();
  linearSolver->setBlockOrdering(false);
  auto blockSolver = std::make_unique<BlockSolverX>(std::move(linearSolver));
  OptimizationAlgorithmGaussNewton *solverGauss = new OptimizationAlgorithmGaussNewton(std::move(blockSolver));
  optimizer.setAlgorithm(solverGauss);
  odometryInitialization<EdgeSE2, VertexSE2>(optimizer);

  // GETTING INLIER AND OUTLIER LABELS + SETTING EXPERIMENTS AS IF IT WAS INCREMENTAL EXPERIMENT
  OptimizableGraph::EdgeContainer loop_edges, odom_edges;
  getLoopEdges<EdgeSE2, VertexSE2>(optimizer, loop_edges);
  getOdometryEdges<EdgeSE2, VertexSE2>(optimizer, odom_edges);
  vector<pair<bool, OptimizableGraph::Edge*>> loops_w_label;
  for (size_t idx = 0 ; idx < cfg.canonic_inliers; loops_w_label.push_back(make_pair(true, loop_edges[idx++])));
  for (size_t idx = cfg.canonic_inliers ; idx < loop_edges.size(); loops_w_label.push_back(make_pair(false, loop_edges[idx++])));
  sort(loops_w_label.begin(), loops_w_label.end(), cmpTime);
  
  // ADDING ROBUST-KERNEL TO ALL THE LOOP EDGES
  double chi2_th = Chi2inv(inlier_th, 3); // 95% quantile of chi2 with 3 dofs
  for ( size_t id_e = 0; id_e < loop_edges.size(); ++id_e )
  {
    EdgeSE2* e = static_cast<EdgeSE2*>(loop_edges[id_e]);

    // Add robust kernel DCS
    RobustDCSX* rk = new RobustDCSX;
    rk->setDelta(chi2_th);
    e->setRobustKernel(rk);
  }
  
  // FIXING ORIGIN OF THE MAP AT 0
  optimizer.vertex(0)->setFixed(true);

  // INCREMENTAL EXPERIMENT
  int last_odom_idx = 0;
  OptimizableGraph::EdgeSet eset_opt;
  double avg_time = 0.0; int n_optimization = 0;
  cout << "TH SET AS: " << chi2_th << endl;
  for ( size_t e_it = 0 ; e_it < loops_w_label.size() ; ++e_it )
  {
    int max_vid = 0;
    for (size_t batch_it = 0; batch_it < batch_size && e_it + batch_it < loops_w_label.size() ; ++batch_it)
    {
      OptimizableGraph::Edge* el = loops_w_label[e_it + batch_it].second;
      int v0 = el->vertices()[0]->id();
      int v1 = el->vertices()[1]->id();
      max_vid = max(max_vid, v0);
      max_vid = max(max_vid, v1);
      eset_opt.insert(el);
    }
    e_it += batch_size - 1;
    for (size_t eo_it = last_odom_idx ; eo_it < max_vid ; eset_opt.insert(odom_edges[eo_it++]));
    last_odom_idx = max_vid;

    // Optimizing the problem until the last vertex
    optimizer.initializeOptimization(eset_opt);
    chrono::steady_clock::time_point begin = chrono::steady_clock::now();
    optimizer.optimize(maxIterations);
    propagateCurrentGuess<EdgeSE2, VertexSE2>(optimizer, last_odom_idx, odom_edges);
    chrono::steady_clock::time_point end = chrono::steady_clock::now();
    chrono::microseconds delta_time = chrono::duration_cast<chrono::microseconds>(end - begin);
    avg_time += delta_time.count() / 1000000.0;
    ++n_optimization;

    printProgress((double)(e_it + 1) / (double)loops_w_label.size());
  }
  cout << endl;

  // EVALUATION OF THE METHOD
  int tp  = 0; int tn = 0; int fp  = 0; int fn = 0;
  for (int idx = 0; idx < inliers; ++idx)
  {
    OptimizableGraph::Edge* e = loop_edges[idx];
    RobustKernel* rk = e->robustKernel();
    Vector3 rho;
    rk->robustify(e->chi2(), rho);
    double weight = rho[1];

    if (weight >= 0.95) ++tp;
    else ++fn;
  }

  for ( int idx = inliers; idx < loop_edges.size(); ++idx)
  {
    OptimizableGraph::Edge* e = loop_edges[idx];
    RobustKernel* rk = e->robustKernel();
    Vector3 rho;
    rk->robustify(e->chi2(), rho);
    double weight = rho[1];

    if (weight >= 0.95) ++fp;
    else ++tn;  
  }

  float precision = tp + fp > 0 ? tp / (float)(tp + fp) : 0.0;
  float recall    = tp + fn > 0 ? tp / (float)(tp + fn) : 0.0; 

  cout << "Total Time of Completion " << avg_time << " [s]" << endl;
  cout << "Avg time per batch " << avg_time / n_optimization << " [s]" << endl;
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
      VertexSE2* v = dynamic_cast<VertexSE2*>(optimizer.vertex(it));
      
      if ( v == nullptr ) continue;

      writeVertex(outfile, v);
  }
  outfile.close();

  string output_file_pr = output_file_trj.substr(0, output_file_trj.size() - 3) + "PR";
  outfile.open(output_file_pr.c_str());
  outfile << precision << " " << recall << endl;
  outfile << avg_time << " " << avg_time / n_optimization << endl;
  outfile.close();

  return 0;
}