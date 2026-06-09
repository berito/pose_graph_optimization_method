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
G2O_USE_TYPE_GROUP(slam3d);
G2O_USE_OPTIMIZATION_LIBRARY(eigen);

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
  int batch_size = cfg.batch_size;

  // PARAMETERS GNC
  int inner_iterations = 10;
  double mu_step = 1.4;
  std::cout << "GNC parameters: inlier_th = " << inlier_th 
            << ", inner_iterations = " << inner_iterations
            << ", mu_step = " << mu_step << std::endl;

  // GNC WITH GAUSS-NEWTON OPTIMIZER + INITIALIZATION
  auto linearSolver = std::make_unique<LinearSolverEigen<BlockSolverX::PoseMatrixType>>();
	linearSolver->setBlockOrdering(false);
	auto blockSolver = std::make_unique<BlockSolverX>(std::move(linearSolver));
	OptimizationAlgorithmGaussNewton *solverGauss = new OptimizationAlgorithmGaussNewton(std::move(blockSolver));
  GNCSparseOptimizer optimizer(GncLossType::TLS);
  optimizer.setAlgorithm(solverGauss);
  optimizer.load(input_dataset.c_str());
  optimizer.setAlpha(inlier_th);
  optimizer.setInnerIterations(inner_iterations);
  optimizer.setMuStep(mu_step);

  // GETTING INLIER AND OUTLIER LABELS + SETTING EXPERIMENTS AS IF IT WAS INCREMENTAL EXPERIMENT
  OptimizableGraph::EdgeContainer loop_edges, odom_edges;
  getLoopEdges<EdgeSE3, VertexSE3>(optimizer, loop_edges);
  getOdometryEdges<EdgeSE3, VertexSE3>(optimizer, odom_edges);
  vector<pair<bool, OptimizableGraph::Edge*>> loops_w_label;
  for (size_t idx = 0 ; idx < cfg.canonic_inliers; loops_w_label.push_back(make_pair(true, loop_edges[idx++])));
  for (size_t idx = cfg.canonic_inliers ; idx < loop_edges.size(); loops_w_label.push_back(make_pair(false, loop_edges[idx++])));
  sort(loops_w_label.begin(), loops_w_label.end(), cmpTime);
  
  // FIXING ORIGIN OF THE MAP AT 0 + KNOWN INLIERS
  optimizer.initializeOptimization();
  optimizer.setKnownInliers(odom_edges);
  optimizer.vertex(0)->setFixed(true);

  // INCREMENTAL EXPERIMENT
  int last_odom_idx = 0;
  OptimizableGraph::EdgeSet eset_opt;
  double avg_time = 0.0; int n_optimization = 0;
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
    optimizer.optimizeX(maxIterations, false);
    propagateCurrentGuess<EdgeSE3, VertexSE3>(optimizer, last_odom_idx, odom_edges);
    chrono::steady_clock::time_point end = chrono::steady_clock::now();
    chrono::microseconds delta_time = chrono::duration_cast<chrono::microseconds>(end - begin);
    avg_time += delta_time.count() / 1000000.0;
    ++n_optimization;

    printProgress((double)(e_it + 1) / (double)loops_w_label.size());
  }
  cout << endl;

  /* Check inliers/outlier state */
  int tp  = 0; int tn = 0; int fp  = 0; int fn = 0; 
  for ( size_t idx = 0; idx < inliers; ++idx)
  {
    if ( optimizer.isEdgeInlier(idx) ) ++tp;
    else ++fn;
  }

  for ( size_t idx = inliers; idx < loop_edges.size(); ++idx)
  {
    if ( !optimizer.isEdgeInlier(idx) ) ++tn;
    else ++fp;
  }

  std::cout << "Total Time of Completion " << avg_time << " [s]" << std::endl;
  std::cout << "Avg time per batch " << avg_time / n_optimization << " [s]" << std::endl;
  std::cout << "Final chi2: " << optimizer.activeRobustChi2() << std::endl;

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

  std::cout << "Loops size = " << loop_edges.size() << std::endl;
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
  outfile << avg_time << " " << avg_time / n_optimization << endl;
  outfile.close();
 
  optimizer.clear();
 
  return 0;
}