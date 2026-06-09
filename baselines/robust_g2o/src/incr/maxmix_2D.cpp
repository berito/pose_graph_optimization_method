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
  int batch_size = cfg.batch_size;

  // GAUSS-NEWTON CREATION AND SOLVER INITIALIZATION
  SparseOptimizer optimizer;
  optimizer.load(input_dataset.c_str());
  auto linearSolver = std::make_unique<LinearSolverEigen<BlockSolverX::PoseMatrixType>>();
  linearSolver->setBlockOrdering(false);
  auto blockSolver = std::make_unique<BlockSolverX>(std::move(linearSolver));
  //OptimizationAlgorithmGaussNewton *solverGauss = new OptimizationAlgorithmGaussNewton(std::move(blockSolver));
  OptimizationAlgorithmLevenberg *solverGauss = new OptimizationAlgorithmLevenberg(std::move(blockSolver));
  optimizer.setAlgorithm(solverGauss);

  // GETTING INLIER AND OUTLIER LABELS + SETTING EXPERIMENTS AS IF IT WAS INCREMENTAL EXPERIMENT
  OptimizableGraph::EdgeContainer loop_edges, odom_edges;
  getLoopEdges<EdgeSE2, VertexSE2>(optimizer, loop_edges);
  getOdometryEdges<EdgeSE2, VertexSE2>(optimizer, odom_edges);
  
  // AUGMENTING THE BASE PGO PROBLEM WIH MAXMIX CONSTRAINTS
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
  }

  vector<pair<bool, OptimizableGraph::Edge*>> loops_w_label;
  for (size_t idx = 0 ; idx < cfg.canonic_inliers; loops_w_label.push_back(make_pair(true, loops_mixture[idx++])));
  for (size_t idx = cfg.canonic_inliers ; idx < loop_edges.size(); loops_w_label.push_back(make_pair(false, loops_mixture[idx++])));
  sort(loops_w_label.begin(), loops_w_label.end(), cmpTime);

  // FIXING ORIGIN OF THE MAP AT 0
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
    optimizer.optimize(maxIterations);
    propagateCurrentGuess<EdgeSE2, VertexSE2>(optimizer, last_odom_idx, odom_edges);
    chrono::steady_clock::time_point end = chrono::steady_clock::now();
    chrono::microseconds delta_time = chrono::duration_cast<chrono::microseconds>(end - begin);
    avg_time += delta_time.count() / 1000000.0;
    ++n_optimization;

    printProgress((double)(e_it + 1) / (double)loops_w_label.size());
  }
  cout << endl;


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

  eset_opt.clear();
  for (size_t i = 0 ; i < inliers_edges.size() ; eset_opt.insert(loop_edges[inliers_edges[i++]]));
  for (size_t i = 0 ; i < odom_edges.size() ; eset_opt.insert(odom_edges[i++]));
  
  std::cout << "Starting Refinment : " << std::endl;
  optimizer.initializeOptimization(eset_opt);
  optimizer.optimize(100);

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


  std::cout << "Total Time of Completion " << avg_time << " [s]" << endl;
  std::cout << "Avg time per batch " << avg_time / n_optimization << " [s]" << endl;
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
  outfile << avg_time << " " << avg_time / n_optimization << endl;
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
