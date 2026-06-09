#include "utils.hpp"
#include "switchableConstraints/include/edge_switchPrior.hpp"
#include "switchableConstraints/include/edge_se2Switchable.hpp"
#include "switchableConstraints/include/vertex_switchLinear.hpp"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_gauss_newton.h"
#include "g2o/solvers/eigen/linear_solver_eigen.h"

using namespace std;
using namespace g2o;

G2O_USE_OPTIMIZATION_LIBRARY(eigen);

EdgeSE2Switchable* getSwitchableEdge(g2o::EdgeSE2* candidate, VertexSwitchLinear* sw);

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

  // AUGMENTING THE BASE PGO PROBLEM WIH SWITCHABLE CONSTRAINTS
  int id_swv = optimizer.vertices().size() + 300;
  std::vector<EdgeSE2Switchable*> sw_contraints;
  std::vector<EdgeSwitchPrior*> sw_priors;
  std::vector<VertexSwitchLinear*> sw_vs;
  for ( auto it_e = loops_w_label.begin(); it_e != loops_w_label.end(); ++it_e )
  {
    // Create a switchable vertex
    EdgeSE2* el = dynamic_cast<EdgeSE2*>((*it_e).second);
    VertexSwitchLinear* sw = new VertexSwitchLinear();
    sw->setEstimate(1.0);
    sw->setFixed(false);
    sw->setId(id_swv);

    // Based on SC paper the prior is set to 1.0
    EdgeSwitchPrior* sw_prior = new EdgeSwitchPrior();
    sw_prior->setVertex(0, sw);
    Eigen::Matrix<double, 1, 1> information_mat = Eigen::Matrix<double, 1, 1>::Identity();
    sw_prior->setMeasurement(1.0);
    sw_prior->setInformation(information_mat * information_edge);      
        
    // Switchable edge
    EdgeSE2Switchable* sw_constraint = getSwitchableEdge(el, sw);

    // Remove later the originals and and the new ones 
    sw_contraints.emplace_back(sw_constraint);
    sw_priors.emplace_back(sw_prior);
    sw_vs.emplace_back(sw);

    optimizer.addVertex(sw);
    optimizer.addEdge(sw_constraint);
    optimizer.addEdge(sw_prior);

    ++id_swv;
  }

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
      eset_opt.insert(sw_contraints[e_it + batch_it]);
      eset_opt.insert(sw_priors[e_it + batch_it]);
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
  for ( size_t it = 0 ; it < loops_w_label.size() ; ++it )
    if ( loops_w_label[it].first && sw_vs[it]->estimate() > inlier_th ) ++tp;
    else if ( loops_w_label[it].first && sw_vs[it]->estimate() <= inlier_th ) ++fn;
    else if ( !loops_w_label[it].first && sw_vs[it]->estimate() > inlier_th ) ++fp;
    else if ( !loops_w_label[it].first && sw_vs[it]->estimate() <= inlier_th ) ++tn;

  float precision = tp + fp > 0 ? tp / (float)(tp + fp) : 0.0;
  float recall    = tp + fn > 0 ? tp / (float)(tp + fn) : 0.0; 

  std::cout << "Total Time of Completion " << avg_time << " [s]" << std::endl;
  std::cout << "Avg time per batch " << avg_time / n_optimization << " [s]" << std::endl;
  std::cout << "Precision  = " << precision << std::endl;
  std::cout << "Recall = " << recall << std::endl;
  std::cout << "TP = " << tp << std::endl;
  std::cout << "TN = " << tn << std::endl;
  std::cout << "FP = " << fp << std::endl;
  std::cout << "FN = " << fn << std::endl;
  
  std::cout << "Optimtization Concluded!" << std::endl;

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

EdgeSE2Switchable* getSwitchableEdge(EdgeSE2* candidate, VertexSwitchLinear* sw)
{
    EdgeSE2Switchable* edge = new EdgeSE2Switchable();
    edge->setVertex(0, candidate->vertex(0));
    edge->setVertex(1, candidate->vertex(1));
    edge->setVertex(2, sw);
    edge->setMeasurement(candidate->measurement());
    edge->setInformation(candidate->information());

    return edge;
}