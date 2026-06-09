#include "utils.hpp"
#include "switchableConstraints/include/edge_switchPrior.hpp"
#include "switchableConstraints/include/edge_se3Switchable.hpp"
#include "switchableConstraints/include/vertex_switchLinear.hpp"
#include "g2o/types/slam3d/isometry3d_gradients.h"

using namespace std;
using namespace g2o;

G2O_USE_OPTIMIZATION_LIBRARY(eigen);

EdgeSE3Switchable* getSwitchableEdge(g2o::EdgeSE3* candidate, VertexSwitchLinear* sw);

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

  // Storing initial guess and vertices of optimization
  vector<Eigen::Isometry3d> init_poses;
  vector<VertexSE3*> v_poses;

  // create the optimizer to load the data and carry out the optimization
  SparseOptimizer optimizer;
  setProblem<Eigen::Isometry3d, EdgeSE3, VertexSE3>(cfg.dataset, optimizer, init_poses, v_poses);
  int id_swv = optimizer.vertices().size() + 300;

  std::vector<EdgeSE3*> e2remove;
  std::vector<EdgeSE3Switchable*> e2add_loops;
  std::vector<EdgeSwitchPrior*> e2add_priors;
  std::vector<VertexSwitchLinear*> sw_vs;
  for ( auto it_e = optimizer.edges().begin(); it_e != optimizer.edges().end(); ++it_e )
  {
    EdgeSE3* edge_odom = dynamic_cast<EdgeSE3*>(*it_e);

    /* Odom edge loop closure */
    if ( edge_odom != nullptr && abs(edge_odom->vertices()[1]->id() - edge_odom->vertices()[0]->id()) > 1  )
    {
        // Create a switchable vertex
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
        EdgeSE3Switchable* sw_constraint = getSwitchableEdge(edge_odom, sw);

        // Remove later the originals and and the new ones 
        e2remove.emplace_back(edge_odom);
        e2add_loops.emplace_back(sw_constraint);
        e2add_priors.emplace_back(sw_prior);
        sw_vs.emplace_back(sw);

        ++id_swv;
    }
  }
  
  for ( int i = 0 ; i < e2remove.size() ; ++i )
  {
    optimizer.addVertex(sw_vs[i]);
    optimizer.addEdge(e2add_loops[i]);
    optimizer.addEdge(e2add_priors[i]);
    optimizer.removeEdge(e2remove[i]);
  }

  // Optimize the problem
  std::cout << "Starting optimization : " << std::endl;
  optimizer.vertex(0)->setFixed(true);
  optimizer.initializeOptimization();
  chrono::steady_clock::time_point begin = chrono::steady_clock::now();
  optimizer.optimize(maxIterations);
  chrono::steady_clock::time_point end = chrono::steady_clock::now();
  chrono::microseconds delta_time = chrono::duration_cast<chrono::microseconds>(end - begin);

  // Updating values 
  int tp  = 0; int tn = 0;
  int fp  = 0; int fn = 0;
  for (int idx = 0; idx < inliers; ++idx)
      sw_vs[idx]->estimate() > inlier_th ? ++tp : ++fn;

  for ( int idx = inliers; idx < sw_vs.size(); ++idx)
      sw_vs[idx]->estimate() > inlier_th ? ++fp : ++tn;  

  float precision = tp + fp > 0 ? tp / (float)(tp + fp) : 0.0;
  float recall    = tp + fn > 0 ? tp / (float)(tp + fn) : 0.0; 
  float dt = delta_time.count() / 1000000.0;

  std::cout << "Optimization complete in " << dt << " [s]" << std::endl;
  std::cout << "Precision  = " << precision << std::endl;
  std::cout << "Recall = " << recall << std::endl;
  std::cout << "TP = " << tp << std::endl;
  std::cout << "TN = " << tn << std::endl;
  std::cout << "FP = " << fp << std::endl;
  std::cout << "FN = " << fn << std::endl;
  
  std::cout << "Optimtization Concluded!" << std::endl;

  ofstream outfile;
  outfile.open(cfg.output.c_str()); 
  for (size_t it = 0; it < optimizer.vertices().size(); ++it )
  {
      VertexSE3* v = dynamic_cast<VertexSE3*>(optimizer.vertex(it));

      if ( v == nullptr ) continue;

      writeVertex(outfile, v);
  }
  outfile.close();

  string out = cfg.output.substr(0, cfg.output.size() - 3) + "PR";
  outfile.open(out.c_str());
  outfile << precision << " " << recall << endl;
  outfile << dt << endl;
  outfile.close();

  return 0;
}

EdgeSE3Switchable* getSwitchableEdge(EdgeSE3* candidate, VertexSwitchLinear* sw)
{
    EdgeSE3Switchable* edge = new EdgeSE3Switchable();
    edge->setVertex(0, candidate->vertex(0));
    edge->setVertex(1, candidate->vertex(1));
    edge->setVertex(2, sw);
    Eigen::Isometry3d meas = candidate->measurement(); 
    edge->setMeasurement(SE3Quat(meas.rotation(), meas.translation()));
    edge->setInformation(candidate->information());

    return edge;
}