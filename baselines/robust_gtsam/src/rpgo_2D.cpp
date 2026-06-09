#include "utils.hpp"

#include <stdlib.h>
#include <memory>
#include <string>

#include "KimeraRPGO/RobustSolver.h"
#include "KimeraRPGO/SolverParams.h"
#include "KimeraRPGO/Logger.h"
#include "KimeraRPGO/utils/GeometryUtils.h"
#include "KimeraRPGO/utils/TypeUtils.h"

using namespace KimeraRPGO;
using namespace gtsam;
using namespace std;

int main(int argc, char* argv[]) 
{
  // Command line parsing
  string cfg_file;

  if (argc < 2) 
  {
    cout << "Usage: " << argv[0] << " <cfg_file>" << endl;
    return 1;
  }
  cfg_file = argv[1];

  Config cfg;
  readConfig(cfg_file, cfg);
  string input_dataset = cfg.dataset;
  int maxIterations = cfg.maxiters;
  double inlier_th = cfg.inlier_th;
  int inliers = cfg.canonic_inliers;
  double alpha = cfg.alpha; 
  string output_file_trj = cfg.output;
  bool init_loop = cfg.init_loop;

  typedef Pose2 PoseType;
  vector<NonlinearFactor::shared_ptr> loops;

  RobustSolverParams params;
  NonlinearFactorGraph::shared_ptr graph;
  Values::shared_ptr initial;
  bool is3D = false;
  boost::tie(graph, initial) = readG2o(input_dataset, is3D);
  Values new_init = *initial;

  int dof = 3;
  double th = 0.5 *Chi2inv(alpha, dof);
  params.setPcm2DParams(-th, th, Verbosity::QUIET);
  params.setLmDiagonalDamping(is3D);
  //params.setGncInlierCostThresholdsAtProbability(alpha);
  unique_ptr<RobustSolver> pgo = KimeraRPGO::make_unique<RobustSolver>(params);  
  for (const auto& factor : *graph) 
  {
    bool isbinary = new_init.exists(factor->front());
    int delta = isbinary ? factor->front() - factor->back() : 0;

    // convert to between factor
    if (isbinary && (init_loop || abs(delta) == 1)) 
    {
      BetweenFactor<PoseType>& btwn =
          *boost::dynamic_pointer_cast<BetweenFactor<PoseType>>(factor);
      new_init.update(
          factor->back(),
          new_init.at<PoseType>(factor->front()).compose(btwn.measured()));
    }

    if ( abs(delta) > 1 ) loops.push_back(factor);    
  }

  std::cout << "Adding prior on pose 0 " << std::endl;
  NonlinearFactorGraph nfg = *graph;
  addPrior2D(nfg);
  
  cout << "Starting optimization" << endl;
  chrono::steady_clock::time_point begin = chrono::steady_clock::now();
  pgo->update(nfg, new_init);
  chrono::steady_clock::time_point end = chrono::steady_clock::now();
  Values result = pgo->calculateBestEstimate();
  double dt = chrono::duration_cast<chrono::microseconds>(end - begin).count() / 1000000.0;

  double barcSq = 0.5 * Chi2inv(inlier_th, dof);
  int tp  = 0; int tn = 0;
  int fp  = 0; int fn = 0;
  for ( int idx = 0; idx < inliers; ++idx)
  {
    const BetweenFactor<PoseType>& btwn = *boost::dynamic_pointer_cast<BetweenFactor<PoseType>>(loops[idx]);
    Values tmp;
    tmp.insert(loops[idx]->back(), result.at<PoseType>(loops[idx]->back()));
    tmp.insert(loops[idx]->front(), result.at<PoseType>(loops[idx]->front()));
    double v = btwn.error(tmp);
    if ( v < barcSq ) ++tp;
    else ++fn;
  }

  for ( int idx = inliers; idx < loops.size(); ++idx)
  {
    const BetweenFactor<PoseType>& btwn = *boost::dynamic_pointer_cast<BetweenFactor<PoseType>>(loops[idx]);
    Values tmp;
    tmp.insert(loops[idx]->back(), result.at<PoseType>(loops[idx]->back()));
    tmp.insert(loops[idx]->front(), result.at<PoseType>(loops[idx]->front()));
    double v = btwn.error(tmp);
    if ( v < barcSq ) ++fp;
    else ++tn;
  }
  float precision = tp + fp > 0.0 ? tp / (float)(tp + fp) : 0.0;
  float recall    = tp + fn > 0.0 ? tp / (float)(tp + fn) : 0.0; 

  cout << "Optimization complete in " << dt << " [s]" << endl;
  cout << "Precision  = " << precision << endl;
  cout << "Recall = " << recall << endl;
  cout << "TP = " << tp << endl;
  cout << "TN = " << tn << endl;
  cout << "FP = " << fp << endl;
  cout << "FN = " << fn << endl;
  cout << "initial error=" << nfg.error(*initial)<< endl;
  cout << "final error=" << nfg.error(result)<< endl;


  store2D(output_file_trj, result);

  string output_file_pr = output_file_trj.substr(0, output_file_trj.size() - 3) + "PR";
  ofstream outfile;
  outfile.open(output_file_pr.c_str());
  outfile << precision << " " << recall << endl;
  outfile << dt << endl;
  outfile.close();

  return 0;
}