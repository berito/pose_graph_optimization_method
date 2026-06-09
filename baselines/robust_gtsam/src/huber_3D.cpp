#include "utils.hpp"

using namespace std;
using namespace gtsam;

int main(int argc, char **argv) 
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

  typedef Pose3 PoseType;
  vector<PoseType> poses;
  vector<NonlinearFactor::shared_ptr> loops;

  NonlinearFactorGraph::shared_ptr graph;
  Values::shared_ptr initial;
  bool is3D = true;
  boost::tie(graph, initial) = readG2o(input_dataset, is3D, KernelFunctionTypeHUBER);
  Values new_init = *initial;

  int edge_counter = 0;
  for (const auto& factor : *graph) 
  {
    // convert to between factor
    if (new_init.exists(factor->front())) 
    {
      BetweenFactor<PoseType>& btwn =
          *boost::dynamic_pointer_cast<BetweenFactor<PoseType>>(factor);
      new_init.update(
          factor->back(),
          new_init.at<PoseType>(factor->front()).compose(btwn.measured()));
    }

    int delta = factor->front() - factor->back();
    if ( std::abs(delta) > 1 ) loops.push_back(factor);    
  }
  
  // Add prior on the pose having index (key) = 0
  std::cout << "Adding prior on pose 0 " << std::endl;
  NonlinearFactorGraph nfg = *graph;
  addPrior3D(nfg);
  
  LevenbergMarquardtParams lmParams;
  lmParams.setMaxIterations(maxIterations);
  LevenbergMarquardtOptimizer lm(nfg, new_init, lmParams);

  std::cout << "Optimizing the factor graph" << std::endl;
  chrono::steady_clock::time_point begin = chrono::steady_clock::now();
  Values result = lm.optimize();
  chrono::steady_clock::time_point end = chrono::steady_clock::now();
  chrono::microseconds delta_time = chrono::duration_cast<chrono::microseconds>(end - begin);

  int dof = 6;
  double barcSq = 0.5 * Chi2inv(alpha, dof);
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

  float precision = tp / (float)(tp + fp);
  float recall    = tp / (float)(tp + fn); 
  float dt = delta_time.count() / 1000000.0;

  std::cout << "Optimization complete in " << dt << " [s]" << std::endl;
  std::cout << "Precision  = " << precision << std::endl;
  std::cout << "Recall = " << recall << std::endl;
  std::cout << "initial error=" <<graph->error(*initial)<< std::endl;
  std::cout << "final error=" <<graph->error(result)<< std::endl;
  

  store3D(output_file_trj, result);

  string output_file_pr = output_file_trj.substr(0, output_file_trj.size() - 3) + "PR";
  ofstream outfile;
  outfile.open(output_file_pr.c_str());
  outfile << precision << " " << recall << endl;
  outfile << dt << endl;
  outfile.close();


  return 0;
}
