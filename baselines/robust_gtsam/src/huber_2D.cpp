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
  bool init_loop = cfg.init_loop;
  
  typedef Pose2 PoseType;
  vector<NonlinearFactor::shared_ptr> loops;
  vector<size_t> idxsLoops;

  // reading file and creating factor graph
  NonlinearFactorGraph::shared_ptr graph;
  Values::shared_ptr initial;
  bool is3D = false;
  boost::tie(graph, initial) = readG2o(input_dataset, is3D, KernelFunctionTypeHUBER);
  Values new_init = *initial;

  for (size_t idx = 0; idx < graph->size(); ++idx)
  {
    auto factor = (*graph)[idx];
    bool isbinary = new_init.exists(factor->front());
    int delta = factor->front() - factor->back();

    // convert to between factor
    /**
    if (isbinary && (init_loop || std::abs(delta) == 1)) 
    {
      BetweenFactor<PoseType>& btwn =
          *boost::dynamic_pointer_cast<BetweenFactor<PoseType>>(factor);
      new_init.update(
          factor->back(),
          new_init.at<PoseType>(factor->front()).compose(btwn.measured()));
    }
    /**/

    if ( std::abs(delta) > 1 ) 
    {
      loops.push_back(factor);
      idxsLoops.push_back(idx);
    }    
  }

  // Add prior on the pose having index (key) = 0
  std::cout << "Adding prior on pose 0 " << std::endl;
  NonlinearFactorGraph nfg = *graph;
  addPrior2D(nfg);

  //LevenbergMarquardtParams lmParams;
  GaussNewtonParams lmParams;
  lmParams.setMaxIterations(maxIterations);
  lmParams.setRelativeErrorTol(-1e+20);
  lmParams.setAbsoluteErrorTol(-1e+20);
  //lmParams.setVerbosityLM("SUMMARY");
  lmParams.setVerbosity("ERROR");
  //LevenbergMarquardtOptimizer lm(nfg, new_init, lmParams);
  GaussNewtonOptimizer lm(nfg, new_init, lmParams);

  std::cout << "Optimizing the factor graph" << std::endl;
  chrono::steady_clock::time_point begin = chrono::steady_clock::now();
  Values result = lm.optimize();
  chrono::steady_clock::time_point end = chrono::steady_clock::now();
  chrono::microseconds delta_time = chrono::duration_cast<chrono::microseconds>(end - begin);
  std::cout << "-------------------" << std::endl;

  int dof = 3;
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

  for (size_t idx = 0; idx < idxsLoops.size(); ++idx)
  {
    int real_idx = idxsLoops[idx];
    auto factor = nfg[real_idx];
    Values tmp;
    tmp.insert(factor->back(), result.at<PoseType>(factor->back()));
    tmp.insert(factor->front(), result.at<PoseType>(factor->front()));
    double v = factor->error(tmp);
    if ( v > barcSq) nfg.remove(real_idx);
  }

  GaussNewtonParams lmParams_ref;
  //LevenbergMarquardtParams lmParams_ref;
  lmParams_ref.setMaxIterations(maxIterations);
  //lmParams_ref.setRelativeErrorTol(-1e+20);
  //lmParams_ref.setAbsoluteErrorTol(-1e+20);
  //lmParams_ref.setVerbosityLM("SUMMARY");
  lmParams_ref.setVerbosity("ERROR");
  GaussNewtonOptimizer lm_ref(nfg, new_init, lmParams_ref);
  //LevenbergMarquardtOptimizer lm_ref(nfg, new_init, lmParams_ref);
  Values result_lm = lm_ref.optimize();
  //Values result_lm = result;

  float precision = tp + fp > 0.0 ? tp / (float)(tp + fp) : 0.0;
  float recall    = tp + fn > 0.0 ? tp / (float)(tp + fn) : 0.0; 
  float dt = delta_time.count() / 1000000.0;

  std::cout << "Optimization complete in " << dt << " [s]" << std::endl;
  std::cout << "TP = " << tp << std::endl;
  std::cout << "TN = " << tn << std::endl;
  std::cout << "FP = " << fp << std::endl;
  std::cout << "FN = " << fn << std::endl;
  std::cout << "Precision  = " << precision << std::endl;
  std::cout << "Recall = " << recall << std::endl;
  std::cout << "initial error=" <<graph->error(new_init)<< std::endl;
  std::cout << "final error=" <<nfg.error(result)<< std::endl;
  
  store2D(output_file_trj, result);
  store2D("init.txt", new_init);
  store2D("refined.txt", result_lm);

  string output_file_pr = output_file_trj.substr(0, output_file_trj.size() - 3) + "PR";
  ofstream outfile;
  outfile.open(output_file_pr.c_str());
  outfile << precision << " " << recall << endl;
  outfile << dt << endl;
  outfile.close();


  return 0;
}
