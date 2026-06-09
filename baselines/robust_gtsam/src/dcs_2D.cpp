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
  double alpha = cfg.alpha; // alpha = 0.99
  string output_file_trj = cfg.output;
  bool init_loop = cfg.init_loop;
  
  typedef Pose2 PoseType;
  vector<NonlinearFactor::shared_ptr> loops;

  // reading file and creating factor graph
  NonlinearFactorGraph::shared_ptr graph;
  Values::shared_ptr initial;
  bool is3D = false;
  boost::tie(graph, initial) = readG2o(input_dataset, is3D);
  Values new_init = *initial;

  int dof = 3;
  int id = 0;
  double th = Chi2inv(alpha, dof);
  for (auto& factor : *graph) 
  {
    bool isbinary = new_init.exists(factor->front());
    int delta = isbinary ? factor->front() - factor->back() : 0;
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
        BetweenFactor<PoseType>& loop =
          *boost::dynamic_pointer_cast<BetweenFactor<PoseType>>(factor);
        auto noise = loop.noiseModel();
        auto robust_noise = noiseModel::Robust::Create(noiseModel::mEstimator::DCS::Create(th), noise);

        NonlinearFactor::shared_ptr new_factor(new BetweenFactor<PoseType>(BetweenFactor<PoseType>(loop.front(), loop.back(), loop.measured(), robust_noise)));
        graph->replace(id, new_factor);
        loops.push_back(factor);
    }

    id++;
  }
    
  // Add prior on the pose having index (key) = 0
  std::cout << "Adding prior on pose 0 " << std::endl;
  NonlinearFactorGraph nfg = *graph;
  addPrior2D(nfg);

  GaussNewtonParams lmParams;
  lmParams.setMaxIterations(maxIterations);
  lmParams.setVerbosity("ERROR");
  //lmParams.setVerbosityLM("SUMMARY");
  GaussNewtonOptimizer lm(nfg, new_init, lmParams);
  //LevenbergMarquardtOptimizer lm(nfg, new_init, lmParams);

  std::cout << "Optimizing the factor graph" << std::endl;

  chrono::steady_clock::time_point begin = chrono::steady_clock::now();
  Values result = lm.optimize();
  chrono::steady_clock::time_point end = chrono::steady_clock::now();
  chrono::microseconds delta_time = chrono::duration_cast<chrono::microseconds>(end - begin);

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

  float precision = tp + fp > 0.0 ? tp / (float)(tp + fp) : 0.0;
  float recall    = tp + fn > 0.0 ? tp / (float)(tp + fn) : 0.0; 
  float dt = delta_time.count() / 1000000.0;

  std::cout << "Optimization complete in " << dt << " [s]" << std::endl;
  std::cout << "Precision  = " << precision << std::endl;
  std::cout << "Recall = " << recall << std::endl;
  std::cout << "TP = " << tp << std::endl;
  std::cout << "TN = " << tn << std::endl;
  std::cout << "FP = " << fp << std::endl;
  std::cout << "FN = " << fn << std::endl;
  std::cout << "initial error=" <<graph->error(*initial)<< std::endl;
  std::cout << "final error=" <<graph->error(result)<< std::endl;
  
  store2D(output_file_trj, result);
  store2D("init.txt", new_init);

  string output_file_pr = output_file_trj.substr(0, output_file_trj.size() - 3) + "PR";
  ofstream outfile;
  outfile.open(output_file_pr.c_str());
  outfile << precision << " " << recall << endl;
  outfile << dt << endl;
  outfile.close();

  return 0;
}
