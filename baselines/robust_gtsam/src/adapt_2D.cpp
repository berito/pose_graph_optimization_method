// ADAPT (Barron adaptive robust loss) baseline — IPC paper comparator [5].
// The author "implemented [it] inside g2o" but did NOT release the code, and it is not in
// any standard library (it needs IRLS, not a fixed kernel). REIMPLEMENTED here following
// Barron CVPR'19 + Chebrolu RA-L'21 (2004.14938) + AEROS Frontiers'22 (2110.02018):
//   inner: GTSAM robust noiseModel does IRLS reweighting for a fixed shape alpha;
//   outer: re-estimate scale c (MAD) and shape alpha (NLL grid-MLE) from residuals, repeat.
// Metrics + I/O are identical to the gm/dcs testers (emits .PR = precision recall \n time).
// NOTE (S3 verify): the alpha grid + truncated partition function are pragmatic choices —
// verify ADAPT against the paper's reported numbers at G1 before trusting it as a comparator.
#include "utils.hpp"
#include "barron.hpp"

using namespace std;
using namespace gtsam;

int main(int argc, char** argv)
{
  if (argc < 2) { cout << "Usage: " << argv[0] << " <cfg_file>" << endl; return 1; }

  Config cfg;
  readConfig(argv[1], cfg);
  string input_dataset = cfg.dataset;
  int maxIterations = cfg.maxiters;
  int inliers = cfg.canonic_inliers;
  double alpha_conf = cfg.alpha;          // chi^2 confidence used for the inlier test (eval only)
  string output_file_trj = cfg.output;

  typedef Pose2 PoseType;

  NonlinearFactorGraph::shared_ptr graph;
  Values::shared_ptr initial;
  bool is3D = false;
  boost::tie(graph, initial) = readG2o(input_dataset, is3D);
  Values new_init = *initial;

  // Mark loop-closure factors (|delta| > 1).
  vector<char> isLoop(graph->size(), 0);
  vector<size_t> loopIdx;
  for (size_t id = 0; id < graph->size(); ++id) {
    auto factor = (*graph)[id];
    bool isbinary = new_init.exists(factor->front());
    int delta = isbinary ? (int)(factor->front() - factor->back()) : 0;
    if (std::abs(delta) > 1) { isLoop[id] = 1; loopIdx.push_back(id); }
  }

  int dof = 3;
  double alpha = 1.0, c = 1.0;     // adaptive state (alpha=1 -> Charbonnier-ish start)
  const int OUTER = 10;
  Values result = new_init;

  chrono::steady_clock::time_point begin = chrono::steady_clock::now();
  for (int it = 0; it < OUTER; ++it) {
    // Rebuild the graph with Barron(alpha,c) robust noise on the loop-closure edges.
    NonlinearFactorGraph nfg;
    auto kernel = barron::Barron::Create(alpha, c);
    for (size_t id = 0; id < graph->size(); ++id) {
      if (isLoop[id]) {
        BetweenFactor<PoseType>& loop =
            *boost::dynamic_pointer_cast<BetweenFactor<PoseType>>((*graph)[id]);
        auto rn = noiseModel::Robust::Create(kernel, loop.noiseModel());
        nfg.add(BetweenFactor<PoseType>(loop.front(), loop.back(), loop.measured(), rn));
      } else {
        nfg.push_back((*graph)[id]);
      }
    }
    addPrior2D(nfg);

    LevenbergMarquardtParams lmParams;
    lmParams.setMaxIterations(maxIterations);
    LevenbergMarquardtOptimizer lm(nfg, new_init, lmParams);
    result = lm.optimize();

    // Whitened residual magnitudes of the loop edges under the BASE noise.
    vector<double> res;
    res.reserve(loopIdx.size());
    for (size_t id : loopIdx) {
      BetweenFactor<PoseType>& loop =
          *boost::dynamic_pointer_cast<BetweenFactor<PoseType>>((*graph)[id]);
      Values tmp;
      tmp.insert(loop.front(), result.at<PoseType>(loop.front()));
      tmp.insert(loop.back(),  result.at<PoseType>(loop.back()));
      res.push_back(std::sqrt(2.0 * loop.error(tmp)));   // error = 0.5*||whitened||^2
    }

    double newC = barron::madScale(res);
    double newAlpha = barron::adaptAlpha(res, newC);
    new_init = result;   // warm-start next outer iteration
    bool converged = std::fabs(newAlpha - alpha) < 1e-9 && std::fabs(newC - c) < 1e-6;
    alpha = newAlpha; c = newC;
    if (converged) break;
  }
  chrono::steady_clock::time_point end = chrono::steady_clock::now();
  double dt = chrono::duration_cast<chrono::microseconds>(end - begin).count() / 1000000.0;

  // Precision/Recall exactly as in the gm/dcs testers (inlier test at alpha_conf).
  vector<NonlinearFactor::shared_ptr> loops;
  for (size_t id : loopIdx) loops.push_back((*graph)[id]);
  double barcSq = 0.5 * Chi2inv(alpha_conf, dof);
  int tp = 0, tn = 0, fp = 0, fn = 0;
  for (int idx = 0; idx < inliers && idx < (int)loops.size(); ++idx) {
    const BetweenFactor<PoseType>& b = *boost::dynamic_pointer_cast<BetweenFactor<PoseType>>(loops[idx]);
    Values tmp;
    tmp.insert(loops[idx]->back(),  result.at<PoseType>(loops[idx]->back()));
    tmp.insert(loops[idx]->front(), result.at<PoseType>(loops[idx]->front()));
    double v = b.error(tmp);
    if (v < barcSq) ++tp; else ++fn;
  }
  for (int idx = inliers; idx < (int)loops.size(); ++idx) {
    const BetweenFactor<PoseType>& b = *boost::dynamic_pointer_cast<BetweenFactor<PoseType>>(loops[idx]);
    Values tmp;
    tmp.insert(loops[idx]->back(),  result.at<PoseType>(loops[idx]->back()));
    tmp.insert(loops[idx]->front(), result.at<PoseType>(loops[idx]->front()));
    double v = b.error(tmp);
    if (v < barcSq) ++fp; else ++tn;
  }

  float precision = tp + fp > 0.0 ? tp / (float)(tp + fp) : 0.0;
  float recall    = tp + fn > 0.0 ? tp / (float)(tp + fn) : 0.0;

  std::cout << "ADAPT converged: alpha=" << alpha << " c=" << c << std::endl;
  std::cout << "Optimization complete in " << dt << " [s]" << std::endl;
  std::cout << "Precision  = " << precision << std::endl;
  std::cout << "Recall = " << recall << std::endl;
  std::cout << "TP = " << tp << " TN = " << tn << " FP = " << fp << " FN = " << fn << std::endl;

  store2D(output_file_trj, result);

  string output_file_pr = output_file_trj.substr(0, output_file_trj.size() - 3) + "PR";
  ofstream outfile;
  outfile.open(output_file_pr.c_str());
  outfile << precision << " " << recall << endl;
  outfile << dt << endl;
  outfile.close();

  return 0;
}
