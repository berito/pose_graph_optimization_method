#include "dc_ipc/dc_simulation.hpp"
#include "ipc/utils.hpp"

using namespace std;
using namespace g2o;

G2O_USE_TYPE_GROUP(slam2d);
G2O_USE_OPTIMIZATION_LIBRARY(eigen);

int main(int argc, char** argv)
{
  string cfgFilename;
  CommandArgs arg;
  arg.param("c", cfgFilename, "", "path to cfg file");
  arg.parseArgs(argc, argv);

  Config cfg;
  readConfig(cfgFilename, cfg);

  // Parse lambda independently; ipc/utils.hpp Config is kept pristine (brief §4 / λ plumbing).
  double lambda = 0.0;
  try { lambda = YAML::LoadFile(cfgFilename)["lambda"].as<double>(); } catch (...) {}

  vector<SE2> init_poses;
  vector<VertexSE2*> v_poses;

  SparseOptimizer optimizer;
  setProblem<SE2, EdgeSE2, VertexSE2>(cfg.dataset, optimizer, init_poses, v_poses);

  vector<EdgeSE2*> loops, odom_edges;
  splitProblemConstraints<EdgeSE2>(optimizer, odom_edges, loops);

  simulating_incremental_data_dc<Eigen::Isometry2d, EdgeSE2, VertexSE2>(cfg, optimizer, loops, lambda);
  return 0;
}
