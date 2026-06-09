#include <gtsam/slam/dataset.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/DoglegOptimizer.h>
#include <gtsam/nonlinear/GncOptimizer.h>

#include <iostream>
#include <yaml-cpp/yaml.h>
#include <random>
#include <chrono>
#include <vector>
#include <algorithm>

#include <fstream>
#include <sstream>
#include <cstdlib>
#include <string>

#define LINESIZE 81920

struct Config
{
  std::string dataset;
  std::string output;
  int canonic_inliers;
  int maxiters;
  double inlier_th;
  double alpha;
  bool init_loop;
};


void readConfig(const std::string& cfg_filepath, Config& out_cfg);
void store3D(const std::string& output_filepath, const gtsam::Values& result);
void store2D(const std::string& output_filepath, const gtsam::Values& result);

void addPrior3D(gtsam::NonlinearFactorGraph& nfg);
void addPrior2D(gtsam::NonlinearFactorGraph& nfg);