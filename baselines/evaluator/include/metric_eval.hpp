#pragma once

#include <iostream>
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Dense>

#include "g2o/core/factory.h"
#include "g2o/core/optimization_algorithm_factory.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/stuff/command_args.h"
#include "g2o/types/slam2d/types_slam2d.h"
#include "g2o/types/slam3d/types_slam3d.h"

void evaluateAte(std::vector<Eigen::Isometry3d>& sol,
                 const std::vector<Eigen::Isometry3d>& gt,
                 double& ate_rotation,
                 double& ate_translation,
                 double& rmse_translation);

void evaluateRpe(const std::vector<Eigen::Isometry3d>& sol,
                 const std::vector<Eigen::Isometry3d>& gt,
                 double& rpe_rotation,
                 double& rpe_translation);
                
void computeAlignmentTransform(const std::vector<Eigen::Isometry3d>& sol,
                               const std::vector<Eigen::Isometry3d>& gt,
                               Eigen::Isometry3d& delta);

void align(std::vector<Eigen::Isometry3d>& sol, const Eigen::Isometry3d& T);

void readSolutionFile2D(std::vector<Eigen::Isometry3d>& poses, const std::string& path);
void readSolutionFile3D(std::vector<Eigen::Isometry3d>& poses, const std::string& path);

void evaluateSectionRpe(std::vector<Eigen::Isometry3d>& sol,
                        const std::vector<Eigen::Isometry3d>& gt,
                        const g2o::SparseOptimizer& graph,
                        double& rmse_translation,
                        double& energy,
                        double& valid_trajectory);


void evaluateFPSRpeVanilla(std::vector<Eigen::Isometry3d>& sol,
                           const std::vector<Eigen::Isometry3d>& gt,
                           const int samples,
                           double& rmse_translation);


void evaluateFPSRpe(std::vector<Eigen::Isometry3d>& sol,
                    const std::vector<Eigen::Isometry3d>& gt,
                    const double percentage,
                    double& rmse_translation);

void evaluateEnergyDeformation(std::vector<Eigen::Isometry3d>& sol,
                               const std::vector<Eigen::Isometry3d>& gt,
                               const g2o::SparseOptimizer& graph,
                               double& energy);