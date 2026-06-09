cmake_minimum_required(VERSION 3.5...4.5)
project(robust_optimization)

# -----------------------------------------------------------------------------
# Vendored from https://github.com/EmilioOlivastri/RobustOptimizationSLAM
# (commit 62b1c9ccab71ac30fad98f5c5df28087eca0c1ae). See VENDOR.md.
#
# Local thesis change vs upstream: the GTSAM/Kimera tier (robust_gtsam) is now
# guarded behind BUILD_GTSAM_BASELINES so the g2o baselines + evaluator build
# with nothing more than the IPC fork already needs (g2o + glog + yaml-cpp).
# When built from the parent superbuild the option is inherited; standalone it
# defaults OFF here too.
# -----------------------------------------------------------------------------

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS "-std=c++17")

option(BUILD_GTSAM_BASELINES "Build the GTSAM/Kimera PCM baseline tier" OFF)

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(Boost REQUIRED COMPONENTS system filesystem regex timer date_time program_options)
find_package(Eigen3 REQUIRED)
get_target_property(EIGEN3_INCLUDE_DIRS Eigen3::Eigen INTERFACE_INCLUDE_DIRECTORIES)
find_package(yaml-cpp REQUIRED)
find_package(G2O REQUIRED)

# g2o baselines (SC / MaxMix / DCS / GNC / Huber / RRR) + metric evaluator.
# These need only g2o + yaml-cpp + glog — same toolchain as the IPC fork.
add_subdirectory(robust_g2o)
add_subdirectory(evaluator)

# GTSAM tier: gtsam_{HUBER,DCS,GNC}_* and gtsam_PCM_* (PCM via Kimera-RPGO).
# Heavy deps (GTSAM + KimeraRPGO + TBB), so opt-in.
if(BUILD_GTSAM_BASELINES)
  set(CMAKE_MESSAGE_LOG_LEVEL WARNING)
  find_package(GTSAM REQUIRED)
  set(CMAKE_MESSAGE_LOG_LEVEL STATUS)
  add_subdirectory(robust_gtsam)
else()
  message(STATUS "robust_optimization: GTSAM tier skipped (set -DBUILD_GTSAM_BASELINES=ON to build gtsam_*/PCM)")
endif()
