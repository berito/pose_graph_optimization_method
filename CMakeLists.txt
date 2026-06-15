cmake_minimum_required(VERSION 3.5)
project(robust_pgo LANGUAGES CXX)

# =============================================================================
# robust_pgo — superbuild
#
#   ipc/        thesis fork of IPC (our contribution; the decision we patch)
#   taco/       TACO comparator (kBL-IPC + SR-SC), built on top of ipc/ (reuses its sources)
#   dc_ipc/     DC-IPC — our correlation-aware contribution, built on top of ipc/ + taco/
#   baselines/  vendored RobustOptimizationSLAM (Olivastri) — the comparators
#
# Each subdirectory is a self-contained CMake project (own project(), own
# find_package, own cmake/ module path), so any of them also builds standalone.
# This top-level file just stitches them into one build tree:
#
#   mkdir build && cd build
#   cmake .. -DCMAKE_BUILD_TYPE=Release          # IPC + g2o baselines + evaluator
#   cmake .. -DBUILD_GTSAM_BASELINES=ON           # also build the GTSAM/PCM tier
#   make -j$(nproc)
#
# All executables land in this one build/ directory.
# =============================================================================

option(BUILD_IPC         "Build the thesis IPC fork (ipc/)"                 ON)
option(BUILD_TACO        "Build the TACO comparator (kBL-IPC + SR-SC)"      ON)
option(BUILD_DC_IPC      "Build DC-IPC (our correlation-aware contribution)" ON)
option(BUILD_BASELINES   "Build the vendored robust-PGO baselines"          ON)
# The GTSAM tier (gtsam_* + PCM via Kimera-RPGO) needs GTSAM + KimeraRPGO + TBB.
# Default OFF so the build succeeds even before those heavy deps are installed;
# the devcontainer installs them and flips this ON.  The g2o baselines
# (SC / MaxMix / DCS / GNC / Huber / RRR) do NOT need it.
option(BUILD_GTSAM_BASELINES "Build the GTSAM/Kimera PCM baseline tier"     OFF)

if(BUILD_IPC)
  add_subdirectory(ipc)
endif()

if(BUILD_TACO)
  add_subdirectory(taco)
endif()

if(BUILD_DC_IPC)
  add_subdirectory(dc_ipc)
endif()

if(BUILD_BASELINES)
  add_subdirectory(baselines)
endif()

message(STATUS "robust_pgo configuration:")
message(STATUS "  BUILD_IPC             = ${BUILD_IPC}")
message(STATUS "  BUILD_TACO            = ${BUILD_TACO}")
message(STATUS "  BUILD_DC_IPC          = ${BUILD_DC_IPC}")
message(STATUS "  BUILD_BASELINES       = ${BUILD_BASELINES}")
message(STATUS "  BUILD_GTSAM_BASELINES = ${BUILD_GTSAM_BASELINES}")
