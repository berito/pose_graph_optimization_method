cmake_minimum_required(VERSION 3.5.2)
project(TACO)

set(CMAKE_CXX_FLAGS "-std=c++14 -O3")
set(CMAKE_BUILD_TYPE Release)

# Reuse IPC's cmake modules (FindG2O / FindGlog) and headers/sources — TACO is built on IPC.
set(IPC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../ipc)
LIST(APPEND CMAKE_MODULE_PATH ${IPC_DIR}/cmake)

find_package(Boost REQUIRED COMPONENTS system)
find_package(Glog REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(G2O REQUIRED)
find_package(yaml-cpp REQUIRED)

find_path(CSPARSE_INCLUDE_DIR cs.h PATHS /usr/include/suitesparse /usr/local/include/suitesparse /usr/include)

set(SC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../baselines/robust_g2o/thirdParty/switchableConstraints)

include_directories(
  include
  ${IPC_DIR}/include
  ${EIGEN_INCLUDE_DIR}
  "/usr/include/eigen3"
  ${G2O_INCLUDE_DIR}
  ${CSPARSE_INCLUDE_DIR}
  ${YAML_CPP_INCLUDE_DIR}
  ${GLOG_INCLUDE_DIR}/glog
  ${SC_DIR}/include
  ${SC_DIR}/../               # so #include "switchableConstraints/include/..." resolves
)

# Reused-from-IPC helpers (g2o I/O, store/restore, chi2 test, propagate, robustify) — NOT reinvented.
set(IPC_REUSED_SOURCES
  ${IPC_DIR}/src/utils.cpp
  ${IPC_DIR}/src/consensus_utils.cpp
)

# Vertigo switchable-constraint types (vendored in baselines).
set(SC_SOURCES
  ${SC_DIR}/src/vertex_switchLinear.cpp
  ${SC_DIR}/src/edge_se2Switchable.cpp
  ${SC_DIR}/src/edge_se3Switchable.cpp
  ${SC_DIR}/src/edge_switchPrior.cpp
  ${SC_DIR}/src/jacobian.cpp
)

# TACO-only additions: kBL subgraph selection + SR-SC + the TACO driver.
set(TACO_SOURCES
  src/kbl_consensus.cpp
  src/srsc.cpp
  src/taco_simulation.cpp
)

set(TACO_LIBS
  ${G2O_TYPES_SLAM2D}
  ${G2O_TYPES_SLAM3D}
  ${G2O_TYPES_DATA}
  ${G2O_CORE_LIBRARY}
  ${G2O_STUFF_LIBRARY}
  ${G2O_SOLVER_EIGEN}
  ${YAML_CPP_LIBRARIES}
  ${GLOG_LIBRARY}
)

add_executable(taco_tester_2D examples/taco_tester_2D.cpp ${TACO_SOURCES} ${IPC_REUSED_SOURCES} ${SC_SOURCES})
target_link_libraries(taco_tester_2D ${TACO_LIBS})

add_executable(taco_tester_3D examples/taco_tester_3D.cpp ${TACO_SOURCES} ${IPC_REUSED_SOURCES} ${SC_SOURCES})
target_link_libraries(taco_tester_3D ${TACO_LIBS})
