cmake_minimum_required(VERSION 3.5.2)
project(DC_IPC)

set(CMAKE_CXX_FLAGS "-std=c++14 -O3")
set(CMAKE_BUILD_TYPE Release)

# DC-IPC is built on top of ipc/ and taco/ — reuse their cmake modules, headers, and sources.
set(IPC_DIR  ${CMAKE_CURRENT_SOURCE_DIR}/../ipc)
set(TACO_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../taco)
set(SC_DIR   ${CMAKE_CURRENT_SOURCE_DIR}/../baselines/robust_g2o/thirdParty/switchableConstraints)
LIST(APPEND CMAKE_MODULE_PATH ${IPC_DIR}/cmake)

find_package(Boost REQUIRED COMPONENTS system)
find_package(Glog REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(G2O REQUIRED)
find_package(yaml-cpp REQUIRED)

find_path(CSPARSE_INCLUDE_DIR cs.h PATHS /usr/include/suitesparse /usr/local/include/suitesparse /usr/include)

include_directories(
  include
  ${IPC_DIR}/include
  ${TACO_DIR}/include
  ${EIGEN_INCLUDE_DIR}
  "/usr/include/eigen3"
  ${G2O_INCLUDE_DIR}
  ${CSPARSE_INCLUDE_DIR}
  ${YAML_CPP_INCLUDE_DIR}
  ${GLOG_INCLUDE_DIR}/glog
  ${SC_DIR}/include
  ${SC_DIR}/../
)

# Reused from ipc/ (helpers) and taco/ (kBL-IPC + SR-SC) and the vertigo switchable types — NOT reinvented.
set(REUSED_SOURCES
  ${IPC_DIR}/src/utils.cpp
  ${IPC_DIR}/src/consensus_utils.cpp
  ${TACO_DIR}/src/kbl_consensus.cpp
  ${TACO_DIR}/src/srsc.cpp
  ${SC_DIR}/src/vertex_switchLinear.cpp
  ${SC_DIR}/src/edge_se2Switchable.cpp
  ${SC_DIR}/src/edge_se3Switchable.cpp
  ${SC_DIR}/src/edge_switchPrior.cpp
  ${SC_DIR}/src/jacobian.cpp
)

# DC-IPC-only additions: the correlation-aware (group-joint) decision + driver.
set(DC_SOURCES
  src/dc_consensus.cpp
  src/dc_simulation.cpp
)

set(DC_LIBS
  ${G2O_TYPES_SLAM2D}
  ${G2O_TYPES_SLAM3D}
  ${G2O_TYPES_DATA}
  ${G2O_CORE_LIBRARY}
  ${G2O_STUFF_LIBRARY}
  ${G2O_SOLVER_EIGEN}
  ${YAML_CPP_LIBRARIES}
  ${GLOG_LIBRARY}
)

add_executable(dc_ipc_tester_2D examples/dc_ipc_tester_2D.cpp ${DC_SOURCES} ${REUSED_SOURCES})
target_link_libraries(dc_ipc_tester_2D ${DC_LIBS})

add_executable(dc_ipc_tester_3D examples/dc_ipc_tester_3D.cpp ${DC_SOURCES} ${REUSED_SOURCES})
target_link_libraries(dc_ipc_tester_3D ${DC_LIBS})
