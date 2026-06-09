cmake_minimum_required(VERSION 3.5.2)
project(IPC)

set(CMAKE_CXX_FLAGS "-std=c++14 -O3")
# g2o is found via find_package(G2O) on standard prefixes (/usr/local). To use a
# non-standard g2o install, pass -DG2O_ROOT=/path or export G2O_ROOT — no hard-coded path here.
set(CMAKE_BUILD_TYPE Release)
LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

find_package(Boost REQUIRED COMPONENTS system)
find_package(Glog REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(G2O REQUIRED)
find_package(yaml-cpp REQUIRED)

# g2o's csparse solver header (#include <cs.h>) is pulled in by src/utils.cpp.
# cs.h ships in SuiteSparse's include dir (e.g. /usr/include/suitesparse), which
# is not on the default compile line — locate and add it.
find_path(CSPARSE_INCLUDE_DIR cs.h PATHS /usr/include/suitesparse /usr/local/include/suitesparse /usr/include)

include_directories(
  include
  ${EIGEN_INCLUDE_DIR}
  "/usr/include/eigen3"
  ${G2O_INCLUDE_DIR}
  ${CSPARSE_INCLUDE_DIR}
  ${YAML_CPP_INCLUDE_DIR}
  ${GLOG_INCLUDE_DIR}/glog
)

add_executable(ipc_tester_2D
examples/ipc_tester_2D.cpp
src/utils.cpp
src/consensus_utils.cpp
src/consensus.cpp
src/simulation.cpp
)

target_link_libraries(ipc_tester_2D
  ${G2O_TYPES_SLAM2D}
  ${G2O_TYPES_SLAM3D}
  ${G2O_TYPES_DATA}
  ${G2O_CORE_LIBRARY}
  ${G2O_STUFF_LIBRARY}
  ${G2O_SOLVER_EIGEN}
  ${YAML_CPP_LIBRARIES}
  ${GLOG_LIBRARY}
 )


add_executable(ipc_tester_3D
examples/ipc_tester_3D.cpp
src/utils.cpp
src/consensus_utils.cpp
src/consensus.cpp
src/simulation.cpp
)

target_link_libraries(ipc_tester_3D
  ${G2O_TYPES_SLAM2D}
  ${G2O_TYPES_SLAM3D}
  ${G2O_TYPES_DATA}
  ${G2O_CORE_LIBRARY}
  ${G2O_STUFF_LIBRARY}
  ${G2O_SOLVER_EIGEN}
  ${YAML_CPP_LIBRARIES}
  ${GLOG_LIBRARY}
 )


add_executable(graph_fixer
examples/graph_fixer.cpp
src/utils.cpp
src/consensus_utils.cpp
src/consensus.cpp
)

target_link_libraries(graph_fixer
  ${G2O_TYPES_SLAM2D}
  ${G2O_TYPES_SLAM3D}
  ${G2O_TYPES_DATA}
  ${G2O_CORE_LIBRARY}
  ${G2O_STUFF_LIBRARY}
  ${G2O_SOLVER_EIGEN}
  ${YAML_CPP_LIBRARIES}
  ${GLOG_LIBRARY}
 )
