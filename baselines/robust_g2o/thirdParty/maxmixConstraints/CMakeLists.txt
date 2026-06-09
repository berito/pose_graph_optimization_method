cmake_minimum_required(VERSION 3.5...4.5)
project(maxmixlib)
set(CMAKE_BUILD_TYPE Release)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${G2O_INCLUDE_DIR}
)

add_library(maxmixlib SHARED 
src/edge_se2_mixture.cpp
src/edge_se3_mixture.cpp)

target_link_libraries(maxmixlib
${G2O_TYPES_SLAM2D}
${G2O_TYPES_SLAM3D}
${G2O_TYPES_DATA}
${G2O_CORE_LIBRARY}
${G2O_STUFF_LIBRARY}
${G2O_SOLVER_EIGEN}
)
