# Cmake test file 
cmake_minimum_required(VERSION 3.10...4.5)
project(gncg2olib)
SET(CMAKE_BUILD_TYPE Release)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${G2O_INCLUDE_DIR}
)

add_library(gncg2olib SHARED
src/gnc_kernel_impl.cpp
src/gnc_sparse_optimizer.cpp)

target_link_libraries(gncg2olib
${G2O_TYPES_SLAM2D}
${G2O_TYPES_SLAM3D}
${G2O_TYPES_DATA}
${G2O_CORE_LIBRARY}
${G2O_STUFF_LIBRARY}
${G2O_SOLVER_EIGEN}
)
