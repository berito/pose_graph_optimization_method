# Cmake test file 
cmake_minimum_required(VERSION 3.5...4.5)
project(rrrlib)
SET(CMAKE_BUILD_TYPE Release)

add_library(rrrlib INTERFACE)

target_include_directories(rrrlib INTERFACE
 ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(rrrlib INTERFACE
${G2O_TYPES_SLAM2D}
${G2O_TYPES_SLAM3D}
${G2O_TYPES_DATA}
${G2O_CORE_LIBRARY}
${G2O_STUFF_LIBRARY}
${G2O_SOLVER_EIGEN}
)
