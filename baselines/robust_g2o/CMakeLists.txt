find_package(glog QUIET)
find_package(yaml-cpp REQUIRED)

add_subdirectory(thirdParty)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${G2O_INCLUDE_DIR}
  ${YAML_CPP_INCLUDE_DIR}
  thirdParty
)

add_library(g2o_common 
  src/utils.cpp
  src/dcs_kernel_impl.cpp
)

target_include_directories(g2o_common PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_link_libraries(g2o_common PUBLIC
  ${G2O_TYPES_SLAM2D}
  ${G2O_TYPES_SLAM3D}
  ${G2O_TYPES_DATA}
  ${G2O_CORE_LIBRARY}
  ${G2O_STUFF_LIBRARY}
  ${G2O_SOLVER_EIGEN}
  yaml-cpp::yaml-cpp
  ${GLOG_LIBRARY}
)

add_subdirectory(src/incr)
add_subdirectory(src/off)


add_executable(g2o_dataset_enhancer src/dataset_enhancer.cpp)
target_link_libraries(g2o_dataset_enhancer PRIVATE g2o_common)

add_executable(g2o_dataset_seeder src/artificial_seeder.cpp)
target_link_libraries(g2o_dataset_seeder PRIVATE g2o_common)


add_executable(g2o_noise_applier_2D src/noise_applier_2D.cpp)
target_link_libraries(g2o_noise_applier_2D PRIVATE g2o_common)

add_executable(g2o_covariance_estimation src/covariance_estimation.cpp)
target_link_libraries(g2o_covariance_estimation PRIVATE g2o_common)