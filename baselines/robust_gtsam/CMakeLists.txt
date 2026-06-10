# Thesis change vs upstream: Intel MKL is optional. GTSAM links fine against
# Eigen+TBB, so we don't force the proprietary MKL dependency. ${MKL_LIBRARIES}
# stays defined (empty when MKL is absent) so the link lines below are unchanged.
find_package(MKL QUIET)
if(NOT MKL_FOUND)
  set(MKL_LIBRARIES "")
  message(STATUS "robust_gtsam: MKL not found — linking GTSAM without MKL (Eigen+TBB).")
endif()
# Thesis change vs upstream: use TBB's CONFIG package (oneTBB ships TBBConfig.cmake
# on Ubuntu 22.04) instead of the vendored FindTBB.cmake module, which reads the
# removed header tbb/tbb_stddef.h and errors on oneTBB. Link names (tbb/tbbmalloc)
# are unchanged and resolve against the system libs.
find_package(TBB CONFIG REQUIRED)
find_package(KimeraRPGO REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
  ${KimeraRPGO_INCLUDE_DIR}
  ${YAML_CPP_INCLUDE_DIR}
)

message(STATUS "GTSAM LIBS: ${GTSAM_LIBS}")

add_executable(gtsam_HUBER_2D src/huber_2D.cpp src/utils.cpp)
add_executable(gtsam_HUBER_3D src/huber_3D.cpp src/utils.cpp)
add_executable(gtsam_DCS_2D src/dcs_2D.cpp src/utils.cpp)
add_executable(gtsam_DCS_3D src/dcs_3D.cpp src/utils.cpp)
add_executable(gtsam_GNC_2D src/gnc_2D.cpp src/utils.cpp)
add_executable(gtsam_GNC_3D src/gnc_3D.cpp src/utils.cpp)
add_executable(gtsam_GM_2D src/gm_2D.cpp src/utils.cpp)
add_executable(gtsam_GM_3D src/gm_3D.cpp src/utils.cpp)
add_executable(gtsam_ADAPT_2D src/adapt_2D.cpp src/utils.cpp)
add_executable(gtsam_ADAPT_3D src/adapt_3D.cpp src/utils.cpp)
add_executable(gtsam_PCM_2D src/rpgo_2D.cpp src/utils.cpp)
add_executable(gtsam_PCM_3D src/rpgo_3D.cpp src/utils.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(gtsam_HUBER_2D
gtsam
${MKL_LIBRARIES}
  tbb
  tbbmalloc
  ${YAML_CPP_LIBRARIES}
)

## Specify libraries to link a library or executable target against
target_link_libraries(gtsam_HUBER_3D
gtsam
${MKL_LIBRARIES}
  tbb
  tbbmalloc
  ${YAML_CPP_LIBRARIES}
)

## Specify libraries to link a library or executable target against
target_link_libraries(gtsam_GNC_2D
gtsam
  tbb
  tbbmalloc
  ${YAML_CPP_LIBRARIES}
)

target_link_libraries(gtsam_GNC_3D
gtsam
${MKL_LIBRARIES}
  tbb
  tbbmalloc
  ${YAML_CPP_LIBRARIES}
)

target_link_libraries(gtsam_DCS_2D
gtsam
${MKL_LIBRARIES}
  tbb
  tbbmalloc
  ${YAML_CPP_LIBRARIES}
)

target_link_libraries(gtsam_DCS_3D
gtsam
${MKL_LIBRARIES}
  tbb
  tbbmalloc
  ${YAML_CPP_LIBRARIES}
)

target_link_libraries(gtsam_GM_2D
gtsam
${MKL_LIBRARIES}
  tbb
  tbbmalloc
  ${YAML_CPP_LIBRARIES}
)

target_link_libraries(gtsam_GM_3D
gtsam
${MKL_LIBRARIES}
  tbb
  tbbmalloc
  ${YAML_CPP_LIBRARIES}
)

target_link_libraries(gtsam_ADAPT_2D
gtsam
${MKL_LIBRARIES}
  tbb
  tbbmalloc
  ${YAML_CPP_LIBRARIES}
)

target_link_libraries(gtsam_ADAPT_3D
gtsam
${MKL_LIBRARIES}
  tbb
  tbbmalloc
  ${YAML_CPP_LIBRARIES}
)

target_link_libraries(gtsam_PCM_2D
gtsam
tbb
tbbmalloc
KimeraRPGO
${YAML_CPP_LIBRARIES}
)

target_link_libraries(gtsam_PCM_3D
gtsam
tbb
tbbmalloc
KimeraRPGO
${YAML_CPP_LIBRARIES}
)
