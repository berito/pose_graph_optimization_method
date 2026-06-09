add_executable(IN_DCS_2D dcs_2D.cpp)
target_link_libraries(IN_DCS_2D PRIVATE g2o_common)

add_executable(IN_DCS_3D dcs_3D.cpp)
target_link_libraries(IN_DCS_3D PRIVATE g2o_common)

add_executable(IN_HUBER_2D huber_2D.cpp)
target_link_libraries(IN_HUBER_2D PRIVATE g2o_common)

add_executable(IN_HUBER_3D huber_3D.cpp)
target_link_libraries(IN_HUBER_3D PRIVATE g2o_common)

add_executable(IN_MAXMIX_2D maxmix_2D.cpp)
target_link_libraries(IN_MAXMIX_2D PRIVATE g2o_common maxmixlib)

add_executable(IN_MAXMIX_3D maxmix_3D.cpp)
target_link_libraries(IN_MAXMIX_3D PRIVATE g2o_common maxmixlib)

add_executable(IN_SC_2D sc_2D.cpp)
target_link_libraries(IN_SC_2D PRIVATE g2o_common switchlib)

add_executable(IN_SC_3D sc_3D.cpp)
target_link_libraries(IN_SC_3D PRIVATE g2o_common switchlib)

add_executable(IN_GNC_2D gnc_2D.cpp)
target_link_libraries(IN_GNC_2D PRIVATE g2o_common gncg2olib)

add_executable(IN_GNC_3D gnc_3D.cpp)
target_link_libraries(IN_GNC_3D PRIVATE g2o_common gncg2olib)

add_executable(IN_RRR_2D rrr_2D.cpp)
target_link_libraries(IN_RRR_2D PRIVATE g2o_common rrrlib)

add_executable(IN_RRR_3D rrr_3D.cpp)
target_link_libraries(IN_RRR_3D PRIVATE g2o_common rrrlib)