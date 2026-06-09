add_executable(DCS_2D dcs_2D.cpp)
target_link_libraries(DCS_2D PRIVATE g2o_common)

add_executable(DCS_3D dcs_3D.cpp)
target_link_libraries(DCS_3D PRIVATE g2o_common)

add_executable(HUBER_2D huber_2D.cpp)
target_link_libraries(HUBER_2D PRIVATE g2o_common)

add_executable(HUBER_3D huber_3D.cpp)
target_link_libraries(HUBER_3D PRIVATE g2o_common)

add_executable(MAXMIX_2D maxmix_2D.cpp)
target_link_libraries(MAXMIX_2D PRIVATE maxmixlib g2o_common)

add_executable(MAXMIX_3D maxmix_3D.cpp)
target_link_libraries(MAXMIX_3D PRIVATE maxmixlib g2o_common )

add_executable(SC_2D sc_2D.cpp)
target_link_libraries(SC_2D PRIVATE g2o_common switchlib)

add_executable(SC_3D sc_3D.cpp)
target_link_libraries(SC_3D PRIVATE g2o_common switchlib)

add_executable(RRR_2D rrr_2D.cpp)
target_link_libraries(RRR_2D PRIVATE g2o_common rrrlib)

add_executable(RRR_3D rrr_3D.cpp)
target_link_libraries(RRR_3D PRIVATE g2o_common rrrlib)

add_executable(GNC_2D gnc_2D.cpp)
target_link_libraries(GNC_2D PRIVATE g2o_common gncg2olib)

add_executable(GNC_3D gnc_3D.cpp)
target_link_libraries(GNC_3D PRIVATE g2o_common gncg2olib)