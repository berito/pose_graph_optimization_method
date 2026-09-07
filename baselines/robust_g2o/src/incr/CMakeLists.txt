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

# ⚠ PATCHED COPY, deliberately. rrr_2D.cpp/rrr_3D.cpp do not compile against this g2o
# (ambiguous `make_unique`), and RRR is a COMPARATOR — editing its source would mean we
# are no longer comparing against the published method. The two source files in this
# directory are byte-identical to upstream; the fixed copies live in vendor_patched/ and
# are what compile. Recorded in scripts/vendor_patches.txt; checked by scripts/check_vendor.sh.
set(VENDOR_PATCHED ${CMAKE_CURRENT_SOURCE_DIR}/../../../../vendor_patched/robust_g2o/incr)

add_executable(IN_RRR_2D ${VENDOR_PATCHED}/rrr_2D.cpp)
target_link_libraries(IN_RRR_2D PRIVATE g2o_common rrrlib)

add_executable(IN_RRR_3D ${VENDOR_PATCHED}/rrr_3D.cpp)
target_link_libraries(IN_RRR_3D PRIVATE g2o_common rrrlib)