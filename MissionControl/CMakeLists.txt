# MissionControl plugin: built as a shared library and dynamically loaded by the simulator.
# It owns its own DroneControl and scan-to-voxel conversion, so common_mission_control/include
# is PRIVATE - IDroneControl.h is internal to this subsystem and must not propagate.

add_library(MissionControl_323998450_211633813 SHARED
    src/DroneControlImpl.cpp
    src/MissionControlImpl.cpp
    src/MissionControlRegistration.cpp
    src/ScanResultToVoxels.cpp
)

target_include_directories(MissionControl_323998450_211633813
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/common_mission_control/include
        ${CMAKE_CURRENT_SOURCE_DIR}/../UserCommon/include
)

target_link_libraries(MissionControl_323998450_211633813
    PRIVATE
        common::common
)

# PREFIX "": the mandated artifact name is MissionControl_<id1>_<id2>.so.
# Do NOT add -Wl,--no-undefined - see the note in Algorithm/CMakeLists.txt.
set_target_properties(MissionControl_323998450_211633813 PROPERTIES
    PREFIX ""
    LIBRARY_OUTPUT_DIRECTORY "${DRONE_PLUGIN_OUTPUT_ROOT}/mission_controls"
)

drone_warnings(MissionControl_323998450_211633813)

# ---------------------------------------------------------------------------------------------
# Unit tests.
#
# A target of its own rather than a home in simulator_unit_test: these classes live in a different
# project and link only common::common, so the simulator's test binary cannot reach them. Their
# fakes are hand-written for the same reason - Map3DImpl is Simulator-side and off limits here.
# ---------------------------------------------------------------------------------------------

add_executable(mission_control_unit_test
    tests/ScanResultToVoxelsTest.cpp
    tests/DroneControlTest.cpp
    tests/MissionControlTest.cpp
    src/DroneControlImpl.cpp
    src/MissionControlImpl.cpp
    src/ScanResultToVoxels.cpp
)

target_include_directories(mission_control_unit_test
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/common_mission_control/include
        ${CMAKE_CURRENT_SOURCE_DIR}/../UserCommon/include
)

target_link_libraries(mission_control_unit_test
    PRIVATE
        common::common
        GTest::gtest_main
)

drone_warnings(mission_control_unit_test)
gtest_discover_tests(mission_control_unit_test)
