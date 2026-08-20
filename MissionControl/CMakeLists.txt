# MissionControl plugin: built as a shared library and dynamically loaded by the simulator.
# It owns its own DroneControl and scan-to-voxel conversion, so common_mission_control/include
# is PRIVATE - IDroneControl.h is internal to this subsystem and must not propagate.

add_library(MissionControl_323998450_211633813 SHARED
    src/Placeholder.cpp
)

target_include_directories(MissionControl_323998450_211633813
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/common_mission_control/include
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
