# Simulator executable: the plugin host. It owns the mocks, the maps, the composition loader,
# the scoring, and the report writers, so it is the only target that needs yaml-cpp and TinyNPY.
# ${CMAKE_DL_LIBS} provides dlopen/dlsym/dlclose.

add_executable(simulator_323998450_211633813
    src/main.cpp
    src/CommandLineArgs.cpp
    src/CompositionLoader.cpp
    src/ErrorLogger.cpp
    src/Map3DImpl.cpp
    src/MapsComparison.cpp
    src/MockGPS.cpp
    src/MockLidar.cpp
    src/MockMovement.cpp
    src/PluginLibrary.cpp
    src/PluginLoader.cpp
    src/PluginSmokeCheck.cpp
    src/Registrar.cpp
    src/RegistrationEntryPoints.cpp
    src/ResultsDirectory.cpp
)

target_include_directories(simulator_323998450_211633813
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/common_simulator/include
)

target_link_libraries(simulator_323998450_211633813
    PRIVATE
        common::common
        yaml-cpp::yaml-cpp
        TinyNPY::TinyNPY
        ${CMAKE_DL_LIBS}
)

# Plugins ship with an UNDEFINED symbol for their registration constructor, which is defined in
# THIS executable (src/RegistrationEntryPoints.cpp) and resolved against it at dlopen time.
# Without exported dynamic symbols the build still succeeds and dlopen fails at runtime with
# "undefined symbol".
set_target_properties(simulator_323998450_211633813 PROPERTIES
    ENABLE_EXPORTS ON
)

drone_warnings(simulator_323998450_211633813)

# ---------------------------------------------------------------------------------------------
# Test fixtures: do-nothing plugins, kept permanently as test doubles.
#
# They are separate from the real Algorithm_/MissionControl_ targets on purpose. Comparative mode
# groups mission controls by behaviour, so it needs two or more MissionControl .so files to have
# anything to group; competitive mode needs a folder holding several algorithms. Building them
# here means phases 04 and 07 have known-good inputs that never change as the real plugins evolve.
# ---------------------------------------------------------------------------------------------

# Declare one fixture plugin.
#   target - CMake target and artifact stem (no "lib" prefix is emitted)
#   source - the fixture translation unit
#   kind   - "algorithms" or "mission_controls"; selects the output folder so a whole folder can
#            be handed to -comparative / -competition directly
function(drone_fixture_plugin target source kind)
    add_library(${target} SHARED ${source})
    target_link_libraries(${target} PRIVATE common::common)
    set_target_properties(${target} PROPERTIES
        PREFIX ""
        LIBRARY_OUTPUT_DIRECTORY "${DRONE_PLUGIN_OUTPUT_ROOT}/fixtures/${kind}"
    )
    drone_warnings(${target})
endfunction()

drone_fixture_plugin(StubAlgorithm_A      tests/fixtures/StubAlgorithm.cpp      algorithms)
drone_fixture_plugin(StubMissionControl_A tests/fixtures/StubMissionControl.cpp mission_controls)
drone_fixture_plugin(StubMissionControl_B tests/fixtures/StubMissionControl.cpp mission_controls)

# The two mission-control fixtures differ only in the step count they report, which is exactly the
# axis comparative grouping keys on: same source, different observable behaviour.
target_compile_definitions(StubMissionControl_A PRIVATE STUB_MC_STEPS=1)
target_compile_definitions(StubMissionControl_B PRIVATE STUB_MC_STEPS=2)

# ---------------------------------------------------------------------------------------------
# Unit tests.
#
# CommandLineArgs has no dependency on the plugin machinery, so its test compiles the one source
# under test directly. That deliberately avoids moving the Simulator's sources into a static
# library: RegistrationEntryPoints.cpp is referenced by nothing in the executable - only by plugins
# at dlopen time - so an archive would let the linker drop it and the registration symbol would
# silently vanish from .dynsym, breaking plugin loading with no build error.
# ---------------------------------------------------------------------------------------------

include(GoogleTest)

add_executable(simulator_unit_test
    tests/CommandLineArgsTest.cpp
    tests/CompositionLoaderTest.cpp
    tests/ErrorLoggerTest.cpp
    tests/Map3DImplTest.cpp
    tests/MapsComparisonTest.cpp
    tests/MocksTest.cpp
    tests/ResultsDirectoryTest.cpp
    src/CommandLineArgs.cpp
    src/CompositionLoader.cpp
    src/ErrorLogger.cpp
    src/Map3DImpl.cpp
    src/MapsComparison.cpp
    src/MockGPS.cpp
    src/MockLidar.cpp
    src/MockMovement.cpp
    src/PluginLibrary.cpp
    src/PluginLoader.cpp
    src/Registrar.cpp
    src/ResultsDirectory.cpp
)

target_include_directories(simulator_unit_test
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/common_simulator/include
)

target_link_libraries(simulator_unit_test
    PRIVATE
        common::common
        yaml-cpp::yaml-cpp
        TinyNPY::TinyNPY
        GTest::gtest_main
        ${CMAKE_DL_LIBS}
)

# Integration-style cases read the shipped inputs/ dataset by absolute path, so they behave the same
# whatever working directory ctest happens to launch them from.
target_compile_definitions(simulator_unit_test
    PRIVATE
        DRONE_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}/.."
)

drone_warnings(simulator_unit_test)
gtest_discover_tests(simulator_unit_test)
