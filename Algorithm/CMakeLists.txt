# Mapping algorithm plugin: built as a shared library and dynamically loaded by the simulator.
# It links common::common and nothing else - nothing inside a plugin parses YAML or reads .npy;
# configs arrive pre-parsed in MappingAlgorithmDependencies.

add_library(Algorithm_323998450_211633813 SHARED
    src/MappingAlgorithmImpl.cpp
    src/MappingAlgorithmRegistration.cpp
)

target_include_directories(Algorithm_323998450_211633813
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/../UserCommon/include
)

target_link_libraries(Algorithm_323998450_211633813
    PRIVATE
        common::common
)

# PREFIX "": the mandated artifact name is Algorithm_<id1>_<id2>.so, not libAlgorithm_<...>.so.
# The target is named after the artifact, so no OUTPUT_NAME is needed.
#
# Do NOT add -Wl,--no-undefined to this target. The plugin deliberately ships with an UNDEFINED
# symbol for ::common::MappingAlgorithmRegistration's constructor, which is defined in the
# simulator executable and resolved at dlopen time.
set_target_properties(Algorithm_323998450_211633813 PROPERTIES
    PREFIX ""
    LIBRARY_OUTPUT_DIRECTORY "${DRONE_PLUGIN_OUTPUT_ROOT}/algorithms"
)

drone_warnings(Algorithm_323998450_211633813)

# ---------------------------------------------------------------------------------------------
# Unit tests.
#
# A target of its own, for the same reason MissionControl needed one: this project links only
# common::common, so neither of the other test binaries can reach these classes. Its map is
# hand-built rather than a Map3DImpl, which lives on the Simulator side and is off limits here.
# ---------------------------------------------------------------------------------------------

add_executable(algorithm_unit_test
    tests/MappingAlgorithmTest.cpp
    src/MappingAlgorithmImpl.cpp
)

target_include_directories(algorithm_unit_test
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/../UserCommon/include
)

target_link_libraries(algorithm_unit_test
    PRIVATE
        common::common
        GTest::gtest_main
)

drone_warnings(algorithm_unit_test)
gtest_discover_tests(algorithm_unit_test)
