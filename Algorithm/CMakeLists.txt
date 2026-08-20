# Mapping algorithm plugin: built as a shared library and dynamically loaded by the simulator.
# It links common::common and nothing else - nothing inside a plugin parses YAML or reads .npy;
# configs arrive pre-parsed in MappingAlgorithmDependencies.

add_library(Algorithm_323998450_211633813 SHARED
    src/Placeholder.cpp
)

target_include_directories(Algorithm_323998450_211633813
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
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
