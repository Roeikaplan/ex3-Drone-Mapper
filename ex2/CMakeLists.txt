cmake_minimum_required(VERSION 3.15)
project(drone_mapper LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 20)


find_package(mp-units CONFIG REQUIRED)
find_package(tinynpy CONFIG REQUIRED)
find_package(yaml-cpp REQUIRED) # new!
find_package(GTest CONFIG REQUIRED)

add_library(drone_mapper
    src/ComparisonConfig.cpp
    src/CompositionLoader.cpp
    src/DroneControlImpl.cpp
    src/ErrorLogger.cpp
    src/MappingAlgorithmImpl.cpp
    src/MapsComparison.cpp
    src/MissionControlImpl.cpp
    src/Map3DImpl.cpp
    src/MockGPS.cpp
    src/MockLidar.cpp
    src/MockMovement.cpp
    src/ScanResultToVoxels.cpp
    src/SimulationManager.cpp
    src/SimulationOutputWriter.cpp
    src/SimulationRunFactoryImpl.cpp
    src/SimulationRunImpl.cpp
)

target_compile_features(drone_mapper PUBLIC cxx_std_20)
target_compile_options(drone_mapper
    PRIVATE
        -Wall
        -Wextra
        -Werror
        -pedantic
)
target_include_directories(drone_mapper
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

target_link_libraries(drone_mapper
    PUBLIC
        mp-units::mp-units
        TinyNPY::TinyNPY
    PRIVATE
        yaml-cpp::yaml-cpp # used only in CompositionLoader/SimulationOutputWriter .cpp; not in headers
)

add_executable(drone_mapper_simulation src/drone_mapper_simulation_main.cpp)
target_link_libraries(drone_mapper_simulation PRIVATE drone_mapper)
#Compile the maps_comparison util
add_executable(maps_comparison src/maps_comparison_main.cpp)
target_link_libraries(maps_comparison PRIVATE drone_mapper)

# Compile the yaml example file
add_executable(example_yml cpp_yaml_example/main.cpp)
target_link_libraries(example_yml PRIVATE yaml-cpp::yaml-cpp)

# Component + integration tests (GTest/GMock)
enable_testing()
add_executable(drone_mapper_simulation_test
    tests/components/CompositionLoaderTest.cpp
    tests/components/DroneControlTest.cpp
    tests/components/MapsComparisonTest.cpp
    tests/components/MappingAlgorithmTest.cpp
    tests/components/MissionControlTest.cpp
    tests/components/MockLidarTest.cpp
    tests/components/SimulationManagerTest.cpp
    tests/components/SimulationRunTest.cpp
    tests/integration/RealAlgorithmFlowTest.cpp
    tests/integration/MockAlgorithmFlowTest.cpp
    tests/integration/InputScenariosTest.cpp
    tests/bonus/DifferentResolutionTest.cpp
)
target_compile_options(drone_mapper_simulation_test
    PRIVATE
        -Wall
        -Wextra
        -Werror
        -pedantic
)
# Integration tests read fixture maps from data_maps/ and inputs/ by absolute path, regardless of the
# working directory the test binary runs from.
target_compile_definitions(drone_mapper_simulation_test
    PRIVATE
        DRONE_MAPPER_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}"
)
target_link_libraries(drone_mapper_simulation_test
    PRIVATE
        drone_mapper
        GTest::gtest_main
        GTest::gmock
)

include(GoogleTest)
gtest_discover_tests(drone_mapper_simulation_test)
