cmake_minimum_required(VERSION 3.15)
project(assignment_3 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(mp-units CONFIG REQUIRED)    # target: mp-units::mp-units
find_package(tinynpy CONFIG REQUIRED)     # target: TinyNPY::TinyNPY  (note the capitalisation)
find_package(yaml-cpp CONFIG REQUIRED)    # target: yaml-cpp::yaml-cpp
find_package(GTest CONFIG REQUIRED)       # targets: GTest::gtest_main, GTest::gmock

# Must be called at top level for ctest to see the suite from the top build directory.
# Test targets themselves arrive with the code they test, starting in phase 02.
enable_testing()

# Both plugin kinds are enumerated as *folders* by the simulator's two run modes, so each kind
# gets its own output folder that -comparative / -competition can be pointed straight at.
set(DRONE_PLUGIN_OUTPUT_ROOT "${CMAKE_BINARY_DIR}/plugins")

function(drone_warnings target)
    target_compile_options(${target} PRIVATE -Wall -Wextra -Werror -pedantic)
endfunction()

add_subdirectory(common)
add_subdirectory(Algorithm)
add_subdirectory(MissionControl)
add_subdirectory(Simulator)
