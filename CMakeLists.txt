cmake_minimum_required(VERSION 3.15)
project(assignment_3 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(mp-units CONFIG REQUIRED)    # target: mp-units::mp-units
find_package(tinynpy CONFIG REQUIRED)     # target: TinyNPY::TinyNPY  (note the capitalisation)
find_package(yaml-cpp CONFIG REQUIRED)    # target: yaml-cpp::yaml-cpp
find_package(GTest CONFIG REQUIRED)       # targets: GTest::gtest_main, GTest::gmock

# The simulator runs its task table on a std::thread pool. On Linux/GCC that needs -pthread at both
# compile and link time, which Threads::Threads carries; without it std::thread links but aborts at
# run time. THREADS_PREFER_PTHREAD_FLAG must be set before the find to get the flag rather than a
# bare -lpthread.
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)            # target: Threads::Threads

# Must be called at top level for ctest to see the suite from the top build directory.
# Test targets themselves arrive with the code they test, starting in phase 02.
enable_testing()

# GoogleTest's helpers must be included before any subdirectory that uses them. Subprojects are
# added in dependency order below, so a per-subdirectory include() would only be visible to the
# subdirectories processed after it - which is how MissionControl ended up unable to see
# gtest_discover_tests despite Simulator including it.
include(GoogleTest)

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
