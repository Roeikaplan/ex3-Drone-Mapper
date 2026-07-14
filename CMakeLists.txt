cmake_minimum_required(VERSION 3.15)
project(assignment_3 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(mp-units CONFIG REQUIRED)

function(drone_warnings target)
    target_compile_options(${target} PRIVATE -Wall -Wextra -Werror -pedantic)
endfunction()

add_subdirectory(common)
add_subdirectory(Algorithm)
add_subdirectory(MissionControl)
add_subdirectory(Simulator)
