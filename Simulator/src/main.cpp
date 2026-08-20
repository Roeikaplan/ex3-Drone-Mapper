/**
 * @file main.cpp
 * @brief Phase-01 driver: exercises the plugin lifecycle end to end.
 * @note Deliberately thin. Real argument handling arrives in phase 02 with `CommandLineArgs`, which
 *       replaces this file's parsing but keeps calling into `runPluginSmokeCheck`'s successor.
 * @note Returns from `main` on every path and never calls `exit()`, per the assignment's rule that
 *       the program always ends by finishing `main`.
 */

#include <Simulator/PluginSmokeCheck.h>

#include <filesystem>
#include <iostream>

/**
 * @brief Entry point.
 * @param argc Argument count.
 * @param argv `argv[1]` is the algorithms folder, `argv[2]` the mission-controls folder.
 * @return 0 when the smoke check passed or the arguments were missing; 1 when a plugin failed.
 * @note Missing arguments print usage and return 0: this driver is scaffolding, and treating a bare
 *       invocation as a hard failure would be noise.
 */
int main(int argc, char** argv) {
    if (argc < 3) {
        std::cerr << "usage: " << (argc > 0 ? argv[0] : "simulator")
                  << " <algorithms_folder> <mission_controls_folder>\n";
        return 0;
    }

    return simulator::runPluginSmokeCheck(std::filesystem::path{argv[1]},
                                          std::filesystem::path{argv[2]});
}
