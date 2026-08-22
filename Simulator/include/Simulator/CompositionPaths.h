/**
 * @file CompositionPaths.h
 * @brief Source config-file paths for a composition, carried alongside the parsed data.
 */

#pragma once

#include <filesystem>
#include <vector>

namespace simulator {

/**
 * @brief The path each config was referenced by, index-parallel to `SimulationCompositionData`.
 *
 * The composition names its configs by path, but `SimulationCompositionData` stores only their
 * parsed values - a filename cannot be recovered from a `DroneConfigData`. The reports have to label
 * runs by source file, so the paths ride alongside in this separate structure.
 *
 * @note Architectural boundary: kept **outside** the provided skeleton types on purpose. The report
 *       needs this metadata, and `common/` and `common_simulator/` are frozen, so it travels as a
 *       side-channel rather than as a new field on `SimulationCompositionData`.
 * @note Every vector is index-parallel to the corresponding vector in the composition, and to the
 *       order in which runs are expanded. An entry is appended only when its group is actually
 *       accepted, so a skipped simulation cannot silently shift the alignment.
 */
struct CompositionPaths {
    /**
     * @brief One path per entry in `SimulationCompositionData::simulation_mission_groups`.
     */
    std::vector<std::filesystem::path> simulation_paths{};

    /**
     * @brief One inner vector per simulation group, holding that group's mission paths.
     * @note Indexed `mission_paths[group][mission]`, matching the nested shape of the composition.
     */
    std::vector<std::vector<std::filesystem::path>> mission_paths{};

    /**
     * @brief One path per entry in `SimulationCompositionData::drone_configs`.
     */
    std::vector<std::filesystem::path> drone_paths{};

    /**
     * @brief One path per entry in `SimulationCompositionData::lidar_configs`.
     */
    std::vector<std::filesystem::path> lidar_paths{};
};

} // namespace simulator
