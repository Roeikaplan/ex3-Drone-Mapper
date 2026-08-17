#pragma once

#include <filesystem>
#include <vector>

namespace drone_mapper {

/**
 * @brief Source config-file paths for a composition, parallel to `SimulationCompositionData`.
 *
 * Records the path string of each config as written in the composition file (the mandated
 * file-reference layout), so the score report can label each simulation/mission/drone/lidar by its
 * source path — matching the assignment's `simulation_output.yaml` sample. Entries are empty for the
 * inline layout (where a config has no source file).
 *
 * @note Kept **outside** the skeleton data types on purpose: it carries the report-labelling metadata
 *       without modifying the provided `SimulationCompositionData` / `SimulationResult` structs. The
 *       vectors are index-parallel to the composition's config vectors (and to the order in which
 *       `SimulationManager` expands runs), which is how the writer associates each run with its paths.
 */
struct CompositionPaths {
    /// Parallel to `SimulationCompositionData::simulation_mission_groups` (one path per group).
    std::vector<std::filesystem::path> simulation_paths;
    /// Parallel to each group's mission list (`mission_paths[g][m]`).
    std::vector<std::vector<std::filesystem::path>> mission_paths;
    /// Parallel to `SimulationCompositionData::drones`.
    std::vector<std::filesystem::path> drone_paths;
    /// Parallel to `SimulationCompositionData::lidars`.
    std::vector<std::filesystem::path> lidar_paths;
};

} // namespace drone_mapper
