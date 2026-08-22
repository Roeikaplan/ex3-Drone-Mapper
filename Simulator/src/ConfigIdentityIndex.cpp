/**
 * @file ConfigIdentityIndex.cpp
 * @brief Building and querying the address-to-source-name map.
 */

#include <Simulator/ConfigIdentityIndex.h>

#include <cstddef>
#include <tuple>

namespace simulator {
namespace {

/**
 * @brief The name used when an address is not in the index.
 * @note Deliberately conspicuous. The realistic way to get here is a composition that was copied
 *       after indexing, and a filename full of "unknown" is a far better symptom than a plausible
 *       but wrong one.
 */
constexpr const char* kUnknown = "unknown";

/**
 * @brief Stand-in for a missing per-group path list.
 * @note A named constant rather than a temporary in the conditional below: binding a reference to
 *       one branch of a conditional forces both branches into a materialised temporary, which would
 *       copy the real path list on every group for no reason.
 */
const std::vector<std::filesystem::path> kNoPaths{};

/**
 * @brief Pick a display name for one config.
 * @param paths The recorded source paths for this config kind.
 * @param index Position of the config within its vector.
 * @param prefix Word used to build a positional name when no path was recorded.
 * @return The path's stem, or `<prefix><index>`.
 * @note The positional fallback keeps output-map filenames distinguishable when `loadComposition`
 *       was called without a `CompositionPaths` sink, as component tests do.
 */
[[nodiscard]] std::string nameFor(const std::vector<std::filesystem::path>& paths,
                                  std::size_t index, const char* prefix) {
    if (index < paths.size() && !paths[index].empty()) {
        return paths[index].stem().string();
    }
    return prefix + std::to_string(index);
}

} // namespace

/**
 * @brief Index every config in a composition against its source file.
 * @param composition The parsed composition; must outlive this object.
 * @param paths Source paths recorded by `loadComposition`, index-parallel to @p composition.
 * @note Addresses are taken from the composition's own storage, so the entries stay valid exactly as
 *       long as the composition does and no longer.
 */
ConfigIdentityIndex::ConfigIdentityIndex(const types::SimulationCompositionData& composition,
                                         const CompositionPaths& paths) {
    for (std::size_t g = 0; g < composition.simulation_mission_groups.size(); ++g) {
        const auto& group = composition.simulation_mission_groups[g];

        names_.emplace(static_cast<const void*>(&std::get<0>(group)),
                       nameFor(paths.simulation_paths, g, "simulation"));

        const std::vector<common::types::MissionConfigData>& missions = std::get<1>(group);
        for (std::size_t m = 0; m < missions.size(); ++m) {
            const std::vector<std::filesystem::path>& mission_paths =
                g < paths.mission_paths.size() ? paths.mission_paths[g] : kNoPaths;
            names_.emplace(static_cast<const void*>(&missions[m]),
                           nameFor(mission_paths, m, "mission"));
        }
    }

    for (std::size_t d = 0; d < composition.drone_configs.size(); ++d) {
        names_.emplace(static_cast<const void*>(&composition.drone_configs[d]),
                       nameFor(paths.drone_paths, d, "drone"));
    }

    for (std::size_t l = 0; l < composition.lidar_configs.size(); ++l) {
        names_.emplace(static_cast<const void*>(&composition.lidar_configs[l]),
                       nameFor(paths.lidar_paths, l, "lidar"));
    }
}

/**
 * @brief The name of the file a simulation config came from.
 * @param config A config belonging to the indexed composition.
 * @return Its source-file stem, a positional name, or the fallback.
 */
std::string ConfigIdentityIndex::nameOf(const types::SimulationConfigData& config) const {
    return lookup(static_cast<const void*>(&config));
}

/**
 * @brief The name of the file a mission config came from.
 * @param config A config belonging to the indexed composition.
 * @return Its source-file stem, a positional name, or the fallback.
 */
std::string ConfigIdentityIndex::nameOf(const common::types::MissionConfigData& config) const {
    return lookup(static_cast<const void*>(&config));
}

/**
 * @brief The name of the file a drone config came from.
 * @param config A config belonging to the indexed composition.
 * @return Its source-file stem, a positional name, or the fallback.
 */
std::string ConfigIdentityIndex::nameOf(const common::types::DroneConfigData& config) const {
    return lookup(static_cast<const void*>(&config));
}

/**
 * @brief The name of the file a lidar config came from.
 * @param config A config belonging to the indexed composition.
 * @return Its source-file stem, a positional name, or the fallback.
 */
std::string ConfigIdentityIndex::nameOf(const common::types::LidarConfigData& config) const {
    return lookup(static_cast<const void*>(&config));
}

/**
 * @brief Look up one address.
 * @param address Address of a config object.
 * @return The recorded name, or the fallback.
 * @note Const and non-mutating, which is what makes concurrent `create()` calls safe without a lock.
 */
std::string ConfigIdentityIndex::lookup(const void* address) const {
    const auto found = names_.find(address);
    return found == names_.end() ? std::string{kUnknown} : found->second;
}

} // namespace simulator
