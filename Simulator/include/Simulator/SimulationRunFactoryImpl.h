/**
 * @file SimulationRunFactoryImpl.h
 * @brief The single wiring point: builds one fully connected run from four configs.
 */

#pragma once

#include <Simulator/ConfigIdentityIndex.h>
#include <Simulator/ISimulationRunFactory.h>
#include <Simulator/PluginRegistry.h>

#include <filesystem>
#include <memory>
#include <string>

namespace simulator {

/**
 * @brief Builds simulation runs for one fixed pair of plugins.
 *
 * @note Architectural boundary: **this instance is bound at construction to one
 *       `(MissionControlFactory, MappingAlgorithmFactory)` pair.** The frozen `create()` signature
 *       has no room for a plugin, and neither does `ISimulation::run`, so the plugin dimension is
 *       closed here instead - by the constructor, which is ours to define. One consequence worth
 *       naming: each run becomes a pure function of its configs and this pair, with no shared
 *       mutable state, which is what will make concurrent execution safe by construction rather
 *       than by locking.
 * @note This is the only place that knows concrete types. Everything else works through interfaces,
 *       and two of the "concrete types" here are opaque - produced by `std::function`s handed over
 *       by libraries loaded at runtime.
 * @note **It holds no factory of its own - it borrows one from each slot per call.** That is what
 *       makes the lazy lifecycle possible: an owned copy of a plugin's `std::function` would keep
 *       that library's code reachable, and therefore un-unloadable, for as long as this object
 *       lived. Asking the registry each time also means the first run of a plugin is what maps it.
 * @note The borrowed factory is valid only while the calling run holds its reserved use - which it
 *       does for the whole of `create()` and beyond, until its `PluginUseGuard` is destroyed.
 */
class SimulationRunFactoryImpl final : public ISimulationRunFactory {
public:
    /**
     * @brief Bind a factory to one plugin pair, named by slot rather than by factory.
     * @param registry Owns the two libraries and performs their loads; must outlive this object.
     * @param mission_control_slot The mission-control plugin every run this builds will use.
     * @param algorithm_slot The algorithm plugin every run this builds will use.
     * @param plugin_label Name used to distinguish this pair's output files from another pair's.
     * @param identity Source-file names for the configs, used to name output maps; must outlive
     *        this object.
     * @param verbose Whether missions should be asked to write verbose output.
     * @note Constructing this loads nothing. The pair is named here and mapped later, by whichever
     *       run gets to it first.
     */
    SimulationRunFactoryImpl(PluginRegistry& registry, PluginSlot& mission_control_slot,
                             PluginSlot& algorithm_slot, std::string plugin_label,
                             const ConfigIdentityIndex& identity, bool verbose);

    /**
     * @brief Build one run.
     * @param simulation_config Ground-truth map and starting pose.
     * @param mission_config Step budget, bounds, and resolution request.
     * @param drone_config The vehicle's limits.
     * @param lidar_config The sensor's geometry.
     * @param output_path Directory the run's output map is written into.
     * @return A run ready to execute.
     * @throws std::runtime_error when the ground-truth map cannot be read.
     * @throws PluginUnavailable when either plugin library could not be loaded.
     * @note Throwing is deliberate and contained: `SimulationManager` catches, logs, and scores the
     *       affected combination -1, which is what the assignment prescribes for a bad map file -
     *       and, now that loading happens here, for an unloadable plugin too.
     * @note A fresh instance of each plugin is created per call and never cached. That is required,
     *       and it is also what keeps runs independent of one another.
     */
    [[nodiscard]] std::unique_ptr<ISimulationRun> create(
        const types::SimulationConfigData& simulation_config,
        const common::types::MissionConfigData& mission_config,
        const common::types::DroneConfigData& drone_config,
        const common::types::LidarConfigData& lidar_config,
        const std::filesystem::path& output_path) override;

private:
    /**
     * @brief Compose the output map's filename from the four configs and the plugin.
     * @param output_path Directory the file belongs in.
     * @param simulation_config Simulation this run uses.
     * @param mission_config Mission this run uses.
     * @param drone_config Drone this run uses.
     * @param lidar_config Lidar this run uses.
     * @return An absolute path naming every dimension of the run.
     * @note Derived from identity rather than a counter, so the same combination always produces the
     *       same filename no matter what order runs execute in.
     */
    [[nodiscard]] std::filesystem::path outputMapFile(
        const std::filesystem::path& output_path,
        const types::SimulationConfigData& simulation_config,
        const common::types::MissionConfigData& mission_config,
        const common::types::DroneConfigData& drone_config,
        const common::types::LidarConfigData& lidar_config) const;

    PluginRegistry& registry_;
    PluginSlot& mission_control_slot_;
    PluginSlot& algorithm_slot_;
    std::string plugin_label_;
    const ConfigIdentityIndex& identity_;
    bool verbose_;
};

} // namespace simulator
