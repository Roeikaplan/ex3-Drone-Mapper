/**
 * @file SimulationRunFactoryImpl.h
 * @brief The single wiring point: builds one fully connected run from four configs.
 */

#pragma once

#include <Simulator/ConfigIdentityIndex.h>
#include <Simulator/ISimulationRunFactory.h>

#include <Common/MappingAlgorithmFactory.h>
#include <Common/MissionControlFactory.h>

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
 * @note **It holds those factories by value**, so their targets live in plugin code. This object
 *       must therefore be destroyed before any plugin library is unloaded.
 */
class SimulationRunFactoryImpl final : public ISimulationRunFactory {
public:
    /**
     * @brief Bind a factory to one plugin pair.
     * @param mission_control_factory Produces the mission control for every run this builds.
     * @param algorithm_factory Produces the mapping algorithm for every run this builds.
     * @param plugin_label Name used to distinguish this pair's output files from another pair's.
     * @param identity Source-file names for the configs, used to name output maps; must outlive
     *        this object.
     * @param verbose Whether missions should be asked to write verbose output.
     */
    SimulationRunFactoryImpl(common::MissionControlFactory mission_control_factory,
                             common::MappingAlgorithmFactory algorithm_factory,
                             std::string plugin_label, const ConfigIdentityIndex& identity,
                             bool verbose);

    /**
     * @brief Build one run.
     * @param simulation_config Ground-truth map and starting pose.
     * @param mission_config Step budget, bounds, and resolution request.
     * @param drone_config The vehicle's limits.
     * @param lidar_config The sensor's geometry.
     * @param output_path Directory the run's output map is written into.
     * @return A run ready to execute.
     * @throws std::runtime_error when the ground-truth map cannot be read.
     * @note Throwing is deliberate and contained: `SimulationManager` catches, logs, and scores the
     *       affected combination -1, which is what the assignment prescribes for a bad map file.
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

    common::MissionControlFactory mission_control_factory_;
    common::MappingAlgorithmFactory algorithm_factory_;
    std::string plugin_label_;
    const ConfigIdentityIndex& identity_;
    bool verbose_;
};

} // namespace simulator
