#pragma once

#include <drone_mapper/ISimulationRun.h>
#include <drone_mapper/Types.h>

#include <filesystem>
#include <memory>

namespace drone_mapper {

/**
 * @brief Contract for the single construction seam that wires one simulation run.
 *
 * Lets the manager obtain a ready-to-run `ISimulationRun` for each `(simulation, mission, drone,
 * lidar)` combination without knowing how the object graph is assembled or owned.
 *
 * @note **Do not change this interface.**
 */
class ISimulationRunFactory {
public:
    virtual ~ISimulationRunFactory() = default;

    /**
     * @brief Build a fully-wired run for one simulation/mission/drone/lidar combination.
     * @param simulation Hidden map path/resolution/offset and initial drone pose.
     * @param mission Mission behaviour, boundaries, and requested output resolution.
     * @param drone Drone capabilities.
     * @param lidar LiDAR sensor configuration.
     * @param output_path Base directory the run writes its output map under.
     * @return An owning `ISimulationRun` with all dependencies injected.
     */
    [[nodiscard]] virtual std::unique_ptr<ISimulationRun>
    create(const types::SimulationConfigData& simulation,
           const types::MissionConfigData& mission,
           const types::DroneConfigData& drone,
           const types::LidarConfigData& lidar,
           const std::filesystem::path& output_path) = 0;
};

} // namespace drone_mapper
