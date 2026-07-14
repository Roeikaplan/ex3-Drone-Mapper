#pragma once

#include <Simulator/ISimulationRun.h>

#include <memory>

namespace Simulator {

class ISimulationRunFactory {
public:
    virtual ~ISimulationRunFactory() = default;
    [[nodiscard]] virtual std::unique_ptr<ISimulationRun> create(
        const types::SimulationConfigData& simulation_config,
        const Common::types::MissionConfigData& mission_config,
        const Common::types::DroneConfigData& drone_config,
        const Common::types::LidarConfigData& lidar_config,
        const std::filesystem::path& output_path) = 0;
};

} // namespace Simulator
