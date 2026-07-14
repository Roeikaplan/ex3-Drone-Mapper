#pragma once

#include <Common/Types.h>

#include <filesystem>
#include <string>
#include <tuple>
#include <vector>

namespace Simulator::types {

struct SimulationConfigData {
    std::filesystem::path map_filename{};
    Common::PhysicalLength map_resolution{};
    Common::Position3D map_offset{};
    Common::Position3D initial_drone_position{};
    Common::HorizontalAngle initial_angle{};
};

struct SimulationCompositionData {
    std::filesystem::path composition_file{};
    std::vector<std::tuple<SimulationConfigData, std::vector<Common::types::MissionConfigData>>> simulation_mission_groups{};
    std::vector<Common::types::DroneConfigData> drone_configs{};
    std::vector<Common::types::LidarConfigData> lidar_configs{};
};

enum class ResolutionRequestStatus { Accepted, Ignored, IgnoredTooSmall };

struct SimulationResult {
    SimulationConfigData simulation_config{};
    Common::types::MissionConfigData mission_config{};
    ResolutionRequestStatus resolution_request_status = ResolutionRequestStatus::Ignored;
    std::vector<Common::types::MissionRunResult> mission_results{};
    std::filesystem::path output_map_file{};
    Common::types::MapConfig output_map_config{};
    double mission_score = 0.0;
};

struct SimulationManagerReport {
    std::filesystem::path composition_file{};
    std::string generated_at_utc{};
    std::string metric{};
    std::tuple<double, double> score_range{};
    int error_score = -1;
    std::vector<SimulationResult> runs{};
};

} // namespace Simulator::types
