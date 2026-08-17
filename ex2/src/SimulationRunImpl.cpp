#include <drone_mapper/SimulationRunImpl.h>

#include <drone_mapper/MapsComparison.h>

#include <stdexcept>
#include <utility>
#include <vector>

namespace drone_mapper {

SimulationRunImpl::SimulationRunImpl(std::unique_ptr<const IMap3D> hidden_map,
                                     std::unique_ptr<IMutableMap3D> output_map,
                                     std::unique_ptr<IGPS> gps,
                                     std::unique_ptr<IDroneMovement> movement,
                                     std::unique_ptr<ILidar> lidar,
                                     std::unique_ptr<IMappingAlgorithm> mapping_algorithm,
                                     std::unique_ptr<IDroneControl> drone_control,
                                     std::unique_ptr<IMissionControl> mission_control,
                                     types::SimulationConfigData simulation_config,
                                     types::MissionConfigData mission_config,
                                     types::ResolutionRequestStatus resolution_status,
                                     std::filesystem::path output_map_file)
    : hidden_map_(std::move(hidden_map)),
      output_map_(std::move(output_map)),
      gps_(std::move(gps)),
      movement_(std::move(movement)),
      lidar_(std::move(lidar)),
      mapping_algorithm_(std::move(mapping_algorithm)),
      drone_control_(std::move(drone_control)),
      mission_control_(std::move(mission_control)),
      simulation_config_(std::move(simulation_config)),
      mission_config_(std::move(mission_config)),
      resolution_status_(resolution_status),
      output_map_file_(std::move(output_map_file)) {
    if (!hidden_map_ ||
        !output_map_ ||
        !gps_ ||
        !movement_ ||
        !lidar_ ||
        !mapping_algorithm_ ||
        !drone_control_ ||
        !mission_control_) {
        throw std::invalid_argument("SimulationRunImpl requires injected dependencies.");
    }
}

types::SimulationResult SimulationRunImpl::run() {
    // Drive the bottom layer: MissionControl runs the step loop and saves the filled output map.
    types::MissionRunResult mission_result = mission_control_->runMission();

    // Score the drone's output against the hidden ground truth. A mission that errored is not
    // scored: it takes the error sentinel -1 and the manager proceeds to the next run
    // (an errored scenario -> score -1). Otherwise compare walks the hidden map's
    // grid (origin) and reports IoU 0-100.
    double score = -1.0;
    if (mission_result.status != types::MissionRunStatus::Error) {
        const std::vector<double> scores =
            MapsComparison::compare(*hidden_map_, {output_map_.get()});
        score = scores.empty() ? -1.0 : scores.front();
    }

    // Assemble the run's result from the pre-loaded metadata plus the fresh mission result/score.
    types::SimulationResult result{};
    result.simulation_config = simulation_config_;
    result.mission_config = mission_config_;
    result.resolution_request_status = resolution_status_;
    result.mission_results = {std::move(mission_result)};
    result.output_map_file = output_map_file_;
    result.output_map_config = output_map_->getMapConfig();
    result.mission_score = score;
    return result;
}

} // namespace drone_mapper
