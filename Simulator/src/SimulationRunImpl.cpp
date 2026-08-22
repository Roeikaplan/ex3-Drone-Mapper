/**
 * @file SimulationRunImpl.cpp
 * @brief Driving one mission and turning its outcome into a scored result.
 */

#include <Simulator/SimulationRunImpl.h>

#include <Simulator/MapsComparison.h>

#include <stdexcept>
#include <utility>

namespace simulator {

/**
 * @brief Take ownership of a fully wired run.
 * @param hidden_map Ground truth.
 * @param output_map The map the mission fills in.
 * @param gps Simulated pose.
 * @param movement Simulated actuator.
 * @param lidar Simulated sensor.
 * @param mapping_algorithm The algorithm plugin's instance.
 * @param mission_control The mission control plugin's instance.
 * @param simulation_config Configuration echoed into the result.
 * @param mission_config Configuration echoed into the result.
 * @param resolution_status How the output-resolution request was handled.
 * @param output_map_file Where the mission control was told to save its map.
 * @throws std::invalid_argument when any dependency is null.
 * @note The null check is a wiring assertion, not input validation. A null here means the factory is
 *       broken, and failing loudly at construction beats a segfault mid-mission inside plugin code.
 */
SimulationRunImpl::SimulationRunImpl(std::unique_ptr<const common::IMap3D> hidden_map,
                                     std::unique_ptr<common::IMutableMap3D> output_map,
                                     std::unique_ptr<common::IGPS> gps,
                                     std::unique_ptr<common::IDroneMovement> movement,
                                     std::unique_ptr<common::ILidar> lidar,
                                     std::unique_ptr<common::IMappingAlgorithm> mapping_algorithm,
                                     std::unique_ptr<common::IMissionControl> mission_control,
                                     types::SimulationConfigData simulation_config,
                                     common::types::MissionConfigData mission_config,
                                     types::ResolutionRequestStatus resolution_status,
                                     std::filesystem::path output_map_file)
    : hidden_map_(std::move(hidden_map)),
      output_map_(std::move(output_map)),
      gps_(std::move(gps)),
      movement_(std::move(movement)),
      lidar_(std::move(lidar)),
      mapping_algorithm_(std::move(mapping_algorithm)),
      mission_control_(std::move(mission_control)),
      simulation_config_(std::move(simulation_config)),
      mission_config_(std::move(mission_config)),
      resolution_status_(resolution_status),
      output_map_file_(std::move(output_map_file)) {
    if (!hidden_map_ || !output_map_ || !gps_ || !movement_ || !lidar_ || !mapping_algorithm_ ||
        !mission_control_) {
        throw std::invalid_argument("SimulationRunImpl requires every dependency to be non-null.");
    }
}

/**
 * @brief Drive the mission and score what it produced.
 * @return The assembled result for this run.
 * @note The mission control saves the output map itself - it was handed the path for exactly that
 *       reason - so by the time `runMission` returns, the map on disk and the map in memory agree.
 *       Scoring reads the in-memory one.
 * @note Scoring compares against the hidden map, which never crossed the plugin boundary. That is
 *       what makes the score meaningful: a plugin has no way to have seen the answer.
 */
types::SimulationResult SimulationRunImpl::run() {
    common::types::MissionRunResult mission_result = mission_control_->runMission();

    /**
     * @note An errored mission takes the sentinel and is not compared at all. A partial map from a
     *       run that failed would still produce a plausible-looking number, and the report needs
     *       failure to be distinguishable from a poor score.
     */
    double score = -1.0;
    if (mission_result.status != common::types::MissionRunStatus::Error) {
        score = MapsComparison::compare(*hidden_map_, *output_map_);
    }

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

} // namespace simulator
