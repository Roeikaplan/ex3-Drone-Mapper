/**
 * @file SimulationRunImpl.h
 * @brief One simulation run: the objects it owns, the mission it drives, the score it produces.
 */

#pragma once

#include <Simulator/ISimulationRun.h>

#include <Common/IDroneMovement.h>
#include <Common/IGPS.h>
#include <Common/ILidar.h>
#include <Common/IMappingAlgorithm.h>
#include <Common/IMissionControl.h>
#include <Common/IMutableMap3D.h>

#include <filesystem>
#include <memory>

namespace simulator {

/**
 * @brief Owns the complete object graph for one (simulation, mission, drone, lidar) combination.
 *
 * @note Ownership model: this holds everything by `unique_ptr` and every inner component holds only
 *       non-owning references to its siblings. Component lifetime is therefore exactly this run's
 *       lifetime, which is what lets the whole graph be discarded between runs without any
 *       cross-run state surviving.
 * @note **Member declaration order is load-bearing.** Members are destroyed in reverse declaration
 *       order, so `mission_control_` is declared last and dies first - it references the algorithm,
 *       the sensors, and the output map, and destroying those first would leave its destructor
 *       holding dangling references.
 * @note Two members come from plugins (`mapping_algorithm_` and `mission_control_`), so every
 *       instance of this class must be destroyed before any plugin library is unloaded. The
 *       destructor chain is the reason a run is a scoped, transient object rather than something
 *       cached.
 */
class SimulationRunImpl final : public ISimulationRun {
public:
    /**
     * @brief Take ownership of a fully wired run.
     * @param hidden_map Ground truth; read only by the lidar and by scoring.
     * @param output_map The map the mission fills in.
     * @param gps Simulated pose.
     * @param movement Simulated actuator.
     * @param lidar Simulated sensor.
     * @param mapping_algorithm The algorithm plugin's instance.
     * @param mission_control The mission control plugin's instance.
     * @param simulation_config Configuration this run was built from, echoed into the result.
     * @param mission_config Configuration this run was built from, echoed into the result.
     * @param resolution_status How the mission's output-resolution request was handled.
     * @param output_map_file Where the mission control was told to save its map.
     * @throws std::invalid_argument when any dependency is null.
     * @note Everything is moved in rather than constructed here: the factory is the single wiring
     *       point, and duplicating that knowledge would give two places to get the order wrong.
     */
    SimulationRunImpl(std::unique_ptr<const common::IMap3D> hidden_map,
                      std::unique_ptr<common::IMutableMap3D> output_map,
                      std::unique_ptr<common::IGPS> gps,
                      std::unique_ptr<common::IDroneMovement> movement,
                      std::unique_ptr<common::ILidar> lidar,
                      std::unique_ptr<common::IMappingAlgorithm> mapping_algorithm,
                      std::unique_ptr<common::IMissionControl> mission_control,
                      types::SimulationConfigData simulation_config,
                      common::types::MissionConfigData mission_config,
                      types::ResolutionRequestStatus resolution_status,
                      std::filesystem::path output_map_file);

    /**
     * @brief Drive the mission and score what it produced.
     * @return The run's configs, mission outcome, output map path and geometry, and final score.
     * @note A mission that ended in error is **not scored**: it takes the -1 sentinel and comparison
     *       is skipped entirely. Scoring a partial map from a failed run would produce a number that
     *       looks meaningful and is not.
     */
    [[nodiscard]] types::SimulationResult run() override;

private:
    std::unique_ptr<const common::IMap3D> hidden_map_;
    std::unique_ptr<common::IMutableMap3D> output_map_;
    std::unique_ptr<common::IGPS> gps_;
    std::unique_ptr<common::IDroneMovement> movement_;
    std::unique_ptr<common::ILidar> lidar_;
    std::unique_ptr<common::IMappingAlgorithm> mapping_algorithm_;
    std::unique_ptr<common::IMissionControl> mission_control_;

    types::SimulationConfigData simulation_config_;
    common::types::MissionConfigData mission_config_;
    types::ResolutionRequestStatus resolution_status_;
    std::filesystem::path output_map_file_;
};

} // namespace simulator
