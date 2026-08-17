#pragma once

#include <drone_mapper/IDroneControl.h>
#include <drone_mapper/IDroneMovement.h>
#include <drone_mapper/IGPS.h>
#include <drone_mapper/ILidar.h>
#include <drone_mapper/IMap3D.h>
#include <drone_mapper/IMappingAlgorithm.h>
#include <drone_mapper/IMissionControl.h>
#include <drone_mapper/IMutableMap3D.h>
#include <drone_mapper/ISimulationRun.h>

#include <filesystem>
#include <memory>

namespace drone_mapper {

/**
 * @brief Concrete simulation run: owns one scenario's full object graph and executes it once.
 *
 * Holds every component via `unique_ptr` (the ownership root — inner components keep only non-owning
 * references to siblings, so their lifetimes end with this object). `run()` drives the mission to
 * completion, scores the filled output map against the hidden map, and assembles a
 * `types::SimulationResult`.
 *
 * @note Architectural boundary: `SimulationRun` assembles the result but does not *decide* the
 *       configs or resolution status — the factory decides those and pre-loads them here (see the
 *       trailing constructor metadata params). This keeps `run()` a straight-line
 *       execute-score-report step.
 */
class SimulationRunImpl final : public ISimulationRun {
public:
    /**
     * @brief Take ownership of the wired component graph plus the run metadata for the result.
     * @param hidden_map Ground-truth map (read by scoring; the drone never sees it).
     * @param output_map Writable map the drone fills in; scored and reported.
     * @param gps,movement,lidar,mapping_algorithm,drone_control,mission_control Owned components.
     * @param simulation_config,mission_config Configs echoed back into the result.
     * @param resolution_status The factory's ACCEPTED/IGNORED decision for the requested output
     *        resolution, reported verbatim in the result.
     * @param output_map_file Path the output map was saved to, echoed into the result.
     */
    SimulationRunImpl(std::unique_ptr<const IMap3D> hidden_map,
                      std::unique_ptr<IMutableMap3D> output_map,
                      std::unique_ptr<IGPS> gps,
                      std::unique_ptr<IDroneMovement> movement,
                      std::unique_ptr<ILidar> lidar,
                      std::unique_ptr<IMappingAlgorithm> mapping_algorithm,
                      std::unique_ptr<IDroneControl> drone_control,
                      std::unique_ptr<IMissionControl> mission_control,
                      // Changed: stores run metadata needed to build SimulationResult.
                      types::SimulationConfigData simulation_config,
                      types::MissionConfigData mission_config,
                      // Changed: the factory's resolution decision, reported in SimulationResult.
                      types::ResolutionRequestStatus resolution_status,
                      std::filesystem::path output_map_file);

    /**
     * @brief Run the mission, score the output map, and assemble the run's result.
     * @return A `SimulationResult` with the configs, resolution status, mission result(s), output
     *         map path and config, and the final score (IoU 0-100, or -1 when the mission errored).
     */
    [[nodiscard]] types::SimulationResult run() override;

private:
    std::unique_ptr<const IMap3D> hidden_map_;
    std::unique_ptr<IMutableMap3D> output_map_;
    std::unique_ptr<IGPS> gps_;
    std::unique_ptr<IDroneMovement> movement_;
    std::unique_ptr<ILidar> lidar_;
    std::unique_ptr<IMappingAlgorithm> mapping_algorithm_;
    std::unique_ptr<IDroneControl> drone_control_;
    std::unique_ptr<IMissionControl> mission_control_;
    // Changed: retained so run() can return the configs and output path in SimulationResult.
    types::SimulationConfigData simulation_config_;
    types::MissionConfigData mission_config_;
    // Changed: the factory-decided resolution status, echoed into SimulationResult.
    types::ResolutionRequestStatus resolution_status_;
    std::filesystem::path output_map_file_;
};

} // namespace drone_mapper
