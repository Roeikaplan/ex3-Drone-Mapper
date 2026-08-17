#pragma once

#include <drone_mapper/IDroneControl.h>
#include <drone_mapper/IDroneMovement.h>
#include <drone_mapper/IGPS.h>
#include <drone_mapper/ILidar.h>
#include <drone_mapper/IMappingAlgorithm.h>
#include <drone_mapper/IMutableMap3D.h>

#include <optional>

namespace drone_mapper {

/**
 * @brief Concrete drone controller that executes one mapping step at a time.
 *
 * Sits between mission control and the (mock or real) hardware. Each step it asks the
 * injected `IMappingAlgorithm` for a `MappingStepCommand`, then validates and performs the
 * requested movement, runs the LiDAR scan, and writes the scan into the output map via
 * `ScanResultToVoxels`. When a command carries both a movement and a scan, the movement is
 * validated and executed first so the scan is taken from the updated pose.
 *
 * @note Architectural boundary: `DroneControl` owns **all** movement validation
 *       (per-command limits, mission boundaries, known-occupied rejection) because
 *       `MockMovement` intentionally never validates. It cannot detect collisions with
 *       *unmapped* solid voxels — it has no hidden map — so ground-truth collision safety is
 *       the algorithm's responsibility (the planner only traverses scanned-`Empty` cells).
 */
class DroneControlImpl final : public IDroneControl {
public:
    /**
     * @brief Construct with configs and non-owning references to run-owned dependencies.
     * @param drone Drone capabilities (radius, per-command movement limits).
     * @param mission Mission behaviour and boundaries used for movement validation.
     * @param lidar Sensor used for scans; also supplies `LidarConfigData` via `config()`.
     * @param gps Positioning sensor; read for the current pose each step.
     * @param movement Motion actuator that mutates the shared pose.
     * @param output_map Writable map the converted scan voxels are written into.
     * @param mapping_algorithm Strategy queried once per step for the next command.
     */
    DroneControlImpl(types::DroneConfigData drone,
                     types::MissionConfigData mission,
                     ILidar& lidar,
                     IGPS& gps,
                     IDroneMovement& movement,
                     IMutableMap3D& output_map,
                     IMappingAlgorithm& mapping_algorithm);

    /**
     * @brief Perform a single mapping step: plan -> (validate+move) -> (scan+apply).
     * @return `Continue` while the algorithm keeps working, `Completed` once it reports it is
     *         finished, or `Error` (with a message) if movement validation or execution fails.
     *         Never throws.
     */
    [[nodiscard]] types::DroneStepResult step() override;
    /**
     * @brief Current drone state assembled from the live GPS pose and the step counter.
     */
    [[nodiscard]] types::DroneState state() const override;

private:
    types::DroneConfigData drone_;
    types::MissionConfigData mission_;
    ILidar& lidar_;
    IGPS& gps_;
    IDroneMovement& movement_;
    IMutableMap3D& output_map_;
    IMappingAlgorithm& mapping_algorithm_;
    std::size_t step_index_ = 0;
    /// Last scan result, forwarded to the next `nextStep`. Empty on the first step so the
    /// algorithm is handed `nullptr` when no LiDAR data exists yet.
    std::optional<types::LidarScanResult> latest_scan_{};
};

} // namespace drone_mapper
