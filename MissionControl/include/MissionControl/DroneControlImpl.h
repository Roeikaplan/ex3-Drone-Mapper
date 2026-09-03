/**
 * @file DroneControlImpl.h
 * @brief One step of the drone: ask the algorithm, validate, move, scan, record.
 */

#pragma once

#include <MissionControl/IDroneControl.h>

#include <Common/IDroneMovement.h>
#include <Common/IGPS.h>
#include <Common/ILidar.h>
#include <Common/IMappingAlgorithm.h>
#include <Common/IMutableMap3D.h>

#include <cstddef>
#include <optional>

namespace mission_control_323998450_211633813 {

/**
 * @brief Turns one algorithm command into validated motion and a recorded scan.
 *
 * @note Architectural boundary: **this lives inside the MissionControl plugin, not the simulator.**
 *       In Assignment 2 the simulator built it and handed it over; here `MissionControlDependencies`
 *       carries raw sensors and the mission control builds its own. How a mission drives its drone -
 *       how strictly it validates, whether it retries, how it sequences movement against scanning -
 *       is mission policy, and another team's plugin may answer all three differently.
 * @note **All movement validation lives here**, because `IDroneMovement` never validates anything.
 *       The actuator will happily fly through a wall; refusing is this class's job.
 * @note What this class can enforce is limited, and knowing the limit matters: it has no hidden map,
 *       so it can only reject a move into a cell *already observed* `Occupied`. Genuine collision
 *       safety comes from the mapping algorithm routing only through observed-`Empty` space.
 */
class DroneControlImpl final : public IDroneControl {
public:
    /**
     * @brief Construct over the sensors and the algorithm this drone will obey.
     * @param drone The vehicle's per-command limits.
     * @param mission The mission's bounds, in the same world frame as the drone's pose.
     * @param lidar Sensor to scan with.
     * @param gps Pose to read.
     * @param movement Actuator to command.
     * @param output_map Map to record scans into and to consult for known obstacles.
     * @param mapping_algorithm The algorithm deciding what to do next.
     * @note Every reference must outlive this object; the run owns them all.
     */
    DroneControlImpl(types::DroneConfigData drone, types::MissionConfigData mission,
                     common::ILidar& lidar, common::IGPS& gps, common::IDroneMovement& movement,
                     common::IMutableMap3D& output_map,
                     common::IMappingAlgorithm& mapping_algorithm);

    /**
     * @brief Take one step.
     * @return `Continue` while the algorithm is still working, `Completed` when it finishes, or
     *         `Error` with a reason when a command was refused or failed.
     * @note Ordering contract, in two parts. **The first step passes `nullptr`** for the latest scan,
     *       because none exists yet - an empty scan would tell the algorithm it looked and saw
     *       nothing. And when a command carries both movement and a scan, **the movement happens
     *       first** and the scan is taken from the updated pose, because the sensor reads the same
     *       GPS and would otherwise sample from a position the drone has already left.
     */
    [[nodiscard]] types::DroneStepResult step() override;

    /**
     * @brief The drone's current state.
     * @return Its pose from the GPS plus the number of steps taken so far.
     */
    [[nodiscard]] types::DroneState state() const override;

    /**
     * @brief The command the algorithm asked for on the most recent step.
     * @return That step's command; default-constructed before the first step.
     * @note Not part of `IDroneControl`, which is frozen - the mission control holds the concrete
     *       type, so a plain accessor is enough. It exists for the verbose trace: knowing *where*
     *       the drone went is much less useful for diagnosing a stalled mission than knowing what it
     *       was asked to do.
     */
    [[nodiscard]] const types::MappingStepCommand& lastCommand() const noexcept {
        return last_command_;
    }

private:
    types::DroneConfigData drone_;
    types::MissionConfigData mission_;
    common::ILidar& lidar_;
    common::IGPS& gps_;
    common::IDroneMovement& movement_;
    common::IMutableMap3D& output_map_;
    common::IMappingAlgorithm& mapping_algorithm_;

    /**
     * @brief The most recent scan, handed to the algorithm on the following step.
     * @note Empty until the first scan actually happens, which is what makes the `nullptr` on step
     *       one fall out naturally rather than needing a separate flag.
     */
    std::optional<types::LidarScanResult> latest_scan_{};

    /**
     * @brief The most recent command, kept for the verbose trace.
     */
    types::MappingStepCommand last_command_{};

    std::size_t step_index_ = 0;
};

} // namespace mission_control_323998450_211633813
