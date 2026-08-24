/**
 * @file DroneControlImpl.cpp
 * @brief Command validation, movement, and scan recording for one step.
 * @note Angle convention: `0 deg` = +X east, `90 deg` = +Y south, defined in
 *       `UserCommon/BeamGeometry.h` and shared with the simulator's actuator.
 */

#include <MissionControl/DroneControlImpl.h>

#include <MissionControl/ScanResultToVoxels.h>

#include <UserCommon/BeamGeometry.h>

#include <cmath>
#include <string>
#include <utility>

namespace mission_control {
namespace {

/**
 * @brief Where a movement command would leave the drone.
 * @param position Pose the movement starts from.
 * @param heading Current orientation; `Advance` travels along its horizontal component.
 * @param command The movement to simulate.
 * @return The resulting position; `Rotate` and `Hover` leave it unchanged.
 * @note **This must agree with the actuator exactly.** They now live in different projects, so
 *       nothing but the shared `pointAlongBeam` helper keeps them in step - which is precisely why
 *       both call it rather than each writing out the trigonometry.
 * @note `Advance` zeroes the altitude component deliberately: horizontal travel belongs to
 *       `advance` and vertical travel to `elevate`, and letting a tilted heading drift the drone
 *       upward would make the prediction ambiguous.
 */
[[nodiscard]] Position3D predictPosition(const Position3D& position, const Orientation& heading,
                                         const types::MovementCommand& command) {
    switch (command.type) {
    case types::MovementCommandType::Advance:
        return user_common::pointAlongBeam(
            position, Orientation{heading.horizontal, 0.0 * altitude_angle[deg]}, command.distance);
    case types::MovementCommandType::Elevate:
        return Position3D{position.x, position.y,
                          position.z + command.distance.force_numerical_value_in(cm) * z_extent[cm]};
    case types::MovementCommandType::Rotate:
    case types::MovementCommandType::Hover:
        return position;
    }
    return position;
}

/**
 * @brief Whether a position lies inside the mission's mapping region.
 * @param bounds The mission's bounds, in world coordinates.
 * @param position World position to test.
 * @return True when the position is within range on every axis.
 * @note Only the endpoints need testing, unlike the obstacle check below: the bounds are an
 *       axis-aligned box and therefore convex, so a straight path between two in-bounds points
 *       cannot leave and re-enter it.
 * @note Compared as plain centimetre scalars because the three axis length types are distinct
 *       quantity specs and cannot be range-tested against one another.
 */
[[nodiscard]] bool withinMissionBounds(const types::MappingBounds& bounds,
                                       const Position3D& position) {
    const double x = position.x.force_numerical_value_in(cm);
    const double y = position.y.force_numerical_value_in(cm);
    const double z = position.z.force_numerical_value_in(cm);

    return x >= bounds.min_x.force_numerical_value_in(cm) &&
           x <= bounds.max_x.force_numerical_value_in(cm) &&
           y >= bounds.min_y.force_numerical_value_in(cm) &&
           y <= bounds.max_y.force_numerical_value_in(cm) &&
           z >= bounds.min_height.force_numerical_value_in(cm) &&
           z <= bounds.max_height.force_numerical_value_in(cm);
}

/**
 * @brief Straight-line distance between two positions, in centimetres.
 * @param from Start position.
 * @param to End position.
 * @return The separation.
 */
[[nodiscard]] double separationCm(const Position3D& from, const Position3D& to) {
    const double dx = (to.x - from.x).force_numerical_value_in(cm);
    const double dy = (to.y - from.y).force_numerical_value_in(cm);
    const double dz = (to.z - from.z).force_numerical_value_in(cm);
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

/**
 * @brief A point a fraction of the way from one position to another.
 * @param from Start position.
 * @param to End position.
 * @param fraction 0 gives @p from, 1 gives @p to.
 * @return The interpolated position.
 * @note Interpolating between the two endpoints rather than re-deriving a direction keeps this
 *       independent of *how* the move was expressed, so it works for an advance and an elevation
 *       without caring which it is looking at.
 */
[[nodiscard]] Position3D interpolate(const Position3D& from, const Position3D& to,
                                     double fraction) {
    const double x = (to.x - from.x).force_numerical_value_in(cm) * fraction;
    const double y = (to.y - from.y).force_numerical_value_in(cm) * fraction;
    const double z = (to.z - from.z).force_numerical_value_in(cm) * fraction;
    return Position3D{from.x + x * x_extent[cm], from.y + y * y_extent[cm],
                      from.z + z * z_extent[cm]};
}

/**
 * @brief Whether the straight path between two positions passes through a known obstacle.
 * @param map The map of what has been observed so far.
 * @param from Where the drone is now.
 * @param to Where the movement would take it.
 * @return True when every sampled point along the path is clear of known-occupied cells.
 *
 * @note **The whole swept path is checked, not just the endpoint.** A single advance can cross many
 *       voxels - 30 cm of travel over a 5 cm grid is six cells, and the larger drone's 50 cm is ten -
 *       so testing only the destination would let a drone pass clean through a known wall and land
 *       in free space beyond it. Assignment 2 checked only the endpoint and relied on the algorithm
 *       never asking; that is not a safe assumption here, because in competitive mode this mission
 *       control runs against *other teams'* algorithms.
 * @note Sampled at a tenth of a voxel edge, the same step the scan converter marches at, so a
 *       diagonal path cannot tunnel through a one-cell wall that a scan did record.
 * @note Only cells already observed `Occupied` can be refused. A wall the drone has never seen is
 *       invisible here, which is why the mapping algorithm must confine itself to observed-`Empty`
 *       space rather than treating this as its safety net.
 */
[[nodiscard]] bool pathIsClear(const common::IMutableMap3D& map, const Position3D& from,
                               const Position3D& to) {
    const double span_cm = separationCm(from, to);
    if (!(span_cm > 0.0)) {
        return true;
    }

    const double step_cm = 0.1 * map.getMapConfig().resolution.force_numerical_value_in(cm);
    if (!(step_cm > 0.0)) {
        return true;
    }

    const auto samples = static_cast<int>(std::ceil(span_cm / step_cm));
    for (int i = 1; i <= samples; ++i) {
        const double fraction = static_cast<double>(i) / static_cast<double>(samples);
        if (map.atVoxel(interpolate(from, to, fraction)) == types::VoxelOccupancy::Occupied) {
            return false;
        }
    }
    return true;
}

/**
 * @brief Magnitude of a length in centimetres.
 * @param length The length to measure.
 * @return Its absolute value.
 */
[[nodiscard]] double absCm(PhysicalLength length) {
    const double value = length.force_numerical_value_in(cm);
    return value < 0.0 ? -value : value;
}

/**
 * @brief Magnitude of an angle in degrees.
 * @param angle The angle to measure.
 * @return Its absolute value.
 */
[[nodiscard]] double absDeg(HorizontalAngle angle) {
    const double value = angle.force_numerical_value_in(deg);
    return value < 0.0 ? -value : value;
}

/**
 * @brief Check a command against the drone's per-command limits.
 * @param drone The vehicle's limits.
 * @param command The requested movement.
 * @return An empty string when the command is within limits, otherwise the reason it is not.
 * @note Checked before the positional rules because a request the drone physically cannot perform is
 *       a different kind of mistake from one that would fly somewhere illegal, and the reported
 *       message should say which.
 */
[[nodiscard]] std::string checkCommandLimits(const types::DroneConfigData& drone,
                                             const types::MovementCommand& command) {
    switch (command.type) {
    case types::MovementCommandType::Rotate:
        if (absDeg(command.angle) > absDeg(drone.max_rotate)) {
            return "rotation exceeds the drone's max_rotate";
        }
        break;
    case types::MovementCommandType::Advance:
        if (absCm(command.distance) > absCm(drone.max_advance)) {
            return "advance exceeds the drone's max_advance";
        }
        break;
    case types::MovementCommandType::Elevate:
        if (absCm(command.distance) > absCm(drone.max_elevate)) {
            return "elevation exceeds the drone's max_elevate";
        }
        break;
    case types::MovementCommandType::Hover:
        break;
    }
    return {};
}

} // namespace

/**
 * @brief Construct over the sensors and the algorithm this drone will obey.
 * @param drone The vehicle's per-command limits.
 * @param mission The mission's bounds.
 * @param lidar Sensor to scan with.
 * @param gps Pose to read.
 * @param movement Actuator to command.
 * @param output_map Map to record scans into.
 * @param mapping_algorithm The algorithm deciding what to do next.
 */
DroneControlImpl::DroneControlImpl(types::DroneConfigData drone, types::MissionConfigData mission,
                                   common::ILidar& lidar, common::IGPS& gps,
                                   common::IDroneMovement& movement,
                                   common::IMutableMap3D& output_map,
                                   common::IMappingAlgorithm& mapping_algorithm)
    : drone_(std::move(drone)),
      mission_(std::move(mission)),
      lidar_(lidar),
      gps_(gps),
      movement_(movement),
      output_map_(output_map),
      mapping_algorithm_(mapping_algorithm) {}

/**
 * @brief Take one step.
 * @return The step's outcome.
 * @note The `nullptr` on the first step falls out of `latest_scan_` being empty rather than being
 *       special-cased, so there is no flag to forget to clear.
 * @note Movement is validated *and executed* before any scan. The sensor reads the same GPS this
 *       class does, so a scan taken first would describe a pose the drone is about to leave.
 */
types::DroneStepResult DroneControlImpl::step() {
    const types::DroneState current = state();

    const types::LidarScanResult* latest = latest_scan_ ? &*latest_scan_ : nullptr;
    const types::MappingStepCommand command = mapping_algorithm_.nextStep(current, latest);

    /**
     * @note Recorded before any validation, so the verbose trace shows what was *asked for* even
     *       when the step is about to be refused - which is exactly the case worth inspecting.
     */
    last_command_ = command;

    if (command.movement) {
        const types::MovementCommand& move = *command.movement;

        const std::string limit_error = checkCommandLimits(drone_, move);
        if (!limit_error.empty()) {
            return {types::DroneStepStatus::Error, limit_error};
        }

        const Position3D target = predictPosition(current.position, current.heading, move);
        if (!withinMissionBounds(mission_.mission_bounds, target)) {
            return {types::DroneStepStatus::Error, "movement leaves the mission boundaries"};
        }
        if (!pathIsClear(output_map_, current.position, target)) {
            return {types::DroneStepStatus::Error, "movement crosses a known-occupied voxel"};
        }

        types::MovementResult result{};
        switch (move.type) {
        case types::MovementCommandType::Rotate:
            result = movement_.rotate(move.rotation, move.angle);
            break;
        case types::MovementCommandType::Advance:
            result = movement_.advance(move.distance);
            break;
        case types::MovementCommandType::Elevate:
            result = movement_.elevate(move.distance);
            break;
        case types::MovementCommandType::Hover:
            break;
        }
        if (!result) {
            return {types::DroneStepStatus::Error, result.message};
        }
    }

    if (command.scan_orientation) {
        /**
         * @note The pose is re-read from the sensor here rather than reused from `current`, because
         *       any movement above has already changed it. This is the same reason the ordering
         *       matters at all.
         */
        types::LidarScanResult scan = lidar_.scan(*command.scan_orientation);
        ScanResultToVoxels::applyToMap(output_map_, gps_.position(), gps_.heading(), scan,
                                       lidar_.config());
        latest_scan_ = std::move(scan);
    }

    ++step_index_;

    switch (command.status) {
    case types::AlgorithmStatus::Working:
        return {types::DroneStepStatus::Continue, {}};
    case types::AlgorithmStatus::Finished:
    case types::AlgorithmStatus::FinishedWithUnmappableVoxels:
        return {types::DroneStepStatus::Completed, {}};
    }
    return {types::DroneStepStatus::Continue, {}};
}

/**
 * @brief The drone's current state.
 * @return Its pose from the GPS plus the number of steps taken so far.
 * @note Read fresh from the sensor every time rather than cached, so this class cannot drift out of
 *       step with the actuator.
 */
types::DroneState DroneControlImpl::state() const {
    return types::DroneState{gps_.position(), gps_.heading(), step_index_};
}

} // namespace mission_control
