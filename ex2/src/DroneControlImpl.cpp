#include <drone_mapper/DroneControlImpl.h>

#include <drone_mapper/ScanResultToVoxels.h>

#include <mp-units/systems/si/math.h>

#include <string>
#include <utility>

namespace drone_mapper {
namespace {

/**
 * @brief Predict where a movement command lands the drone from a given starting pose.
 * @param position Pose the movement starts from.
 * @param heading Current orientation; Advance travels along its horizontal heading.
 * @param command Movement to simulate (Advance / Elevate / Rotate / Hover).
 * @return The resulting position: Advance moves horizontally along the heading, Elevate changes
 *         Z only, Rotate/Hover leave the position unchanged.
 * @note Mirrors the coordinate math in `MockMovement` so validation sees exactly what the
 *       actuator will do. Angle convention: 0 deg = +X east, 90 deg = +Y south.
 */
[[nodiscard]] Position3D predictPosition(const Position3D& position,
                                         const Orientation& heading,
                                         const types::MovementCommand& command) {
    switch (command.type) {
    case types::MovementCommandType::Advance: {
        // Strip units to a plain cm scalar, scale the dimensionless heading direction, then
        // re-attach the per-axis quantity spec (x_extent/y_extent) — the same idiom used across
        // MockMovement/MockLidar so the coordinate math stays consistent.
        const double cos_h = si::cos(heading.horizontal).force_numerical_value_in(mp::one);
        const double sin_h = si::sin(heading.horizontal).force_numerical_value_in(mp::one);
        const double distance_cm = command.distance.force_numerical_value_in(cm);
        return Position3D{
            position.x + cos_h * distance_cm * x_extent[cm],
            position.y + sin_h * distance_cm * y_extent[cm],
            position.z,
        };
    }
    case types::MovementCommandType::Elevate: {
        const double distance_cm = command.distance.force_numerical_value_in(cm);
        return Position3D{position.x, position.y, position.z + distance_cm * z_extent[cm]};
    }
    case types::MovementCommandType::Rotate:
    case types::MovementCommandType::Hover:
        return position;
    }
    return position;
}

/**
 * @brief Test whether a target position lies inside the mission mapping boundaries.
 * @param bounds Mission mapping boundaries.
 * @param pos World position to test.
 * @return True iff pos is within [min, max] on every axis.
 * @note Bounds are compared on plain cm scalars (force_ bypasses the strict X/Y/Z quantity-spec
 *       checks) so the distinct axis length types can be range-tested uniformly.
 */
[[nodiscard]] bool withinMissionBounds(const types::MappingBounds& bounds, const Position3D& pos) {
    const double x = pos.x.force_numerical_value_in(cm);
    const double y = pos.y.force_numerical_value_in(cm);
    const double z = pos.z.force_numerical_value_in(cm);
    return x >= bounds.min_x.force_numerical_value_in(cm)
        && x <= bounds.max_x.force_numerical_value_in(cm)
        && y >= bounds.min_y.force_numerical_value_in(cm)
        && y <= bounds.max_y.force_numerical_value_in(cm)
        && z >= bounds.min_height.force_numerical_value_in(cm)
        && z <= bounds.max_height.force_numerical_value_in(cm);
}

/**
 * @brief Absolute value of a length as a plain cm scalar.
 * @param length Length quantity.
 * @return |length| in cm.
 */
[[nodiscard]] double absCm(PhysicalLength length) {
    const double value = length.force_numerical_value_in(cm);
    return value < 0.0 ? -value : value;
}

/**
 * @brief Absolute value of a horizontal angle as a plain degree scalar.
 * @param angle Angle quantity.
 * @return |angle| in degrees.
 */
[[nodiscard]] double absDeg(HorizontalAngle angle) {
    const double value = angle.force_numerical_value_in(deg);
    return value < 0.0 ? -value : value;
}

} // namespace

DroneControlImpl::DroneControlImpl(types::DroneConfigData drone,
                                   types::MissionConfigData mission,
                                   ILidar& lidar,
                                   IGPS& gps,
                                   IDroneMovement& movement,
                                   IMutableMap3D& output_map,
                                   IMappingAlgorithm& mapping_algorithm)
    : drone_(std::move(drone)),
      mission_(std::move(mission)),
      lidar_(lidar),
      gps_(gps),
      movement_(movement),
      output_map_(output_map),
      mapping_algorithm_(mapping_algorithm) {}

types::DroneStepResult DroneControlImpl::step() {
    const types::DroneState current = state();

    // First step has no prior scan: the algorithm must be handed a null pointer so it can boot
    // its exploration (it scans in place before it can plan any movement).
    const types::LidarScanResult* latest = latest_scan_ ? &*latest_scan_ : nullptr;
    const types::MappingStepCommand command = mapping_algorithm_.nextStep(current, latest);

    // Movement is validated and executed before the scan, so the scan is taken from the updated
    // pose (the ordering contract when a command carries both).
    if (command.movement) {
        const types::MovementCommand& move = *command.movement;

        // Per-command limits: reject anything the drone physically cannot do in one step.
        switch (move.type) {
        case types::MovementCommandType::Rotate:
            if (absDeg(move.angle) > absDeg(drone_.max_rotate)) {
                return {types::DroneStepStatus::Error, "Rotation exceeds drone max_rotate."};
            }
            break;
        case types::MovementCommandType::Advance:
            if (absCm(move.distance) > absCm(drone_.max_advance)) {
                return {types::DroneStepStatus::Error, "Advance exceeds drone max_advance."};
            }
            break;
        case types::MovementCommandType::Elevate:
            if (absCm(move.distance) > absCm(drone_.max_elevate)) {
                return {types::DroneStepStatus::Error, "Elevate exceeds drone max_elevate."};
            }
            break;
        case types::MovementCommandType::Hover:
            break;
        }

        // Positional legality: the target must stay inside the mission bounds and must not be a
        // voxel already *known* to be solid. Collisions with still-unmapped solids cannot be seen
        // here (no hidden map); the algorithm guarantees it only moves through scanned-Empty space.
        const Position3D target = predictPosition(current.position, current.heading, move);
        if (!withinMissionBounds(mission_.mission_bounds, target)) {
            return {types::DroneStepStatus::Error, "Movement leaves mission boundaries."};
        }
        if (output_map_.atVoxel(target) == types::VoxelOccupancy::Occupied) {
            return {types::DroneStepStatus::Error, "Movement enters a known-occupied voxel."};
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
        // Read the pose *after* any movement: the scan origin/heading must match the state the
        // LiDAR itself samples from (MockLidar reads the same GPS internally).
        types::LidarScanResult scan = lidar_.scan(*command.scan_orientation);
        ScanResultToVoxels::applyToMap(output_map_,
                                       gps_.position(),
                                       gps_.heading(),
                                       scan,
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

types::DroneState DroneControlImpl::state() const {
    return types::DroneState{gps_.position(), gps_.heading(), step_index_};
}

} // namespace drone_mapper
