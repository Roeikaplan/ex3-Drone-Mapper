/**
 * @file MockMovement.cpp
 * @brief Pose arithmetic for the three movement primitives.
 * @note Angle convention throughout: `0 deg` = +X east, `90 deg` = +Y south, so a heading decomposes
 *       into `dx = cos(h)`, `dy = sin(h)`. `MockLidar` and any movement prediction inside a plugin
 *       must use the same convention, or a move that validated cleanly lands somewhere else.
 */

#include <Simulator/MockMovement.h>

#include <UserCommon/BeamGeometry.h>

#include <cmath>

namespace simulator {
namespace {

using common::cm;
using common::deg;
using common::z_extent;

} // namespace

/**
 * @brief Construct over the pose this actuator will update.
 * @param gps The run's GPS; must outlive this object.
 */
MockMovement::MockMovement(MockGPS& gps) : gps_(gps) {}

/**
 * @brief Turn the drone in place.
 * @param direction Left or right.
 * @param angle Magnitude of the turn.
 * @return Always success.
 * @note Left is the positive direction, matching the angle convention. Only the horizontal component
 *       changes: the altitude angle describes where a scan points, not how the drone is attitude-
 *       oriented, so rotating must not disturb it.
 */
common::types::MovementResult MockMovement::rotate(common::types::RotationDirection direction,
                                                   common::HorizontalAngle angle) {
    const common::Orientation current = gps_.heading();
    const common::HorizontalAngle signed_angle =
        direction == common::types::RotationDirection::Left ? angle : -angle;

    /**
     * @note Wrapped into [0, 360). Letting the heading accumulate is not merely untidy: a mission of
     *       several thousand steps reaches tens of thousands of degrees, and the sine and cosine of
     *       an angle that large are computed from a correspondingly large radian argument, which
     *       returns roughly 1e-16 where the exact answer is zero. A drone travelling along voxel
     *       boundaries - which it does whenever its start position is a multiple of the resolution -
     *       is then nudged to the wrong side of one, and a move through free space gets refused for
     *       clipping a wall it never approached.
     */
    const double wrapped_deg =
        std::fmod((current.horizontal + signed_angle).force_numerical_value_in(deg), 360.0);
    const double normalised_deg = wrapped_deg < 0.0 ? wrapped_deg + 360.0 : wrapped_deg;

    gps_.setHeading(common::Orientation{normalised_deg * common::horizontal_angle[deg],
                                        current.altitude});
    return common::types::MovementResult{true, {}};
}

/**
 * @brief Move the drone horizontally along its current heading.
 * @param distance Distance to travel.
 * @return Always success.
 * @note Horizontal only. The altitude angle is deliberately ignored because vertical motion belongs
 *       to `elevate`; letting a tilted heading drift the drone upward would make the two primitives
 *       overlap and any movement prediction ambiguous.
 * @note Unit idiom: strip to a plain centimetre scalar, scale the dimensionless direction, then
 *       re-attach the per-axis quantity spec.
 */
common::types::MovementResult MockMovement::advance(common::PhysicalLength distance) {
    const common::Orientation heading = gps_.heading();

    /**
     * @note Altitude is held level rather than taken from the heading: horizontal travel belongs to
     *       this primitive and vertical travel to `elevate`. Reusing the beam helper with a level
     *       orientation is what keeps this identical to the prediction the mission control's drone
     *       controller makes before allowing the move.
     */
    gps_.setPosition(user_common::pointAlongBeam(
        gps_.position(), common::Orientation{heading.horizontal, 0.0 * common::altitude_angle[deg]},
        distance));
    return common::types::MovementResult{true, {}};
}

/**
 * @brief Change the drone's altitude.
 * @param distance Distance to climb; negative descends.
 * @return Always success.
 */
common::types::MovementResult MockMovement::elevate(common::PhysicalLength distance) {
    const common::Position3D position = gps_.position();
    const double distance_cm = distance.force_numerical_value_in(cm);

    gps_.setPosition(common::Position3D{
        position.x,
        position.y,
        position.z + distance_cm * z_extent[cm],
    });
    return common::types::MovementResult{true, {}};
}

} // namespace simulator
