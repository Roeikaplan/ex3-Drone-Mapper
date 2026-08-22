/**
 * @file MockMovement.cpp
 * @brief Pose arithmetic for the three movement primitives.
 * @note Angle convention throughout: `0 deg` = +X east, `90 deg` = +Y south, so a heading decomposes
 *       into `dx = cos(h)`, `dy = sin(h)`. `MockLidar` and any movement prediction inside a plugin
 *       must use the same convention, or a move that validated cleanly lands somewhere else.
 */

#include <Simulator/MockMovement.h>

#include <mp-units/systems/si/math.h>

namespace simulator {
namespace {

namespace mp = mp_units;
namespace si = mp_units::si;

using common::cm;
using common::x_extent;
using common::y_extent;
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
    gps_.setHeading(common::Orientation{current.horizontal + signed_angle, current.altitude});
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
    const common::Position3D position = gps_.position();
    const common::Orientation heading = gps_.heading();

    const double cos_h = si::cos(heading.horizontal).force_numerical_value_in(mp::one);
    const double sin_h = si::sin(heading.horizontal).force_numerical_value_in(mp::one);
    const double distance_cm = distance.force_numerical_value_in(cm);

    gps_.setPosition(common::Position3D{
        position.x + cos_h * distance_cm * x_extent[cm],
        position.y + sin_h * distance_cm * y_extent[cm],
        position.z,
    });
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
