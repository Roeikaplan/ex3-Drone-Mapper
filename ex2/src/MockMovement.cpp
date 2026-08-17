#include <drone_mapper/MockMovement.h>

#include <mp-units/systems/si/math.h>

namespace drone_mapper {

MockMovement::MockMovement(MockGPS& gps) : gps_(gps) {}

types::MovementResult MockMovement::rotate(types::RotationDirection direction, HorizontalAngle angle) {
    const Orientation current = gps_.heading();
    const HorizontalAngle signed_angle =
        (direction == types::RotationDirection::Left) ? angle : -angle;
    gps_.setHeading(Orientation{current.horizontal + signed_angle, current.altitude});
    return types::MovementResult{true, {}};
}

types::MovementResult MockMovement::advance(PhysicalLength distance) {
    const Position3D position = gps_.position();
    const Orientation heading = gps_.heading();

    // Advance is horizontal-only; the altitude angle is deliberately ignored here
    // because vertical motion is owned by elevate(). Direction uses the same angle
    // convention as MockLidar (0 deg = +X east, 90 deg = +Y south), so the heading
    // decomposes into dx = cos(h), dy = sin(h).
    const double cos_h = si::cos(heading.horizontal).force_numerical_value_in(mp::one);
    const double sin_h = si::sin(heading.horizontal).force_numerical_value_in(mp::one);

    // Strip units to a plain cm scalar so we can scale the dimensionless direction,
    // then re-attach the per-axis quantity spec (x_extent/y_extent) — mirrors the
    // idiom in MockLidar/ScanResultToVoxels so the coordinate math stays consistent.
    const double distance_cm = distance.force_numerical_value_in(cm);

    gps_.setPosition(Position3D{
        position.x + cos_h * distance_cm * x_extent[cm],
        position.y + sin_h * distance_cm * y_extent[cm],
        position.z,
    });
    return types::MovementResult{true, {}};
}

types::MovementResult MockMovement::elevate(PhysicalLength distance) {
    const Position3D position = gps_.position();

    // Elevate changes altitude only; distance may be negative to descend.
    const double distance_cm = distance.force_numerical_value_in(cm);

    gps_.setPosition(Position3D{
        position.x,
        position.y,
        position.z + distance_cm * z_extent[cm],
    });
    return types::MovementResult{true, {}};
}

} // namespace drone_mapper
