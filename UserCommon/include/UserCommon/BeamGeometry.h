/**
 * @file BeamGeometry.h
 * @brief The world geometry shared between the simulated sensor and the code that reads it.
 *
 * @note Architectural boundary: this is the first genuine cross-project duplication the design
 *       predicted. `MockLidar` (Simulator) marches rays through the hidden map, `ScanResultToVoxels`
 *       (MissionControl) marches the *same* rays through the output map, and `MockMovement` advances
 *       the drone along its heading - three call sites, two projects, one formula. Keeping three
 *       copies in step is how a scan eventually lands in the map rotated.
 * @note Header-only and therefore build-file-free, which is how `UserCommon/` is specified. Both
 *       projects add this directory to their include path and nothing else changes.
 * @note **Angle convention, defined here and nowhere else:** `0 deg` = +X east, `90 deg` = +Y south.
 *       Altitude is measured from the horizontal plane, positive upward.
 */

#pragma once

#include <Common/Units.h>

#include <mp-units/systems/si/math.h>

#include <cmath>

namespace user_common_323998450_211633813 {

/**
 * @brief Collapse a direction component that is zero in all but floating-point representation.
 * @param component One component of a unit direction vector.
 * @return Exactly zero when the value is negligible, otherwise @p component unchanged.
 *
 * @note An axis-aligned heading should give exactly 0 on the two axes it does not travel along, but
 *       `cos(90 deg)` computed through radians returns about 6e-17 rather than 0. That residue is
 *       physically meaningless - at these distances it is a fraction of an atom - yet it is enough
 *       to matter: a drone whose start position is a multiple of the map resolution travels along
 *       voxel *boundaries*, where the sign of a 1e-16 offset decides which cell `floor` reports. The
 *       symptom is a move through open space being refused for clipping a wall in the neighbouring
 *       column.
 * @note The threshold is far below anything the centimetre-scale geometry can express, so a
 *       genuinely near-axis beam is snapped to the axis it was already indistinguishable from.
 */
[[nodiscard]] inline double snapToAxis(double component) {
    constexpr double kNegligible = 1e-12;
    return std::abs(component) < kNegligible ? 0.0 : component;
}

/**
 * @brief Turn a heading-relative orientation into a world-facing one.
 * @param heading The drone's current orientation.
 * @param relative An orientation expressed relative to that heading.
 * @return The orientation in world terms.
 * @note Lidar hits report their angle **relative** to the scan direction, so whoever converts them
 *       into map coordinates must add the heading back exactly once. Doing it twice rotates the
 *       whole scan; not at all leaves it pinned to whatever direction the drone happened to face.
 */
[[nodiscard]] inline common::Orientation absoluteBeam(const common::Orientation& heading,
                                                      const common::Orientation& relative) {
    return common::Orientation{
        relative.horizontal + heading.horizontal,
        relative.altitude + heading.altitude,
    };
}

/**
 * @brief The world position a given distance along a beam.
 * @param origin Where the beam starts.
 * @param beam_orientation World-facing direction of the beam.
 * @param distance How far along it to sample; may be negative to travel backwards.
 * @return `origin + distance * direction`.
 * @note Unit idiom: strip each quantity to a plain centimetre scalar, scale the dimensionless
 *       direction, then re-attach the per-axis quantity spec. `XLength`, `YLength` and `ZLength` are
 *       distinct specs, so the re-attachment is what stops an axis being silently swapped - and
 *       `force_numerical_value_in` is what temporarily bypasses that check.
 * @note Passing an orientation whose altitude is zero reduces this to horizontal travel along the
 *       heading, which is exactly what a drone's `advance` needs.
 */
[[nodiscard]] inline common::Position3D pointAlongBeam(const common::Position3D& origin,
                                                       const common::Orientation& beam_orientation,
                                                       common::PhysicalLength distance) {
    namespace mp = mp_units;
    namespace si = mp_units::si;

    const auto cos_altitude = si::cos(beam_orientation.altitude);
    const auto dx = cos_altitude * si::cos(beam_orientation.horizontal);
    const auto dy = cos_altitude * si::sin(beam_orientation.horizontal);
    const auto dz = si::sin(beam_orientation.altitude);

    const double distance_cm = distance.force_numerical_value_in(common::cm);
    const double dir_x = snapToAxis(dx.force_numerical_value_in(mp::one));
    const double dir_y = snapToAxis(dy.force_numerical_value_in(mp::one));
    const double dir_z = snapToAxis(dz.force_numerical_value_in(mp::one));

    return common::Position3D{
        origin.x + dir_x * distance_cm * common::x_extent[common::cm],
        origin.y + dir_y * distance_cm * common::y_extent[common::cm],
        origin.z + dir_z * distance_cm * common::z_extent[common::cm],
    };
}

} // namespace user_common_323998450_211633813
