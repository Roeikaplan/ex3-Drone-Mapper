/**
 * @file MockLidar.cpp
 * @brief Beam layout and ray marching against the hidden map.
 * @note Angle convention: `0 deg` = +X east, `90 deg` = +Y south, matching `MockMovement`.
 * @note This is the provided Assignment 2 sensor behaviour, carried over deliberately rather than
 *       reinvented: the beam geometry and the two sentinel distances are what every scan-to-voxel
 *       converter is written against, so changing them would silently change every produced map.
 */

#include <Simulator/MockLidar.h>

#include <UserCommon/BeamGeometry.h>

#include <mp-units/systems/si/math.h>

#include <cstddef>
#include <limits>

namespace simulator {
namespace {

namespace si = mp_units::si;

using common::cm;
using common::deg;

/**
 * @brief How many beams sit on one field-of-view circle.
 * @param circle_index Zero-based ring index outward from the centre beam.
 * @return `4^circle_index`, so 1 at the centre.
 * @note The quadrupling is what keeps angular coverage roughly uniform as the rings widen; a
 *       constant count per ring would leave the outer rings sparse enough to miss whole voxels.
 */
[[nodiscard]] std::size_t beamsOnCircle(std::size_t circle_index) {
    std::size_t count = 1;
    for (std::size_t i = 0; i < circle_index; ++i) {
        count *= 4;
    }
    return count;
}

/**
 * @brief Horizontal angular offset of a beam displaced sideways from the scan axis.
 * @param offset Lateral displacement from the axis.
 * @param distance Distance along the axis at which the displacement is measured.
 * @return The offset as a horizontal angle.
 */
[[nodiscard]] common::HorizontalAngle horizontalDelta(common::PhysicalLength offset,
                                                      common::PhysicalLength distance) {
    return common::HorizontalAngle{si::atan2(offset, distance)};
}

/**
 * @brief Vertical angular offset of a beam displaced above or below the scan axis.
 * @param offset Vertical displacement from the axis.
 * @param distance Distance along the axis at which the displacement is measured.
 * @return The offset as an altitude angle.
 */
[[nodiscard]] common::AltitudeAngle altitudeDelta(common::PhysicalLength offset,
                                                  common::PhysicalLength distance) {
    return common::AltitudeAngle{si::atan2(offset, distance)};
}

} // namespace

/**
 * @brief Construct over the world this sensor observes.
 * @param config Beam geometry.
 * @param map The hidden ground-truth map.
 * @param gps The drone's pose.
 */
MockLidar::MockLidar(common::types::LidarConfigData config, const common::IMap3D& map,
                     const common::IGPS& gps)
    : config_(config), map_(map), gps_(gps) {}

/**
 * @brief This sensor's configuration.
 * @return The configured beam geometry.
 */
common::types::LidarConfigData MockLidar::config() const {
    return config_;
}

/**
 * @brief Take a scan.
 * @param scan_orientation Direction to scan, relative to the drone's current heading.
 * @return One hit per beam, with angles relative to @p scan_orientation.
 * @note Each beam's direction is built in two steps: the ring offset is added to the requested scan
 *       orientation to get a *relative* beam, and the drone's heading is added on top only to trace
 *       it. The relative form is what gets reported, so a caller that never learns the heading
 *       cannot accidentally double-apply it.
 * @note Ring displacement is measured at `z_min`, the nearest range the sensor can resolve, which
 *       is what fixes the angular spread of a ring independently of how far a beam eventually
 *       travels.
 */
common::types::LidarScanResult MockLidar::scan(common::Orientation scan_orientation) const {
    common::types::LidarScanResult results;
    if (config_.fov_circles == 0) {
        return results;
    }

    const common::Orientation sensor_heading = gps_.heading();
    const common::Orientation centre_beam_absolute =
        user_common_323998450_211633813::absoluteBeam(sensor_heading, scan_orientation);

    results.push_back(common::types::LidarHit{traceBeam(centre_beam_absolute), scan_orientation});

    for (std::size_t circle = 1; circle < config_.fov_circles; ++circle) {
        const std::size_t beam_count = beamsOnCircle(circle);
        const common::PhysicalLength radius = static_cast<double>(circle) * config_.d;

        for (std::size_t i = 0; i < beam_count; ++i) {
            const auto theta =
                (360.0 * static_cast<double>(i) / static_cast<double>(beam_count)) * deg;
            const common::PhysicalLength horizontal_offset = radius * si::cos(theta);
            const common::PhysicalLength altitude_offset = radius * si::sin(theta);

            const common::Orientation relative_beam{
                scan_orientation.horizontal + horizontalDelta(horizontal_offset, config_.z_min),
                scan_orientation.altitude + altitudeDelta(altitude_offset, config_.z_min),
            };
            const common::Orientation absolute_beam =
                user_common_323998450_211633813::absoluteBeam(sensor_heading, relative_beam);

            results.push_back(common::types::LidarHit{traceBeam(absolute_beam), relative_beam});
        }
    }

    return results;
}

/**
 * @brief March one ray until it hits something or leaves the operational range.
 * @param beam_orientation Absolute direction of the beam.
 * @return The hit distance; `0` for a hit nearer than `z_min`; the `double` maximum on a miss.
 * @note The step is a tenth of a voxel edge. A step near the voxel size would let a diagonal ray
 *       tunnel straight through a one-cell wall, so obstacles would vanish rather than the failure
 *       being visible.
 * @note A hit closer than `z_min` reports `0`, not its true range. The sensor can tell something is
 *       there but not where, and reporting the distance would let a converter mark a definite
 *       `Occupied` cell that the hardware never justified.
 * @note `atVoxel == Occupied` is the stopping test, which is why any positive byte in the hidden map
 *       must read back as `Occupied` - block ids included.
 */
common::PhysicalLength MockLidar::traceBeam(const common::Orientation& beam_orientation) const {
    const common::Position3D origin = gps_.position();
    const common::PhysicalLength step = 0.1 * map_.getMapConfig().resolution;

    for (common::PhysicalLength distance = 0.0 * cm; distance <= config_.z_max; distance += step) {
        const common::Position3D sample =
            user_common_323998450_211633813::pointAlongBeam(origin, beam_orientation, distance);

        if (map_.atVoxel(sample) == common::types::VoxelOccupancy::Occupied) {
            /**
             * @note Written as a branch rather than a conditional expression: `0.0 * cm` yields a
             *       bare centimetre quantity while `distance` carries the `isq::length` spec, and a
             *       conditional has no common type for the two. Returning each separately lets the
             *       implicit conversion apply.
             */
            if (distance < config_.z_min) {
                return 0.0 * cm;
            }
            return distance;
        }
    }

    return std::numeric_limits<double>::max() * cm;
}

} // namespace simulator
