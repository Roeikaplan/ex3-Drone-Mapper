#include <drone_mapper/ScanResultToVoxels.h>

#include <mp-units/systems/si/math.h>

#include <limits>

namespace drone_mapper {
namespace {

/**
 * @brief Whether a beam distance is exactly zero.
 * @param distance Beam distance.
 * @return True iff distance == 0 cm.
 */
[[nodiscard]] bool isZeroDistance(PhysicalLength distance) {
    return distance == 0.0 * cm;
}

/**
 * @brief Whether a beam distance encodes a miss (no hit).
 * @param distance Beam distance.
 * @return True iff distance is the sentinel max value MockLidar reports for a miss.
 */
[[nodiscard]] bool isMissDistance(PhysicalLength distance) {
    return distance.force_numerical_value_in(cm) == std::numeric_limits<double>::max();
}

/**
 * @brief Convert a heading-relative beam orientation into a world-facing one.
 * @param drone_heading Current drone heading.
 * @param relative_beam Beam orientation relative to the scan direction.
 * @return The world-facing beam orientation (relative + heading).
 * @note LidarHit angles are relative to the scan direction; the map update needs a world-facing
 *       beam direction, so the drone heading is added.
 */
[[nodiscard]] Orientation absoluteBeamOrientation(const Orientation& drone_heading,
                                                  const Orientation& relative_beam) {
    return Orientation{
        relative_beam.horizontal + drone_heading.horizontal,
        relative_beam.altitude + drone_heading.altitude,
    };
}

/**
 * @brief World position a given distance along a beam.
 * @param origin Scan origin.
 * @param beam_orientation World-facing beam direction.
 * @param distance Distance along the beam.
 * @return The point `origin + distance * direction`, re-attaching the per-axis quantity specs.
 * @note Could have been an external util shared with MockLidar.
 */
[[nodiscard]] Position3D pointAlongBeam(const Position3D& origin,
                                        const Orientation& beam_orientation,
                                        PhysicalLength distance) {
    const auto cos_altitude = si::cos(beam_orientation.altitude);
    const auto dx = cos_altitude * si::cos(beam_orientation.horizontal);
    const auto dy = cos_altitude * si::sin(beam_orientation.horizontal);
    const auto dz = si::sin(beam_orientation.altitude);

    const double distance_cm = distance.force_numerical_value_in(cm);
    const double dir_x = dx.force_numerical_value_in(mp::one);
    const double dir_y = dy.force_numerical_value_in(mp::one);
    const double dir_z = dz.force_numerical_value_in(mp::one);

    return Position3D{
        origin.x + dir_x * distance_cm * x_extent[cm],
        origin.y + dir_y * distance_cm * y_extent[cm],
        origin.z + dir_z * distance_cm * z_extent[cm],
    };
}

/**
 * @brief Evidence-strength ranking for conflicting writes to the same voxel.
 * @param occupancy Occupancy value to rank.
 * @return Priority Occupied(3) > Empty(2) > PotentiallyOccupied(1) > Unmapped/OutOfBounds(0).
 * @note Occupied is a measured hit, Empty is proven free space, and PotentiallyOccupied is only
 *       uncertainty from a too-close hit.
 */
[[nodiscard]] int occupancyPriority(types::VoxelOccupancy occupancy) {
    switch (occupancy) {
    case types::VoxelOccupancy::Occupied:
        return 3;
    case types::VoxelOccupancy::Empty:
        return 2;
    case types::VoxelOccupancy::PotentiallyOccupied:
        return 1;
    case types::VoxelOccupancy::Unmapped:
    case types::VoxelOccupancy::OutOfBounds:
        return 0;
    }

    return 0;
}

/**
 * @brief Write a voxel only if the new observation outranks what is already stored there.
 * @param output_map Map to update; out-of-bounds positions are skipped.
 * @param position World position to write.
 * @param value Candidate occupancy.
 * @note Uses `occupancyPriority` so stronger evidence is never overwritten by weaker.
 */
void setIfStronger(IMutableMap3D& output_map,
                   const Position3D& position,
                   types::VoxelOccupancy value) {
    if (!output_map.isInBounds(position)) {
        return;
    }

    const types::VoxelOccupancy current_value = output_map.atVoxel(position);
    if (occupancyPriority(value) > occupancyPriority(current_value)) {
        output_map.set(position, value);
    }
}

/**
 * @brief Apply one observation to every sampled point along a beam segment.
 * @param output_map Map to update.
 * @param scan_origin Scan origin.
 * @param beam_orientation World-facing beam direction.
 * @param start_distance Segment start distance along the beam.
 * @param end_distance Segment end distance along the beam.
 * @param step Sampling step between consecutive points.
 * @param value Occupancy to write at each in-bounds sample.
 * @note Stops at the first out-of-bounds sample.
 */
void markBeamSegment(IMutableMap3D& output_map,
                     const Position3D& scan_origin,
                     const Orientation& beam_orientation,
                     PhysicalLength start_distance,
                     PhysicalLength end_distance,
                     PhysicalLength step,
                     types::VoxelOccupancy value) {
    for (PhysicalLength distance = start_distance; distance <= end_distance; distance += step) {
        const Position3D current_point = pointAlongBeam(scan_origin, beam_orientation, distance);
        if (!output_map.isInBounds(current_point)) {
            break;
        }
        setIfStronger(output_map, current_point, value);
    }
}

} // namespace

void ScanResultToVoxels::applyToMap(IMutableMap3D& output_map,
                                    const Position3D& scan_origin,
                                    const Orientation& drone_heading,
                                    const types::LidarScanResult& scan,
                                    const types::LidarConfigData& lidar_config) {
    if (!output_map.isInBounds(scan_origin)) {
        return;
    }

    // Use a sub-voxel step like MockLidar so we do not skip thin voxels along
    // diagonal rays.
    const PhysicalLength step = 0.1 * output_map.getMapConfig().resolution;
    if (step <= 0.0 * cm) {
        return;
    }

    for (const types::LidarHit& hit : scan) {
        const Orientation beam_orientation = absoluteBeamOrientation(drone_heading, hit.angle);

        if (isZeroDistance(hit.distance)) {
            // Distance 0 means the hit happened before z_min. The exact voxel
            // is unknown, so mark the near segment as uncertain.
            markBeamSegment(output_map,
                            scan_origin,
                            beam_orientation,
                            0.0 * cm,
                            lidar_config.z_min,
                            step,
                            types::VoxelOccupancy::PotentiallyOccupied);
            continue;
        }

        if (isMissDistance(hit.distance)) {
            // A miss means the beam found no obstacle through the measurable
            // range, so sampled points along the ray are empty.
            markBeamSegment(output_map,
                            scan_origin,
                            beam_orientation,
                            0.0 * cm,
                            lidar_config.z_max,
                            step,
                            types::VoxelOccupancy::Empty);
            continue;
        }

        if (hit.distance > 0.0 * cm) {
            // A normal hit proves the path before the hit is empty and the hit
            // position itself is occupied.
            markBeamSegment(output_map,
                            scan_origin,
                            beam_orientation,
                            0.0 * cm,
                            hit.distance,
                            step,
                            types::VoxelOccupancy::Empty);
            setIfStronger(output_map,
                          pointAlongBeam(scan_origin, beam_orientation, hit.distance),
                          types::VoxelOccupancy::Occupied);
        }
    }
}

} // namespace drone_mapper
