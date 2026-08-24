/**
 * @file ScanResultToVoxels.cpp
 * @brief Marching each beam and recording what it proved.
 * @note Ported from Assignment 2, where this file was course-provided. The ray geometry now comes
 *       from `UserCommon/BeamGeometry.h`, shared with the simulator's lidar - the two march the same
 *       rays through different maps, and a divergence between them would put every scan in the wrong
 *       place.
 */

#include <MissionControl/ScanResultToVoxels.h>

#include <UserCommon/BeamGeometry.h>

#include <limits>

namespace mission_control {

/**
 * @note The subsystem's own `IDroneControl.h` opens `namespace mission_control` with
 *       `using namespace common;`, so every file that includes it refers to `types::` and the unit
 *       aliases unqualified. This file does not include that header but follows the same convention
 *       rather than half-qualifying and reading differently from its neighbours.
 */
using namespace common;

namespace {

/**
 * @brief Whether a beam distance is exactly zero.
 * @param distance Reported beam distance.
 * @return True when the sensor reported zero.
 * @note Zero is a sentinel, not a measurement: it means the sensor detected something nearer than
 *       `z_min` and cannot say where.
 */
[[nodiscard]] bool isZeroDistance(PhysicalLength distance) {
    return distance == 0.0 * cm;
}

/**
 * @brief Whether a beam distance is the miss sentinel.
 * @param distance Reported beam distance.
 * @return True when the sensor reported its maximum value.
 * @note The sensor signals "nothing within range" with the largest representable distance rather
 *       than an absent hit, so the scan has one entry per beam either way.
 */
[[nodiscard]] bool isMissDistance(PhysicalLength distance) {
    return distance.force_numerical_value_in(cm) == std::numeric_limits<double>::max();
}

/**
 * @brief Evidence strength of an occupancy value.
 * @param occupancy The value to rank.
 * @return `Occupied` 3, `Empty` 2, `PotentiallyOccupied` 1, anything else 0.
 * @note `Occupied` is a measured hit, `Empty` is proven free space, and `PotentiallyOccupied` is
 *       only uncertainty. Ranking them is what stops a later grazing beam from erasing an earlier
 *       measurement.
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
 * @brief Write a voxel only when the new observation outranks what is stored.
 * @param output_map The map being built.
 * @param position World position to write.
 * @param value Candidate occupancy.
 * @note A voxel is observed many times from many poses over a mission. Without this ranking a wall
 *       seen head-on and later grazed would be overwritten with `Empty`, and the map would dissolve
 *       as the drone flew - the map getting *worse* the more it scanned.
 * @note Out-of-bounds positions are skipped rather than clamped; the map's own `set` would ignore
 *       them anyway, but checking here avoids the pointless read.
 */
void setIfStronger(IMutableMap3D& output_map, const Position3D& position,
                   types::VoxelOccupancy value) {
    if (!output_map.isInBounds(position)) {
        return;
    }
    if (occupancyPriority(value) > occupancyPriority(output_map.atVoxel(position))) {
        output_map.set(position, value);
    }
}

/**
 * @brief Apply one observation to every sampled point along a segment of a beam.
 * @param output_map The map being built.
 * @param scan_origin Where the beam starts.
 * @param beam_orientation World-facing direction of the beam.
 * @param start_distance Distance along the beam to begin at.
 * @param end_distance Distance along the beam to stop at.
 * @param step Spacing between samples.
 * @param value Occupancy to claim at each in-bounds sample.
 * @note Stops at the first out-of-bounds sample rather than continuing: a beam that has left the
 *       mapped region will not re-enter it, so the remaining samples are wasted work.
 */
void markBeamSegment(IMutableMap3D& output_map, const Position3D& scan_origin,
                     const Orientation& beam_orientation, PhysicalLength start_distance,
                     PhysicalLength end_distance, PhysicalLength step,
                     types::VoxelOccupancy value) {
    for (PhysicalLength distance = start_distance; distance <= end_distance; distance += step) {
        const Position3D point =
            user_common::pointAlongBeam(scan_origin, beam_orientation, distance);
        if (!output_map.isInBounds(point)) {
            break;
        }
        setIfStronger(output_map, point, value);
    }
}

} // namespace

/**
 * @brief Apply one scan to the map.
 * @param output_map The map being built.
 * @param scan_origin Where the scan was taken from.
 * @param drone_heading The drone's orientation at that moment.
 * @param scan The hits the sensor reported.
 * @param lidar_config The sensor's geometry.
 * @note The sampling step is a tenth of a voxel edge, matching the simulated sensor's own marching
 *       step. A coarser step would let a diagonal ray pass straight through a one-cell wall that the
 *       sensor did detect, so the map would disagree with the scan that produced it.
 * @note A scan taken from outside the mapped region is discarded whole: every sample along every
 *       beam would be out of bounds, and the loop below would do nothing but waste time proving it.
 */
void ScanResultToVoxels::applyToMap(IMutableMap3D& output_map, const Position3D& scan_origin,
                                    const Orientation& drone_heading,
                                    const types::LidarScanResult& scan,
                                    const types::LidarConfigData& lidar_config) {
    if (!output_map.isInBounds(scan_origin)) {
        return;
    }

    const PhysicalLength step = 0.1 * output_map.getMapConfig().resolution;
    if (step <= 0.0 * cm) {
        return;
    }

    for (const types::LidarHit& hit : scan) {
        const Orientation beam = user_common::absoluteBeam(drone_heading, hit.angle);

        if (isZeroDistance(hit.distance)) {
            /**
             * @note The hit happened nearer than `z_min`, so its exact voxel is unknowable. Marking
             *       the near segment merely *potentially* occupied is the strongest honest claim -
             *       writing `Occupied` somewhere along it would invent a wall the sensor never
             *       located, and the algorithm would then refuse to fly through real free space.
             */
            markBeamSegment(output_map, scan_origin, beam, 0.0 * cm, lidar_config.z_min, step,
                            types::VoxelOccupancy::PotentiallyOccupied);
            continue;
        }

        if (isMissDistance(hit.distance)) {
            markBeamSegment(output_map, scan_origin, beam, 0.0 * cm, lidar_config.z_max, step,
                            types::VoxelOccupancy::Empty);
            continue;
        }

        if (hit.distance > 0.0 * cm) {
            /**
             * @note A normal hit proves two things at once: everything the beam passed through is
             *       empty, and the endpoint is solid. The endpoint is written last so the ranking
             *       never has to arbitrate between the two claims of the same beam.
             */
            markBeamSegment(output_map, scan_origin, beam, 0.0 * cm, hit.distance, step,
                            types::VoxelOccupancy::Empty);
            setIfStronger(output_map, user_common::pointAlongBeam(scan_origin, beam, hit.distance),
                          types::VoxelOccupancy::Occupied);
        }
    }
}

} // namespace mission_control
