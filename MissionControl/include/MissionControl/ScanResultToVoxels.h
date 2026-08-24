/**
 * @file ScanResultToVoxels.h
 * @brief Conversion of a lidar scan into occupancy written on the output map.
 */

#pragma once

#include <Common/IMutableMap3D.h>
#include <Common/Types.h>

namespace mission_control {

/**
 * @brief Writes what a scan observed into the map being built.
 *
 * @note Architectural boundary: this was course-provided in Assignment 2 and is absent from
 *       Assignment 3's frozen `common/`, so it is ours - and it belongs with whoever drives the
 *       drone, because turning a scan into voxels is part of taking a step rather than part of
 *       simulating a sensor.
 * @note **This is the only path that writes to the output map.** The mapping algorithm holds the map
 *       as `const IMap3D&` and may plan against it but never edit it, so every voxel in a produced
 *       map came through here.
 * @note Only observation states are written: `Occupied`, `Empty`, and `PotentiallyOccupied`. The map
 *       starts `Unmapped`, so a cell still reading `Unmapped` at the end genuinely was never seen.
 */
class ScanResultToVoxels {
public:
    /**
     * @brief Apply one scan to the map.
     * @param output_map The map being built.
     * @param scan_origin Where the scan was taken from, in world coordinates.
     * @param drone_heading The drone's orientation at that moment.
     * @param scan The hits the sensor reported.
     * @param lidar_config The sensor's geometry, needed to interpret the sentinel distances.
     * @note Hit angles arrive **relative to the scan direction**, so the heading is added back here
     *       exactly once. That is the single most likely place for a produced map to come out
     *       rotated.
     * @note Three beam outcomes map to three different claims: a hit proves the path before it is
     *       empty and the endpoint solid; a miss proves the whole measurable range is empty; a zero
     *       distance means something was detected too close to place, so the near segment is marked
     *       only *potentially* occupied.
     */
    static void applyToMap(common::IMutableMap3D& output_map,
                           const common::Position3D& scan_origin,
                           const common::Orientation& drone_heading,
                           const common::types::LidarScanResult& scan,
                           const common::types::LidarConfigData& lidar_config);
};

} // namespace mission_control
