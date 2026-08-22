/**
 * @file MockLidar.h
 * @brief The simulation's ray-marching lidar.
 */

#pragma once

#include <Common/IGPS.h>
#include <Common/ILidar.h>
#include <Common/IMap3D.h>

namespace simulator {

/**
 * @brief Produces lidar returns by marching rays through the hidden ground-truth map.
 *
 * Beams are arranged in concentric circles around the scan axis. Circle 0 is a single central beam;
 * circle *i* carries `4^i` beams evenly spread at radius `i * d`, with `d` measured at `z_min`. So
 * `fov_circles = 5` yields 341 beams.
 *
 * @note Architectural boundary: this and `MapsComparison` are the **only** things that read the
 *       hidden map, and neither ever hands it to a plugin. A MissionControl sees ground truth solely
 *       as beam distances, which is what stops a third-party plugin producing a perfect map without
 *       flying.
 * @note The sensor's imprecision is modelled, not hidden. A hit closer than `z_min` reports distance
 *       `0` rather than its true range, because the hardware can detect it but not place it - and a
 *       scan-to-voxel converter must mark that region uncertain rather than solid.
 * @note A simulation fiction: a real deployment replaces this with a driver behind the same `ILidar`
 *       interface.
 */
class MockLidar final : public common::ILidar {
public:
    /**
     * @brief Construct over the world this sensor observes.
     * @param config Beam geometry: operational range, circle spacing, and circle count.
     * @param map The hidden ground-truth map; must outlive this object.
     * @param gps The drone's pose, read fresh on every scan; must outlive this object.
     * @note The pose is read at scan time rather than captured, so a scan always reflects wherever
     *       the drone has been moved to since - which is what makes the movement-before-scan
     *       ordering meaningful.
     */
    MockLidar(common::types::LidarConfigData config, const common::IMap3D& map,
              const common::IGPS& gps);

    /**
     * @brief Take a scan.
     * @param scan_orientation Direction to scan, **relative to the drone's current heading**.
     * @return One hit per beam.
     * @note The returned angles are relative to the scan direction, not absolute. Whoever converts
     *       them to voxels must add the drone heading back - forgetting to is how a scan ends up
     *       written into the map rotated.
     * @note An empty result when `fov_circles` is 0: a sensor configured with no beams sees nothing,
     *       which is a configuration problem rather than an error to report here.
     */
    [[nodiscard]] common::types::LidarScanResult scan(
        common::Orientation scan_orientation) const override;

    /**
     * @brief This sensor's configuration.
     * @return The configured beam geometry.
     * @note Exposed because a scan-to-voxel converter needs `z_min` and `z_max` to interpret the
     *       sentinel distances this returns.
     */
    [[nodiscard]] common::types::LidarConfigData config() const override;

private:
    /**
     * @brief March one ray until it hits something or leaves the operational range.
     * @param beam_orientation Absolute direction of the beam in world terms.
     * @return The hit distance; `0` for a hit nearer than `z_min`; the `double` maximum on a miss.
     */
    [[nodiscard]] common::PhysicalLength traceBeam(const common::Orientation& beam_orientation) const;

    common::types::LidarConfigData config_;
    const common::IMap3D& map_;
    const common::IGPS& gps_;
};

} // namespace simulator
