/**
 * @file ScanResultToVoxelsTest.cpp
 * @brief Coverage of what each beam outcome claims and how conflicting claims are resolved.
 * @note The evidence-ranking case is the one with teeth: everything else here would pass just as
 *       happily with a converter that overwrote blindly, and a map that dissolves as the drone flies
 *       is the exact failure that would cause.
 */

#include <MissionControl/ScanResultToVoxels.h>

#include "Fakes.h"

#include <gtest/gtest.h>

#include <limits>

namespace {

using namespace common;
using mission_control_323998450_211633813::ScanResultToVoxels;
using mission_control_323998450_211633813::testing::FakeMap;

/**
 * @brief Build a world position from plain centimetre values.
 * @param x X coordinate in centimetres.
 * @param y Y coordinate in centimetres.
 * @param z Z coordinate in centimetres.
 * @return The position.
 */
[[nodiscard]] Position3D pos(double x, double y, double z) {
    return Position3D{x * x_extent[cm], y * y_extent[cm], z * z_extent[cm]};
}

/**
 * @brief A level orientation at a given bearing.
 * @param degrees Horizontal angle in degrees.
 * @return The orientation.
 */
[[nodiscard]] Orientation facing(double degrees) {
    return Orientation{degrees * horizontal_angle[deg], 0.0 * altitude_angle[deg]};
}

/**
 * @brief A lidar configuration with the given operational range.
 * @param z_min_cm Nearest resolvable distance.
 * @param z_max_cm Furthest detectable distance.
 * @return The configuration.
 */
[[nodiscard]] types::LidarConfigData lidarConfig(double z_min_cm, double z_max_cm) {
    types::LidarConfigData config{};
    config.z_min = z_min_cm * cm;
    config.z_max = z_max_cm * cm;
    config.d = 1.0 * cm;
    config.fov_circles = 1;
    return config;
}

/**
 * @brief A scan holding a single beam pointed straight ahead.
 * @param distance_cm Distance the beam reported.
 * @return The scan.
 */
[[nodiscard]] types::LidarScanResult singleBeam(double distance_cm) {
    return {types::LidarHit{distance_cm * cm, facing(0.0)}};
}

/**
 * @brief A normal hit proves two things at once: the path is clear and the endpoint is solid.
 * @note The third assertion is the load-bearing one. Nothing beyond the hit may be touched, because
 *       the beam stopped there and saw nothing further - claiming otherwise would invent evidence.
 */
TEST(ScanResultToVoxels, AHitMarksThePathEmptyAndTheEndpointOccupied) {
    FakeMap map{100.0, 1.0};

    ScanResultToVoxels::applyToMap(map, pos(10.0, 10.0, 10.0), facing(0.0), singleBeam(20.0),
                                   lidarConfig(5.0, 50.0));

    EXPECT_EQ(map.atVoxel(pos(15.0, 10.0, 10.0)), types::VoxelOccupancy::Empty)
        << "the beam proved this cell was passed through";
    EXPECT_EQ(map.atVoxel(pos(30.0, 10.0, 10.0)), types::VoxelOccupancy::Occupied)
        << "the endpoint is where it stopped";
    EXPECT_EQ(map.atVoxel(pos(40.0, 10.0, 10.0)), types::VoxelOccupancy::Unmapped)
        << "beyond the hit the beam proved nothing";
}

/**
 * @brief A miss clears the beam out to `z_max` and claims nothing solid.
 * @note The sensor signals "nothing in range" with the largest representable distance rather than
 *       omitting the beam, so the sentinel must be recognised *before* the value is used as a
 *       length - marching to `DBL_MAX` would sweep far past the map instead of stopping at `z_max`.
 */
TEST(ScanResultToVoxels, AMissMarksTheWholeRangeEmpty) {
    FakeMap map{100.0, 1.0};
    const types::LidarScanResult scan = {
        types::LidarHit{std::numeric_limits<double>::max() * cm, facing(0.0)}};

    ScanResultToVoxels::applyToMap(map, pos(10.0, 10.0, 10.0), facing(0.0), scan,
                                   lidarConfig(5.0, 30.0));

    EXPECT_EQ(map.atVoxel(pos(20.0, 10.0, 10.0)), types::VoxelOccupancy::Empty);
    EXPECT_EQ(map.atVoxel(pos(39.0, 10.0, 10.0)), types::VoxelOccupancy::Empty)
        << "empty right out to z_max";
    EXPECT_EQ(map.atVoxel(pos(45.0, 10.0, 10.0)), types::VoxelOccupancy::Unmapped)
        << "and nothing past it";
    EXPECT_EQ(map.countOf(types::VoxelOccupancy::Occupied), 0u);
}

/**
 * @brief A hit nearer than `z_min` claims only `PotentiallyOccupied` - never `Empty`, never solid.
 */
TEST(ScanResultToVoxels, AZeroDistanceOnlyClaimsUncertainty) {
    /**
     * @note The sensor detected something nearer than `z_min` but cannot place it. Claiming a
     *       definite `Occupied` cell anywhere along that segment would invent a wall, and the
     *       algorithm would then refuse to fly through free space.
     */
    FakeMap map{100.0, 1.0};

    ScanResultToVoxels::applyToMap(map, pos(10.0, 10.0, 10.0), facing(0.0), singleBeam(0.0),
                                   lidarConfig(8.0, 50.0));

    EXPECT_EQ(map.atVoxel(pos(13.0, 10.0, 10.0)), types::VoxelOccupancy::PotentiallyOccupied);
    EXPECT_EQ(map.countOf(types::VoxelOccupancy::Occupied), 0u)
        << "nothing may be claimed as definitely solid";
    EXPECT_EQ(map.countOf(types::VoxelOccupancy::Empty), 0u)
        << "nor may anything be claimed as definitely clear";
    EXPECT_EQ(map.atVoxel(pos(25.0, 10.0, 10.0)), types::VoxelOccupancy::Unmapped)
        << "beyond z_min the beam said nothing at all";
}

/**
 * @brief The evidence ranking stops a later `Empty` sweep from erasing a measured wall.
 * @note The case with teeth, as the file header says: every other test here would pass just as
 *       happily against a converter that overwrote blindly.
 */
TEST(ScanResultToVoxels, AnEmptyClaimNeverErasesAMeasuredHit) {
    /**
     * @note The failure this guards against is subtle and progressive: without the ranking, a wall
     *       seen head-on and later grazed by a passing beam is overwritten with `Empty`, so the map
     *       gets *worse* the more the drone scans.
     */
    FakeMap map{100.0, 1.0};
    const types::LidarConfigData config = lidarConfig(5.0, 50.0);

    ScanResultToVoxels::applyToMap(map, pos(10.0, 10.0, 10.0), facing(0.0), singleBeam(20.0),
                                   config);
    ASSERT_EQ(map.atVoxel(pos(30.0, 10.0, 10.0)), types::VoxelOccupancy::Occupied);

    const types::LidarScanResult miss = {
        types::LidarHit{std::numeric_limits<double>::max() * cm, facing(0.0)}};
    ScanResultToVoxels::applyToMap(map, pos(10.0, 10.0, 10.0), facing(0.0), miss, config);

    EXPECT_EQ(map.atVoxel(pos(30.0, 10.0, 10.0)), types::VoxelOccupancy::Occupied)
        << "a later sweep of empty space must not demolish a wall already measured";
}

/**
 * @brief The ranking is an ordering, not a freeze: a real measurement replaces earlier uncertainty.
 * @note The other direction of the previous test. `PotentiallyOccupied` must yield to `Occupied`, or
 *       one unplaceable near-hit would permanently block a cell the sensor later resolved properly.
 */
TEST(ScanResultToVoxels, AMeasuredHitUpgradesAnUncertainCell) {
    FakeMap map{100.0, 1.0};
    const types::LidarConfigData config = lidarConfig(25.0, 50.0);

    ScanResultToVoxels::applyToMap(map, pos(10.0, 10.0, 10.0), facing(0.0), singleBeam(0.0),
                                   config);
    ASSERT_EQ(map.atVoxel(pos(30.0, 10.0, 10.0)), types::VoxelOccupancy::PotentiallyOccupied);

    ScanResultToVoxels::applyToMap(map, pos(10.0, 10.0, 10.0), facing(0.0), singleBeam(20.0),
                                   config);

    EXPECT_EQ(map.atVoxel(pos(30.0, 10.0, 10.0)), types::VoxelOccupancy::Occupied)
        << "a real measurement outranks earlier uncertainty";
}

/**
 * @brief The drone's heading is applied to hit angles, placing the scan in world coordinates.
 * @note Two identical scans taken from one point at different headings must land on different cells.
 *       Checked against the `0 deg = +X east, 90 deg = +Y south` convention the whole project shares.
 */
TEST(ScanResultToVoxels, TheDroneHeadingRotatesWhereTheScanLands) {
    /**
     * @note Hit angles are relative to the scan direction, so the heading must be added back exactly
     *       once. Adding it twice - or not at all - writes the whole scan into the wrong part of the
     *       map, which is the single easiest way to produce a plausible but useless result.
     */
    FakeMap east{100.0, 1.0};
    FakeMap south{100.0, 1.0};
    const types::LidarConfigData config = lidarConfig(5.0, 50.0);

    ScanResultToVoxels::applyToMap(east, pos(10.0, 10.0, 10.0), facing(0.0), singleBeam(20.0),
                                   config);
    ScanResultToVoxels::applyToMap(south, pos(10.0, 10.0, 10.0), facing(90.0), singleBeam(20.0),
                                   config);

    EXPECT_EQ(east.atVoxel(pos(30.0, 10.0, 10.0)), types::VoxelOccupancy::Occupied);
    EXPECT_EQ(south.atVoxel(pos(10.0, 30.0, 10.0)), types::VoxelOccupancy::Occupied)
        << "90 degrees points at +Y south";
}

/**
 * @brief A beam leaving the map records the part inside it and drops the rest.
 * @note Partial evidence is still evidence. Discarding the whole beam because its endpoint fell
 *       outside would throw away clear space the drone genuinely observed on the way there.
 */
TEST(ScanResultToVoxels, SamplesOutsideTheMapAreDropped) {
    FakeMap map{20.0, 1.0};

    ScanResultToVoxels::applyToMap(map, pos(15.0, 10.0, 10.0), facing(0.0), singleBeam(30.0),
                                   lidarConfig(5.0, 50.0));

    EXPECT_EQ(map.countOf(types::VoxelOccupancy::Occupied), 0u)
        << "the endpoint lies outside the map and must not be recorded";
    EXPECT_GT(map.countOf(types::VoxelOccupancy::Empty), 0u)
        << "the part of the path inside the map is still valid evidence";
}

/**
 * @brief A scan whose origin lies outside the mapped region is discarded whole.
 * @note A short-circuit at the top of `applyToMap` rather than a separate rule - a drone outside the
 *       region it was asked to map has nothing to contribute, and checking once beats proving it
 *       once per sample of every beam.
 */
TEST(ScanResultToVoxels, AScanFromOutsideTheMapIsDiscarded) {
    FakeMap map{20.0, 1.0};

    ScanResultToVoxels::applyToMap(map, pos(50.0, 50.0, 50.0), facing(180.0), singleBeam(10.0),
                                   lidarConfig(5.0, 50.0));

    EXPECT_EQ(map.countOf(types::VoxelOccupancy::Empty), 0u);
    EXPECT_EQ(map.countOf(types::VoxelOccupancy::Occupied), 0u);
}

} // namespace
