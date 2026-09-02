/**
 * @file MapsComparisonTest.cpp
 * @brief Coverage of the occupied-voxel IoU metric and its positional sampling.
 * @note The differing-resolution case is the one that pins down *why* the metric samples by world
 *       position rather than by matching indices; the rest would pass either way.
 */

#include <Simulator/Map3DImpl.h>
#include <Simulator/MapsComparison.h>

#include <gtest/gtest.h>

namespace {

using common::cm;
using common::x_extent;
using common::y_extent;
using common::z_extent;

/**
 * @brief Build a world position from plain centimetre values.
 * @param x X coordinate in centimetres.
 * @param y Y coordinate in centimetres.
 * @param z Z coordinate in centimetres.
 * @return The position with its per-axis quantity specs attached.
 */
[[nodiscard]] common::Position3D pos(double x, double y, double z) {
    return common::Position3D{x * x_extent[cm], y * y_extent[cm], z * z_extent[cm]};
}

/**
 * @brief Build a cubic map geometry anchored at the origin.
 * @param span_cm Extent of each axis in centimetres.
 * @param resolution_cm Voxel edge length in centimetres.
 * @return A `MapConfig` describing a cube of voxels.
 */
[[nodiscard]] common::types::MapConfig cubeConfig(double span_cm, double resolution_cm) {
    common::types::MapConfig config{};
    config.resolution = resolution_cm * cm;
    config.boundaries.max_x = span_cm * x_extent[cm];
    config.boundaries.max_y = span_cm * y_extent[cm];
    config.boundaries.max_height = span_cm * z_extent[cm];
    return config;
}

/**
 * @brief Allocate an empty map over a geometry.
 * @param config Geometry for the new map.
 * @return A writable map with every cell `Unmapped`.
 */
[[nodiscard]] simulator::Map3DImpl emptyMap(const common::types::MapConfig& config) {
    return simulator::Map3DImpl{simulator::Map3DImpl::makeEmptyArray(config), config};
}

/**
 * @brief Two maps with the same occupied cells score 100.
 */
TEST(MapsComparison, IdenticalMapsScorePerfect) {
    const common::types::MapConfig config = cubeConfig(4.0, 1.0);
    simulator::Map3DImpl origin = emptyMap(config);
    simulator::Map3DImpl target = emptyMap(config);

    for (double x = 0.5; x < 4.0; x += 1.0) {
        origin.set(pos(x, 0.5, 0.5), common::types::VoxelOccupancy::Occupied);
        target.set(pos(x, 0.5, 0.5), common::types::VoxelOccupancy::Occupied);
    }

    EXPECT_DOUBLE_EQ(simulator::MapsComparison::compare(origin, target), 100.0);
}

/**
 * @brief A target that found nothing scores 0 against an origin that holds something.
 */
TEST(MapsComparison, AnEmptyTargetScoresZero) {
    const common::types::MapConfig config = cubeConfig(4.0, 1.0);
    simulator::Map3DImpl origin = emptyMap(config);
    const simulator::Map3DImpl target = emptyMap(config);

    origin.set(pos(0.5, 0.5, 0.5), common::types::VoxelOccupancy::Occupied);

    EXPECT_DOUBLE_EQ(simulator::MapsComparison::compare(origin, target), 0.0);
}

/**
 * @brief Two maps with no occupied cells anywhere score 100, not 0.
 * @note The degenerate numerator and denominator are both zero, and the answer has to be agreement
 *       rather than a divide-by-zero or a punitive 0 - the target reproduced the origin exactly.
 */
TEST(MapsComparison, TwoEmptyMapsAreTriviallyIdentical) {
    const common::types::MapConfig config = cubeConfig(4.0, 1.0);
    const simulator::Map3DImpl origin = emptyMap(config);
    const simulator::Map3DImpl target = emptyMap(config);

    EXPECT_DOUBLE_EQ(simulator::MapsComparison::compare(origin, target), 100.0)
        << "no occupied voxels anywhere means the maps agree, not that the target failed";
}

/**
 * @brief A partial overlap scores intersection over union, not a percentage of either map alone.
 */
TEST(MapsComparison, PartialOverlapGivesTheIntersectionOverUnion) {
    const common::types::MapConfig config = cubeConfig(4.0, 1.0);
    simulator::Map3DImpl origin = emptyMap(config);
    simulator::Map3DImpl target = emptyMap(config);

    /**
     * @note Origin holds three cells, target holds three, two are shared. Union is four, so the
     *       score is 2/4.
     */
    origin.set(pos(0.5, 0.5, 0.5), common::types::VoxelOccupancy::Occupied);
    origin.set(pos(1.5, 0.5, 0.5), common::types::VoxelOccupancy::Occupied);
    origin.set(pos(2.5, 0.5, 0.5), common::types::VoxelOccupancy::Occupied);

    target.set(pos(1.5, 0.5, 0.5), common::types::VoxelOccupancy::Occupied);
    target.set(pos(2.5, 0.5, 0.5), common::types::VoxelOccupancy::Occupied);
    target.set(pos(3.5, 0.5, 0.5), common::types::VoxelOccupancy::Occupied);

    EXPECT_DOUBLE_EQ(simulator::MapsComparison::compare(origin, target), 50.0);
}

/**
 * @brief Agreement on empty space earns nothing; only occupied voxels count.
 */
TEST(MapsComparison, EmptyAgreementDoesNotInflateTheScore) {
    /**
     * @note The two maps agree on 63 of 64 cells, disagreeing only on the single occupied one. A
     *       metric that counted empty agreement would score this near 98; occupied-voxel IoU scores
     *       it 0, which is what keeps the metric able to rank algorithms.
     */
    const common::types::MapConfig config = cubeConfig(4.0, 1.0);
    simulator::Map3DImpl origin = emptyMap(config);
    simulator::Map3DImpl target = emptyMap(config);

    origin.set(pos(0.5, 0.5, 0.5), common::types::VoxelOccupancy::Occupied);
    for (double x = 0.5; x < 4.0; x += 1.0) {
        for (double y = 0.5; y < 4.0; y += 1.0) {
            for (double z = 0.5; z < 4.0; z += 1.0) {
                target.set(pos(x, y, z), common::types::VoxelOccupancy::Empty);
            }
        }
    }

    EXPECT_DOUBLE_EQ(simulator::MapsComparison::compare(origin, target), 0.0);
}

/**
 * @brief Maps at different resolutions are still comparable, because sampling is by world position.
 */
TEST(MapsComparison, ATargetAtADifferentResolutionIsStillScored) {
    /**
     * @note This is the property that positional sampling buys. The target's grid is half the
     *       origin's cell size, so no index in one corresponds to an index in the other - only the
     *       world coordinates line up.
     */
    const common::types::MapConfig origin_config = cubeConfig(4.0, 1.0);
    const common::types::MapConfig target_config = cubeConfig(4.0, 0.5);

    simulator::Map3DImpl origin = emptyMap(origin_config);
    simulator::Map3DImpl target = emptyMap(target_config);

    origin.set(pos(1.5, 1.5, 1.5), common::types::VoxelOccupancy::Occupied);
    target.set(pos(1.5, 1.5, 1.5), common::types::VoxelOccupancy::Occupied);

    EXPECT_DOUBLE_EQ(simulator::MapsComparison::compare(origin, target), 100.0);
}

/**
 * @brief A degenerate map geometry scores 0 instead of dividing by a zero cell size.
 * @note Scoring runs after a mission has already completed, so it must not be able to crash the run
 *       it is reporting on - a bad geometry becomes a bad score, not an exception.
 */
TEST(MapsComparison, NonPositiveResolutionScoresZeroRatherThanDividing) {
    common::types::MapConfig broken = cubeConfig(4.0, 1.0);
    const common::types::MapConfig usable = cubeConfig(4.0, 1.0);
    broken.resolution = 0.0 * cm;

    const simulator::Map3DImpl origin{simulator::Map3DImpl::makeEmptyArray(usable), broken};
    const simulator::Map3DImpl target = emptyMap(usable);

    EXPECT_DOUBLE_EQ(simulator::MapsComparison::compare(origin, target), 0.0);
}

} // namespace
