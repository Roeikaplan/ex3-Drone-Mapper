#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/MapsComparison.h>

#include <gtest/gtest.h>

#include <cstddef>
#include <vector>

namespace {

using namespace drone_mapper;

// A voxel grid index; the tests build maps by marking a handful of these Occupied.
struct Voxel {
    std::size_t i;
    std::size_t j;
    std::size_t k;
};

/**
 * @brief Shared 4x4x4 test geometry: 1 cm voxels anchored at the world origin.
 * @return A `MapConfig` where grid index `(i, j, k)` maps to world position `(i, j, k)` cm.
 * @note Small and offset-free so voxel centres and indices line up 1:1, keeping the expected
 *       IoU values trivial to reason about.
 */
[[nodiscard]] types::MapConfig testConfig() {
    const types::MappingBounds bounds{
        XLength{}, 4.0 * x_extent[cm],
        YLength{}, 4.0 * y_extent[cm],
        ZLength{}, 4.0 * z_extent[cm],
    };
    return types::MapConfig{bounds, Position3D{}, 1.0 * cm};
}

/**
 * @brief World position of a voxel's centre on the shared test geometry.
 * @param v Grid index of the voxel.
 * @return A `Position3D` at `(i + 0.5, j + 0.5, k + 0.5)` cm so `set`/`atVoxel` resolve to
 *         cell `(i, j, k)` (their `floor` lands inside the cell).
 */
[[nodiscard]] Position3D center(const Voxel& v) {
    return Position3D{
        (static_cast<double>(v.i) + 0.5) * x_extent[cm],
        (static_cast<double>(v.j) + 0.5) * y_extent[cm],
        (static_cast<double>(v.k) + 0.5) * z_extent[cm],
    };
}

/**
 * @brief Build a map on the shared test geometry with the given voxels marked Occupied.
 * @param occupied Voxels to set to `Occupied`; all others stay `Unmapped`.
 * @return A ready-to-compare `Map3DImpl`.
 */
[[nodiscard]] Map3DImpl mapWith(const std::vector<Voxel>& occupied) {
    const types::MapConfig cfg = testConfig();
    Map3DImpl map{Map3DImpl::makeEmptyArray(cfg), cfg};
    for (const Voxel& v : occupied) {
        map.set(center(v), types::VoxelOccupancy::Occupied);
    }
    return map;
}

} // namespace

TEST(MapsComparison, IdenticalMapsScorePerfect) {
    Map3DImpl origin = mapWith({{0, 0, 0}, {1, 1, 1}, {2, 2, 2}});
    Map3DImpl target = mapWith({{0, 0, 0}, {1, 1, 1}, {2, 2, 2}});

    const std::vector<double> scores = MapsComparison::compare(origin, {&target});
    ASSERT_EQ(scores.size(), 1u);
    EXPECT_DOUBLE_EQ(scores[0], 100.0);
}

TEST(MapsComparison, DisjointOccupiedScoresZero) {
    Map3DImpl origin = mapWith({{0, 0, 0}});
    Map3DImpl target = mapWith({{1, 1, 1}});

    const std::vector<double> scores = MapsComparison::compare(origin, {&target});
    ASSERT_EQ(scores.size(), 1u);
    EXPECT_DOUBLE_EQ(scores[0], 0.0);
}

TEST(MapsComparison, PartialOverlapScoresIoU) {
    // intersection = {(1,1,1),(2,2,2)} = 2 ; union = 4 -> 50%.
    Map3DImpl origin = mapWith({{0, 0, 0}, {1, 1, 1}, {2, 2, 2}});
    Map3DImpl target = mapWith({{1, 1, 1}, {2, 2, 2}, {3, 3, 3}});

    const std::vector<double> scores = MapsComparison::compare(origin, {&target});
    ASSERT_EQ(scores.size(), 1u);
    EXPECT_NEAR(scores[0], 50.0, 1e-9);
}

TEST(MapsComparison, TwoEmptyMapsScorePerfect) {
    Map3DImpl origin = mapWith({});
    Map3DImpl target = mapWith({});

    const std::vector<double> scores = MapsComparison::compare(origin, {&target});
    ASSERT_EQ(scores.size(), 1u);
    EXPECT_DOUBLE_EQ(scores[0], 100.0);
}

TEST(MapsComparison, PreservesTargetOrder) {
    Map3DImpl origin = mapWith({{0, 0, 0}, {1, 1, 1}});
    Map3DImpl identical = mapWith({{0, 0, 0}, {1, 1, 1}});
    Map3DImpl disjoint = mapWith({{2, 2, 2}, {3, 3, 3}});

    const std::vector<double> scores =
        MapsComparison::compare(origin, {&identical, &disjoint});
    ASSERT_EQ(scores.size(), 2u);
    EXPECT_DOUBLE_EQ(scores[0], 100.0);
    EXPECT_DOUBLE_EQ(scores[1], 0.0);
}

/**
 * @brief No targets in, no scores out.
 */
TEST(MapsComparison, EmptyTargetsYieldEmptyScores) {
    Map3DImpl origin = mapWith({{0, 0, 0}});

    const std::vector<double> scores = MapsComparison::compare(origin, {});

    EXPECT_TRUE(scores.empty());
}

/**
 * @brief A null target is scored as if fully unmapped: the origin's occupancy alone sets the union.
 *
 * With two occupied origin voxels and no target contribution, intersection/union = 0/2 -> 0.
 */
TEST(MapsComparison, NullTargetScoresAgainstOriginOnly) {
    Map3DImpl origin = mapWith({{0, 0, 0}, {1, 1, 1}});

    const std::vector<double> scores = MapsComparison::compare(origin, {nullptr});

    ASSERT_EQ(scores.size(), 1u);
    EXPECT_DOUBLE_EQ(scores[0], 0.0);
}

/**
 * @brief A zero-resolution origin has no grid to walk: every target takes the floor score.
 */
TEST(MapsComparison, ZeroResolutionOriginScoresFloor) {
    const types::MapConfig degenerate{types::MappingBounds{}, Position3D{}, 0.0 * cm};
    Map3DImpl origin{Map3DImpl::makeEmptyArray(testConfig()), degenerate};
    Map3DImpl target = mapWith({{0, 0, 0}});

    const std::vector<double> scores = MapsComparison::compare(origin, {&target});

    ASSERT_EQ(scores.size(), 1u);
    EXPECT_DOUBLE_EQ(scores[0], 0.0);
}

/**
 * @brief Extra occupied voxels in the target grow the union and are penalized (IoU, not recall).
 *
 * intersection = 2 shared cells; union = 3 (the target's extra cell counts) -> 100 * 2/3.
 */
TEST(MapsComparison, SupersetTargetPenalized) {
    Map3DImpl origin = mapWith({{0, 0, 0}, {1, 1, 1}});
    Map3DImpl target = mapWith({{0, 0, 0}, {1, 1, 1}, {2, 2, 2}});

    const std::vector<double> scores = MapsComparison::compare(origin, {&target});

    ASSERT_EQ(scores.size(), 1u);
    EXPECT_NEAR(scores[0], 100.0 * 2.0 / 3.0, 1e-9);
}

/**
 * @brief Grids anchored at a nonzero offset compare in world space: identical maps still score 100.
 *
 * Both maps live on a grid offset by {10,10,10} cm; the comparison must walk THAT grid (offset +
 * half-resolution centres), not the world origin.
 */
TEST(MapsComparison, SharedNonzeroOffsetGridScoresPerfect) {
    const Position3D offset{10.0 * x_extent[cm], 10.0 * y_extent[cm], 10.0 * z_extent[cm]};
    const types::MappingBounds bounds{
        10.0 * x_extent[cm], 14.0 * x_extent[cm],
        10.0 * y_extent[cm], 14.0 * y_extent[cm],
        10.0 * z_extent[cm], 14.0 * z_extent[cm],
    };
    const types::MapConfig offset_config{bounds, offset, 1.0 * cm};

    // One occupied voxel at the same world position ({10.5, 10.5, 10.5} centre) in both maps.
    const Position3D occupied_center{10.5 * x_extent[cm], 10.5 * y_extent[cm], 10.5 * z_extent[cm]};
    Map3DImpl origin{Map3DImpl::makeEmptyArray(offset_config), offset_config};
    origin.set(occupied_center, types::VoxelOccupancy::Occupied);
    Map3DImpl target{Map3DImpl::makeEmptyArray(offset_config), offset_config};
    target.set(occupied_center, types::VoxelOccupancy::Occupied);

    const std::vector<double> scores = MapsComparison::compare(origin, {&target});

    ASSERT_EQ(scores.size(), 1u);
    EXPECT_DOUBLE_EQ(scores[0], 100.0);
}

/**
 * @brief An all-unmapped origin against an occupied target scores 0 (the target sets the union).
 */
TEST(MapsComparison, EmptyOriginVsOccupiedTargetScoresZero) {
    Map3DImpl origin = mapWith({});
    Map3DImpl target = mapWith({{1, 1, 1}});

    const std::vector<double> scores = MapsComparison::compare(origin, {&target});

    ASSERT_EQ(scores.size(), 1u);
    EXPECT_DOUBLE_EQ(scores[0], 0.0);
}