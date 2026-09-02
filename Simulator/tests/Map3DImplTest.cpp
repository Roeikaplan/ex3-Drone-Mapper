/**
 * @file Map3DImplTest.cpp
 * @brief Coverage of world-to-voxel geometry, the signed-byte storage contract, and serialisation.
 * @note Fixtures are generated rather than checked in: `makeEmptyArray` -> `set` -> `save` ->
 *       `loadArray` exercises the write and read paths against each other, so a change that breaks
 *       one is caught even if it breaks the other identically.
 */

#include <Simulator/Map3DImpl.h>

#include <gtest/gtest.h>

#include <cstdlib>
#include <filesystem>
#include <string>

namespace {

namespace fs = std::filesystem;

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
 * @brief Build a cubic map geometry anchored at an offset.
 * @param span_cm Extent of each axis in centimetres, measured from the offset.
 * @param resolution_cm Voxel edge length in centimetres.
 * @param offset_cm Origin of the grid on every axis, in centimetres.
 * @return A `MapConfig` describing a cube of voxels.
 */
[[nodiscard]] common::types::MapConfig cubeConfig(double span_cm, double resolution_cm,
                                                  double offset_cm = 0.0) {
    common::types::MapConfig config{};
    config.resolution = resolution_cm * cm;
    config.offset = pos(offset_cm, offset_cm, offset_cm);
    config.boundaries.min_x = offset_cm * x_extent[cm];
    config.boundaries.max_x = (offset_cm + span_cm) * x_extent[cm];
    config.boundaries.min_y = offset_cm * y_extent[cm];
    config.boundaries.max_y = (offset_cm + span_cm) * y_extent[cm];
    config.boundaries.min_height = offset_cm * z_extent[cm];
    config.boundaries.max_height = (offset_cm + span_cm) * z_extent[cm];
    return config;
}

/**
 * @brief Gives each test its own scratch directory for generated `.npy` files.
 */
class Map3DImplTest : public ::testing::Test {
protected:
    /**
     * @brief Create a uniquely named scratch directory under the system temp folder.
     */
    void SetUp() override {
        const ::testing::TestInfo* const info =
            ::testing::UnitTest::GetInstance()->current_test_info();
        dir_ = fs::temp_directory_path() /
               ("ex3_map3d_" + std::string{info->name()} + "_" + std::to_string(::getpid()));
        std::error_code ec;
        fs::remove_all(dir_, ec);
        fs::create_directories(dir_, ec);
    }

    /**
     * @brief Remove the scratch directory and everything in it.
     */
    void TearDown() override {
        std::error_code ec;
        fs::remove_all(dir_, ec);
    }

    fs::path dir_{};
};

/**
 * @brief A freshly allocated map reads `Unmapped` everywhere, not `Empty`.
 * @note The distinction is the whole basis of scoring and of frontier search: `Empty` is a claim the
 *       drone proved, `Unmapped` means nobody has looked. A zero-filled buffer would say the wrong
 *       one of those in every cell.
 */
TEST_F(Map3DImplTest, NewMapStartsEntirelyUnmapped) {
    const common::types::MapConfig config = cubeConfig(10.0, 1.0);
    const simulator::Map3DImpl map{simulator::Map3DImpl::makeEmptyArray(config), config};

    EXPECT_EQ(map.atVoxel(pos(0.5, 0.5, 0.5)), common::types::VoxelOccupancy::Unmapped);
    EXPECT_EQ(map.atVoxel(pos(9.5, 9.5, 9.5)), common::types::VoxelOccupancy::Unmapped);
}

/**
 * @brief Bounds are reported with an exclusive upper edge, and writes outside them are dropped.
 * @note Dropping rather than wrapping is the point. An out-of-range index cast to an unsigned type
 *       would land somewhere real inside the buffer and corrupt an unrelated cell.
 */
TEST_F(Map3DImplTest, ReportsBoundsAndDropsOutOfBoundsWrites) {
    const common::types::MapConfig config = cubeConfig(10.0, 1.0);
    simulator::Map3DImpl map{simulator::Map3DImpl::makeEmptyArray(config), config};

    EXPECT_TRUE(map.isInBounds(pos(0.0, 0.0, 0.0)));
    EXPECT_TRUE(map.isInBounds(pos(9.9, 9.9, 9.9)));
    EXPECT_FALSE(map.isInBounds(pos(-0.1, 5.0, 5.0)));
    EXPECT_FALSE(map.isInBounds(pos(10.0, 5.0, 5.0))) << "the upper bound is exclusive";

    EXPECT_EQ(map.atVoxel(pos(-1.0, 5.0, 5.0)), common::types::VoxelOccupancy::OutOfBounds);

    map.set(pos(-1.0, 5.0, 5.0), common::types::VoxelOccupancy::Occupied);
    EXPECT_EQ(map.atVoxel(pos(-1.0, 5.0, 5.0)), common::types::VoxelOccupancy::OutOfBounds)
        << "an out-of-bounds write must be dropped, not wrapped into a real cell";
}

/**
 * @brief All five occupancy values survive a save and reload unchanged.
 * @note Three of the five are negative sentinels, so this fails immediately if the buffer is ever
 *       read back as unsigned - the values would return as 253, 254 and 255.
 */
TEST_F(Map3DImplTest, EveryOccupancyValueRoundTripsThroughAFile) {
    const common::types::MapConfig config = cubeConfig(10.0, 1.0);
    const fs::path file = dir_ / "roundtrip.npy";

    {
        simulator::Map3DImpl map{simulator::Map3DImpl::makeEmptyArray(config), config};
        map.set(pos(0.5, 0.5, 0.5), common::types::VoxelOccupancy::Occupied);
        map.set(pos(1.5, 0.5, 0.5), common::types::VoxelOccupancy::Empty);
        map.set(pos(2.5, 0.5, 0.5), common::types::VoxelOccupancy::PotentiallyOccupied);
        map.set(pos(3.5, 0.5, 0.5), common::types::VoxelOccupancy::OutOfBounds);
        map.save(file);
    }

    const simulator::Map3DImpl reloaded{simulator::Map3DImpl::loadArray(file), config};
    EXPECT_EQ(reloaded.atVoxel(pos(0.5, 0.5, 0.5)), common::types::VoxelOccupancy::Occupied);
    EXPECT_EQ(reloaded.atVoxel(pos(1.5, 0.5, 0.5)), common::types::VoxelOccupancy::Empty);
    EXPECT_EQ(reloaded.atVoxel(pos(2.5, 0.5, 0.5)),
              common::types::VoxelOccupancy::PotentiallyOccupied)
        << "the negative sentinels only survive if the buffer is read as signed";
    EXPECT_EQ(reloaded.atVoxel(pos(3.5, 0.5, 0.5)), common::types::VoxelOccupancy::OutOfBounds);
    EXPECT_EQ(reloaded.atVoxel(pos(4.5, 0.5, 0.5)), common::types::VoxelOccupancy::Unmapped);
}

/**
 * @brief A voxel covers a whole resolution-sized cell, not just the point that was written.
 * @note Checked at 2 cm rather than 1 cm precisely so that a coordinate and its cell differ - at unit
 *       resolution an off-by-one in the index arithmetic would be invisible.
 */
TEST_F(Map3DImplTest, VoxelsSpanTheResolutionNotASinglePoint) {
    const common::types::MapConfig config = cubeConfig(10.0, 2.0);
    simulator::Map3DImpl map{simulator::Map3DImpl::makeEmptyArray(config), config};

    map.set(pos(3.0, 1.0, 1.0), common::types::VoxelOccupancy::Occupied);

    EXPECT_EQ(map.atVoxel(pos(2.0, 1.0, 1.0)), common::types::VoxelOccupancy::Occupied)
        << "2.0 and 3.0 fall in the same 2 cm cell";
    EXPECT_EQ(map.atVoxel(pos(3.9, 1.0, 1.0)), common::types::VoxelOccupancy::Occupied);
    EXPECT_EQ(map.atVoxel(pos(4.0, 1.0, 1.0)), common::types::VoxelOccupancy::Unmapped)
        << "4.0 starts the next cell";
}

/**
 * @brief A map with an offset covers world coordinates around that offset, not around the origin.
 * @note `house_simulation.yaml` sets a height offset of 150, so an implementation that ignored the
 *       offset would place that whole scenario's drone outside its own map.
 */
TEST_F(Map3DImplTest, OffsetShiftsTheGridIntoWorldCoordinates) {
    const common::types::MapConfig config = cubeConfig(10.0, 1.0, 100.0);
    simulator::Map3DImpl map{simulator::Map3DImpl::makeEmptyArray(config), config};

    EXPECT_FALSE(map.isInBounds(pos(0.0, 0.0, 0.0))) << "the origin is now outside the map";
    EXPECT_TRUE(map.isInBounds(pos(105.0, 105.0, 105.0)));

    map.set(pos(105.5, 105.5, 105.5), common::types::VoxelOccupancy::Occupied);
    EXPECT_EQ(map.atVoxel(pos(105.5, 105.5, 105.5)), common::types::VoxelOccupancy::Occupied);
}

/**
 * @brief A span that does not divide evenly still gets its partial trailing voxel.
 */
TEST_F(Map3DImplTest, PartialTrailingVoxelIsKept) {
    /**
     * @note A 10 cm span at 3 cm resolution needs four cells, not three: `ceil(10/3)`. The fourth is
     *       partial, and dropping it would silently make the far edge of every map unmappable.
     */
    const common::types::MapConfig config = cubeConfig(10.0, 3.0);
    const simulator::Map3DImpl map{simulator::Map3DImpl::makeEmptyArray(config), config};

    EXPECT_TRUE(map.isInBounds(pos(9.9, 9.9, 9.9)));
    EXPECT_FALSE(map.isInBounds(pos(12.0, 0.0, 0.0)));
}

/**
 * @brief A zero resolution throws at allocation and yields a grid where every lookup is out of bounds.
 * @note Two defences for the same degenerate config: refuse to build an array from it, and - if one
 *       is built from a different config - never divide by it during a lookup.
 */
TEST_F(Map3DImplTest, NonPositiveResolutionIsRejectedRatherThanDividedBy) {
    common::types::MapConfig config = cubeConfig(10.0, 1.0);
    config.resolution = 0.0 * cm;

    EXPECT_THROW((void)simulator::Map3DImpl::makeEmptyArray(config), std::invalid_argument);

    const common::types::MapConfig usable = cubeConfig(10.0, 1.0);
    const simulator::Map3DImpl map{simulator::Map3DImpl::makeEmptyArray(usable), config};
    EXPECT_EQ(map.atVoxel(pos(1.0, 1.0, 1.0)), common::types::VoxelOccupancy::OutOfBounds)
        << "a zero-resolution config has no grid, so every lookup is out of bounds";
}

/**
 * @brief Constructing over a null array throws rather than deferring the failure to first use.
 */
TEST_F(Map3DImplTest, NullArrayIsRejected) {
    EXPECT_THROW(simulator::Map3DImpl{std::unique_ptr<NpyArray>{}}, std::invalid_argument);
}

/**
 * @brief A shipped `.npy` scenario map loads and satisfies the one-byte-per-voxel storage contract.
 * @note The only test here that reads real input rather than a generated fixture, so it is what
 *       catches a dtype or dimensionality assumption that holds for our own writes but not theirs.
 */
TEST_F(Map3DImplTest, LoadingARealScenarioMap) {
    const fs::path map_file = fs::path{DRONE_SOURCE_DIR} / "inputs" / "map" / "scenario_small.npy";
    ASSERT_TRUE(fs::exists(map_file)) << map_file.string();

    const std::unique_ptr<NpyArray> array = simulator::Map3DImpl::loadArray(map_file);
    ASSERT_EQ(array->Shape().size(), 3u);
    EXPECT_EQ(array->SizeValueBytes(), 1u) << "the storage contract is one byte per voxel";
    EXPECT_GT(array->NumValue(), 0u);
}

/**
 * @brief A missing map file throws rather than yielding an empty map.
 * @note This is the exception the run factory relies on: it becomes `RUN_FAILED` and scores that
 *       combination -1. Returning an empty map instead would score it as a legitimate bad result.
 */
TEST_F(Map3DImplTest, LoadingAMissingFileThrows) {
    EXPECT_THROW((void)simulator::Map3DImpl::loadArray(dir_ / "absent.npy"), std::runtime_error);
}

} // namespace
