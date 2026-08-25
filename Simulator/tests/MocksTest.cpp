/**
 * @file MocksTest.cpp
 * @brief Coverage of the simulated pose, actuator, and lidar.
 * @note The actuator cases pin down the angle convention (`0 deg` = +X east, `90 deg` = +Y south),
 *       which the lidar and any plugin-side movement prediction must agree with. A silent
 *       disagreement there would send a validated move somewhere else entirely.
 */

#include <Simulator/Map3DImpl.h>
#include <Simulator/MockGPS.h>
#include <Simulator/MockLidar.h>
#include <Simulator/MockMovement.h>

#include <gtest/gtest.h>

#include <cstddef>
#include <limits>

namespace {

using common::cm;
using common::deg;
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
 * @brief Build a heading from a plain degree value.
 * @param degrees Horizontal angle in degrees; altitude is left level.
 * @return The orientation.
 */
[[nodiscard]] common::Orientation heading(double degrees) {
    return common::Orientation{degrees * common::horizontal_angle[deg],
                               0.0 * common::altitude_angle[deg]};
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
 * @brief Strip units from a length.
 * @param length Quantity to convert.
 * @return The value in centimetres.
 */
[[nodiscard]] double asCm(common::PhysicalLength length) {
    return length.force_numerical_value_in(cm);
}

TEST(MockGPS, ReportsTheExactPoseWithoutQuantizing) {
    /**
     * @note A 5 cm "precision" that actually applied would snap 12.3 to 10 or 15. It deliberately
     *       does not: the algorithm must be able to trust the pose it plans against.
     */
    const simulator::MockGPS gps{pos(12.3, 7.7, 3.1), heading(37.0), 5.0 * cm};

    EXPECT_DOUBLE_EQ(gps.position().x.force_numerical_value_in(cm), 12.3);
    EXPECT_DOUBLE_EQ(gps.position().y.force_numerical_value_in(cm), 7.7);
    EXPECT_DOUBLE_EQ(gps.heading().horizontal.force_numerical_value_in(deg), 37.0);
}

TEST(MockMovement, AdvanceAtZeroDegreesMovesEast) {
    simulator::MockGPS gps{pos(0.0, 0.0, 0.0), heading(0.0), 1.0 * cm};
    simulator::MockMovement movement{gps};

    ASSERT_TRUE(static_cast<bool>(movement.advance(10.0 * cm)));

    EXPECT_NEAR(gps.position().x.force_numerical_value_in(cm), 10.0, 1e-9);
    EXPECT_NEAR(gps.position().y.force_numerical_value_in(cm), 0.0, 1e-9);
    EXPECT_NEAR(gps.position().z.force_numerical_value_in(cm), 0.0, 1e-9);
}

TEST(MockMovement, AdvanceAtNinetyDegreesMovesSouth) {
    simulator::MockGPS gps{pos(0.0, 0.0, 0.0), heading(90.0), 1.0 * cm};
    simulator::MockMovement movement{gps};

    ASSERT_TRUE(static_cast<bool>(movement.advance(10.0 * cm)));

    EXPECT_NEAR(gps.position().x.force_numerical_value_in(cm), 0.0, 1e-9);
    EXPECT_NEAR(gps.position().y.force_numerical_value_in(cm), 10.0, 1e-9);
}

TEST(MockMovement, ElevateChangesOnlyAltitude) {
    simulator::MockGPS gps{pos(1.0, 2.0, 3.0), heading(45.0), 1.0 * cm};
    simulator::MockMovement movement{gps};

    ASSERT_TRUE(static_cast<bool>(movement.elevate(-1.5 * cm)));

    EXPECT_NEAR(gps.position().x.force_numerical_value_in(cm), 1.0, 1e-9);
    EXPECT_NEAR(gps.position().y.force_numerical_value_in(cm), 2.0, 1e-9);
    EXPECT_NEAR(gps.position().z.force_numerical_value_in(cm), 1.5, 1e-9);
}

TEST(MockMovement, RotateChangesOnlyHeading) {
    simulator::MockGPS gps{pos(1.0, 2.0, 3.0), heading(0.0), 1.0 * cm};
    simulator::MockMovement movement{gps};

    ASSERT_TRUE(static_cast<bool>(
        movement.rotate(common::types::RotationDirection::Left, 30.0 * common::horizontal_angle[deg])));
    EXPECT_NEAR(gps.heading().horizontal.force_numerical_value_in(deg), 30.0, 1e-9);

    ASSERT_TRUE(static_cast<bool>(
        movement.rotate(common::types::RotationDirection::Right, 50.0 * common::horizontal_angle[deg])));

    /**
     * @note 340 rather than -20: the heading is wrapped into [0, 360). The two describe the same
     *       direction, and the wrapping is what keeps the angle bounded over a mission of thousands
     *       of turns - an unbounded heading eventually makes the sine and cosine of an axis-aligned
     *       direction inexact, which is enough to push a drone travelling along voxel boundaries
     *       onto the wrong side of one.
     */
    EXPECT_NEAR(gps.heading().horizontal.force_numerical_value_in(deg), 340.0, 1e-9);

    EXPECT_NEAR(gps.position().x.force_numerical_value_in(cm), 1.0, 1e-9);
}

TEST(MockMovement, TheHeadingStaysBoundedOverManyTurns) {
    /**
     * @note The regression this guards. Before wrapping, a long mission drove the heading into the
     *       tens of thousands of degrees, and `cos` of an angle that large returns roughly 1e-16
     *       where zero is correct. A drone whose start position is a multiple of the map resolution
     *       rides voxel boundaries, so that residue decides which cell it is judged to be in - and a
     *       move through open space gets refused for clipping a wall one column over.
     */
    simulator::MockGPS gps{pos(0.0, 0.0, 0.0), heading(0.0), 1.0 * cm};
    simulator::MockMovement movement{gps};

    for (int i = 0; i < 200; ++i) {
        ASSERT_TRUE(static_cast<bool>(movement.rotate(
            common::types::RotationDirection::Left, 90.0 * common::horizontal_angle[deg])));
    }

    const double final_deg = gps.heading().horizontal.force_numerical_value_in(deg);
    EXPECT_GE(final_deg, 0.0);
    EXPECT_LT(final_deg, 360.0);

    /**
     * @note 200 quarter-turns is a whole number of revolutions, so the drone faces east again. An
     *       advance must therefore move purely along +X, with the other axes exactly unchanged -
     *       not merely close.
     */
    ASSERT_TRUE(static_cast<bool>(movement.advance(10.0 * cm)));
    EXPECT_EQ(gps.position().y.force_numerical_value_in(cm), 0.0);
    EXPECT_EQ(gps.position().z.force_numerical_value_in(cm), 0.0);
    EXPECT_NEAR(gps.position().x.force_numerical_value_in(cm), 10.0, 1e-9);
}

TEST(MockMovement, NeverRefusesAnything) {
    /**
     * @note Flying far outside the world still reports success. Validation belongs to whoever drives
     *       the drone, and putting it here would silently constrain a third-party plugin.
     */
    simulator::MockGPS gps{pos(0.0, 0.0, 0.0), heading(0.0), 1.0 * cm};
    simulator::MockMovement movement{gps};

    EXPECT_TRUE(static_cast<bool>(movement.advance(1.0e6 * cm)));
    EXPECT_NEAR(gps.position().x.force_numerical_value_in(cm), 1.0e6, 1e-3);
}

/**
 * @brief A world with a single wall, for the lidar cases.
 */
class MockLidarTest : public ::testing::Test {
protected:
    /**
     * @brief Build a 100 cm cube at 1 cm resolution with a wall at x = 50.
     */
    void SetUp() override {
        config_ = cubeConfig(100.0, 1.0);
        map_ = std::make_unique<simulator::Map3DImpl>(
            simulator::Map3DImpl::makeEmptyArray(config_), config_);
        for (double y = 0.5; y < 100.0; y += 1.0) {
            for (double z = 0.5; z < 100.0; z += 1.0) {
                map_->set(pos(50.5, y, z), common::types::VoxelOccupancy::Occupied);
            }
        }
    }

    /**
     * @brief A lidar configuration with a chosen number of beam circles.
     * @param circles How many circles the sensor emits.
     * @return The configuration.
     */
    [[nodiscard]] static common::types::LidarConfigData lidarConfig(std::size_t circles) {
        common::types::LidarConfigData config{};
        config.z_min = 10.0 * cm;
        config.z_max = 80.0 * cm;
        config.d = 2.0 * cm;
        config.fov_circles = circles;
        return config;
    }

    common::types::MapConfig config_{};
    std::unique_ptr<simulator::Map3DImpl> map_{};
};

TEST_F(MockLidarTest, BeamCountFollowsThePowerOfFourRule) {
    const simulator::MockGPS gps{pos(10.0, 50.0, 50.0), heading(0.0), 1.0 * cm};

    for (std::size_t circles = 1; circles <= 4; ++circles) {
        const simulator::MockLidar lidar{lidarConfig(circles), *map_, gps};
        std::size_t expected = 0;
        std::size_t beams = 1;
        for (std::size_t i = 0; i < circles; ++i) {
            expected += beams;
            beams *= 4;
        }
        EXPECT_EQ(lidar.scan(heading(0.0)).size(), expected) << "circles=" << circles;
    }
}

TEST_F(MockLidarTest, ZeroCirclesSeesNothing) {
    const simulator::MockGPS gps{pos(10.0, 50.0, 50.0), heading(0.0), 1.0 * cm};
    const simulator::MockLidar lidar{lidarConfig(0), *map_, gps};

    EXPECT_TRUE(lidar.scan(heading(0.0)).empty());
}

TEST_F(MockLidarTest, TheCentralBeamFindsTheWall) {
    const simulator::MockGPS gps{pos(20.0, 50.0, 50.0), heading(0.0), 1.0 * cm};
    const simulator::MockLidar lidar{lidarConfig(1), *map_, gps};

    const common::types::LidarScanResult scan = lidar.scan(heading(0.0));
    ASSERT_EQ(scan.size(), 1u);
    EXPECT_NEAR(asCm(scan.front().distance), 30.0, 0.2)
        << "the wall starts at x=50 and the drone is at x=20, within one marching step";
}

TEST_F(MockLidarTest, AHitInsideTheMinimumRangeReportsZero) {
    /**
     * @note The drone sits 5 cm from the wall, well inside `z_min` of 10. The sensor can tell
     *       something is there but not where, so it must report 0 rather than 5.
     */
    const simulator::MockGPS gps{pos(45.5, 50.0, 50.0), heading(0.0), 1.0 * cm};
    const simulator::MockLidar lidar{lidarConfig(1), *map_, gps};

    const common::types::LidarScanResult scan = lidar.scan(heading(0.0));
    ASSERT_EQ(scan.size(), 1u);
    EXPECT_DOUBLE_EQ(asCm(scan.front().distance), 0.0);
}

TEST_F(MockLidarTest, ABeamIntoOpenSpaceReportsTheMissSentinel) {
    const simulator::MockGPS gps{pos(20.0, 50.0, 50.0), heading(0.0), 1.0 * cm};
    const simulator::MockLidar lidar{lidarConfig(1), *map_, gps};

    const common::types::LidarScanResult scan = lidar.scan(heading(180.0));
    ASSERT_EQ(scan.size(), 1u);
    EXPECT_DOUBLE_EQ(asCm(scan.front().distance), std::numeric_limits<double>::max())
        << "facing away from the wall there is nothing to hit within z_max";
}

TEST_F(MockLidarTest, ReportedAnglesAreRelativeToTheScanDirection) {
    /**
     * @note The drone faces 90 deg and scans straight ahead. The reported angle must be the scan
     *       orientation as asked for, not the absolute 90 deg the beam actually travelled - a caller
     *       that added the heading back would otherwise double-apply it.
     */
    const simulator::MockGPS gps{pos(20.0, 50.0, 50.0), heading(90.0), 1.0 * cm};
    const simulator::MockLidar lidar{lidarConfig(1), *map_, gps};

    const common::types::LidarScanResult scan = lidar.scan(heading(0.0));
    ASSERT_EQ(scan.size(), 1u);
    EXPECT_DOUBLE_EQ(scan.front().angle.horizontal.force_numerical_value_in(deg), 0.0);
}

TEST_F(MockLidarTest, TheHeadingRotatesWhatTheBeamActuallyHits) {
    const simulator::MockGPS facing_wall{pos(20.0, 50.0, 50.0), heading(0.0), 1.0 * cm};
    const simulator::MockGPS facing_away{pos(20.0, 50.0, 50.0), heading(180.0), 1.0 * cm};

    const simulator::MockLidar hits{lidarConfig(1), *map_, facing_wall};
    const simulator::MockLidar misses{lidarConfig(1), *map_, facing_away};

    EXPECT_LT(asCm(hits.scan(heading(0.0)).front().distance), 100.0);
    EXPECT_DOUBLE_EQ(asCm(misses.scan(heading(0.0)).front().distance),
                     std::numeric_limits<double>::max());
}

} // namespace
