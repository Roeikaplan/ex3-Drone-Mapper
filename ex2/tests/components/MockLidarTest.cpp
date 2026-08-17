#include <drone_mapper/MockLidar.h>

#include <drone_mapper/IGPS.h>
#include <drone_mapper/IMap3D.h>

#include <gtest/gtest.h>

#include <cstddef>
#include <functional>
#include <limits>

namespace {

using namespace drone_mapper;

// A hidden map whose occupancy is a predicate — used to place a "wall" the beams can hit.
class WallMap : public IMap3D {
public:
    WallMap(types::MapConfig config, std::function<bool(const Position3D&)> wall)
        : config_(config), wall_(std::move(wall)) {}

    [[nodiscard]] types::VoxelOccupancy atVoxel(const Position3D& pos) const override {
        return wall_(pos) ? types::VoxelOccupancy::Occupied : types::VoxelOccupancy::Unmapped;
    }
    [[nodiscard]] types::MapConfig getMapConfig() const override {
        return config_;
    }
    [[nodiscard]] bool isInBounds(const Position3D&) const override {
        return true;
    }

private:
    types::MapConfig config_;
    std::function<bool(const Position3D&)> wall_;
};

// Fixed, directly-settable pose.
class FakeGPS : public IGPS {
public:
    [[nodiscard]] Position3D position() const override {
        return position_;
    }
    [[nodiscard]] Orientation heading() const override {
        return heading_;
    }
    Position3D position_{};
    Orientation heading_{};
};

/**
 * @brief A LiDAR config; resolution of the beam march comes from the map (0.1·resolution).
 */
[[nodiscard]] types::LidarConfigData lidarCfg(double z_min, double z_max, double d,
                                              std::size_t fov_circles) {
    return types::LidarConfigData{z_min * cm, z_max * cm, d * cm, fov_circles};
}

/**
 * @brief Map geometry whose 10-cm resolution gives a 1-cm ray-march step.
 */
[[nodiscard]] types::MapConfig mapCfg() {
    return types::MapConfig{types::MappingBounds{}, Position3D{}, 10.0 * cm};
}

/// Wall predicate: Occupied at or beyond `threshold` cm on +X.
[[nodiscard]] std::function<bool(const Position3D&)> xWall(double threshold) {
    return [threshold](const Position3D& p) {
        return p.x.force_numerical_value_in(cm) >= threshold;
    };
}

/// Wall predicate: Occupied at or beyond `threshold` cm on +Y.
[[nodiscard]] std::function<bool(const Position3D&)> yWall(double threshold) {
    return [threshold](const Position3D& p) {
        return p.y.force_numerical_value_in(cm) >= threshold;
    };
}

/// Wall predicate: Occupied at or beyond `threshold` cm on +Z (a ceiling).
[[nodiscard]] std::function<bool(const Position3D&)> zWall(double threshold) {
    return [threshold](const Position3D& p) {
        return p.z.force_numerical_value_in(cm) >= threshold;
    };
}

[[nodiscard]] std::function<bool(const Position3D&)> noWall() {
    return [](const Position3D&) { return false; };
}

constexpr double kMiss = std::numeric_limits<double>::max();

} // namespace

/**
 * @brief config() returns the configuration the sensor was built with.
 */
TEST(MockLidar, ConfigReturnsInjected) {
    WallMap map{mapCfg(), noWall()};
    FakeGPS gps;
    MockLidar lidar{lidarCfg(20.0, 120.0, 2.5, 5), map, gps};

    const types::LidarConfigData config = lidar.config();
    EXPECT_DOUBLE_EQ(config.z_min.force_numerical_value_in(cm), 20.0);
    EXPECT_DOUBLE_EQ(config.z_max.force_numerical_value_in(cm), 120.0);
    EXPECT_DOUBLE_EQ(config.d.force_numerical_value_in(cm), 2.5);
    EXPECT_EQ(config.fov_circles, 5u);
}

/**
 * @brief Beam count follows the 4^circle ring rule: 1, 21, and 341 beams for fov 1, 3, 5.
 */
TEST(MockLidar, BeamCounts) {
    WallMap map{mapCfg(), noWall()};
    FakeGPS gps;
    const Orientation forward{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]};

    EXPECT_EQ(MockLidar(lidarCfg(20, 120, 2.5, 1), map, gps).scan(forward).size(), 1u);
    EXPECT_EQ(MockLidar(lidarCfg(20, 120, 2.5, 3), map, gps).scan(forward).size(), 21u);
    EXPECT_EQ(MockLidar(lidarCfg(20, 120, 2.5, 5), map, gps).scan(forward).size(), 341u);
}

/**
 * @brief fov_circles == 0 yields no beams at all.
 */
TEST(MockLidar, ZeroFovYieldsNoBeams) {
    WallMap map{mapCfg(), noWall()};
    FakeGPS gps;
    MockLidar lidar{lidarCfg(20, 120, 2.5, 0), map, gps};

    EXPECT_TRUE(lidar.scan(Orientation{}).empty());
}

/**
 * @brief The central beam reports the distance to a wall it hits.
 */
TEST(MockLidar, CentralBeamHitsWallAtDistance) {
    WallMap map{mapCfg(), xWall(50.0)};
    FakeGPS gps; // origin, heading 0 (=> beam +X)
    MockLidar lidar{lidarCfg(20, 120, 2.5, 1), map, gps};

    const types::LidarScanResult result = lidar.scan(Orientation{});
    ASSERT_EQ(result.size(), 1u);
    EXPECT_NEAR(result[0].distance.force_numerical_value_in(cm), 50.0, 1.5); // within one march step
}

/**
 * @brief A hit closer than z_min is reported as distance 0 (detected but unmeasurable).
 */
TEST(MockLidar, BelowZMinReturnsZero) {
    WallMap map{mapCfg(), xWall(5.0)};
    FakeGPS gps;
    MockLidar lidar{lidarCfg(20, 120, 2.5, 1), map, gps}; // z_min 20 > 5

    const types::LidarScanResult result = lidar.scan(Orientation{});
    ASSERT_EQ(result.size(), 1u);
    EXPECT_DOUBLE_EQ(result[0].distance.force_numerical_value_in(cm), 0.0);
}

/**
 * @brief A wall beyond z_max is not detected — the beam misses.
 */
TEST(MockLidar, BeyondZMaxMisses) {
    WallMap map{mapCfg(), xWall(200.0)};
    FakeGPS gps;
    MockLidar lidar{lidarCfg(20, 120, 2.5, 1), map, gps}; // z_max 120 < 200

    const types::LidarScanResult result = lidar.scan(Orientation{});
    ASSERT_EQ(result.size(), 1u);
    EXPECT_DOUBLE_EQ(result[0].distance.force_numerical_value_in(cm), kMiss);
}

/**
 * @brief Empty space returns a miss for the central beam.
 */
TEST(MockLidar, EmptySpaceMisses) {
    WallMap map{mapCfg(), noWall()};
    FakeGPS gps;
    MockLidar lidar{lidarCfg(20, 120, 2.5, 1), map, gps};

    const types::LidarScanResult result = lidar.scan(Orientation{});
    ASSERT_EQ(result.size(), 1u);
    EXPECT_DOUBLE_EQ(result[0].distance.force_numerical_value_in(cm), kMiss);
}

/**
 * @brief Beam direction matters: with a +X-only wall, scanning +X hits but scanning +Y misses.
 */
TEST(MockLidar, DirectionMatters) {
    WallMap map{mapCfg(), xWall(50.0)};
    FakeGPS gps;
    MockLidar lidar{lidarCfg(20, 120, 2.5, 1), map, gps};

    const double east = lidar.scan(Orientation{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]})
                            .at(0)
                            .distance.force_numerical_value_in(cm);
    const double south =
        lidar.scan(Orientation{90.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]})
            .at(0)
            .distance.force_numerical_value_in(cm);

    EXPECT_NEAR(east, 50.0, 1.5);
    EXPECT_DOUBLE_EQ(south, kMiss);
}

/**
 * @brief The scan orientation is composed with the sensor heading.
 *
 * With the GPS heading at 90 deg and a scan orientation of 0, the central beam points +Y and hits a
 * +Y wall.
 */
TEST(MockLidar, HeadingComposedWithScanOrientation) {
    WallMap map{mapCfg(), yWall(50.0)};
    FakeGPS gps;
    gps.heading_ = Orientation{90.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]};
    MockLidar lidar{lidarCfg(20, 120, 2.5, 1), map, gps};

    const types::LidarScanResult result = lidar.scan(Orientation{});
    ASSERT_EQ(result.size(), 1u);
    EXPECT_NEAR(result[0].distance.force_numerical_value_in(cm), 50.0, 1.5);
}

/**
 * @brief The central hit's angle echoes the REQUESTED (heading-relative) scan orientation.
 *
 * Consumers (ScanResultToVoxels) re-compose hits with the drone pose, so the sensor must report the
 * relative beam angle, not the absolute world direction it actually traced.
 */
TEST(MockLidar, CentralHitAngleEchoesScanOrientation) {
    WallMap map{mapCfg(), noWall()};
    FakeGPS gps;
    gps.heading_ = Orientation{90.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]};
    MockLidar lidar{lidarCfg(20, 120, 2.5, 1), map, gps};
    const Orientation requested{30.0 * horizontal_angle[deg], -10.0 * altitude_angle[deg]};

    const types::LidarScanResult result = lidar.scan(requested);

    ASSERT_EQ(result.size(), 1u);
    EXPECT_DOUBLE_EQ(result[0].angle.horizontal.force_numerical_value_in(deg), 30.0);
    EXPECT_DOUBLE_EQ(result[0].angle.altitude.force_numerical_value_in(deg), -10.0);
}

/**
 * @brief An altitude-angled scan measures vertically: pointing straight up hits the ceiling.
 */
TEST(MockLidar, AltitudeScanHitsCeiling) {
    WallMap map{mapCfg(), zWall(50.0)};
    FakeGPS gps; // origin, level heading
    MockLidar lidar{lidarCfg(20, 120, 2.5, 1), map, gps};

    const types::LidarScanResult result =
        lidar.scan(Orientation{0.0 * horizontal_angle[deg], 90.0 * altitude_angle[deg]});

    ASSERT_EQ(result.size(), 1u);
    EXPECT_NEAR(result[0].distance.force_numerical_value_in(cm), 50.0, 1.5);
}

/**
 * @brief Distance is measured from the sensor's CURRENT position, not the world origin.
 *
 * With the drone at x=30 and a wall from x=80, the +X beam must read ~50 (not ~80).
 */
TEST(MockLidar, ScanFromOffsetPositionMeasuresRelativeDistance) {
    WallMap map{mapCfg(), xWall(80.0)};
    FakeGPS gps;
    gps.position_ = Position3D{30.0 * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]};
    MockLidar lidar{lidarCfg(20, 120, 2.5, 1), map, gps};

    const types::LidarScanResult result = lidar.scan(Orientation{});

    ASSERT_EQ(result.size(), 1u);
    EXPECT_NEAR(result[0].distance.force_numerical_value_in(cm), 50.0, 1.5);
}

/**
 * @brief A negative scan orientation composes with the heading (90 + -90 = world 0 -> +X wall hit).
 */
TEST(MockLidar, NegativeScanOrientationComposesWithHeading) {
    WallMap map{mapCfg(), xWall(50.0)};
    FakeGPS gps;
    gps.heading_ = Orientation{90.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]};
    MockLidar lidar{lidarCfg(20, 120, 2.5, 1), map, gps};

    const types::LidarScanResult result =
        lidar.scan(Orientation{-90.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]});

    ASSERT_EQ(result.size(), 1u);
    EXPECT_NEAR(result[0].distance.force_numerical_value_in(cm), 50.0, 1.5);
}

/**
 * @brief Every beam of a multi-ring cone hits a large wall — the outer rings really are traced.
 *
 * fov 2 = 1 centre + 4 ring beams against a thick +X wall: no beam may miss, and each ring beam
 * (slightly tilted off-axis) reads a distance at least the wall's perpendicular ~50 cm.
 */
TEST(MockLidar, OuterRingBeamsAllHitLargeWall) {
    WallMap map{mapCfg(), xWall(50.0)};
    FakeGPS gps;
    MockLidar lidar{lidarCfg(20, 120, 2.5, 2), map, gps};

    const types::LidarScanResult result = lidar.scan(Orientation{});

    ASSERT_EQ(result.size(), 5u); // 1 + 4^1
    for (std::size_t i = 0; i < result.size(); ++i) {
        const double distance = result[i].distance.force_numerical_value_in(cm);
        EXPECT_NE(distance, kMiss) << "beam " << i;
        EXPECT_GE(distance, 50.0 - 1.5) << "beam " << i;
    }
}

/**
 * @brief A wall at or beyond z_min is measured normally — the 0 sentinel is only for closer hits.
 *
 * Complements BelowZMinReturnsZero: with z_min 20 and the wall at 25, the beam must report ~25, not 0.
 */
TEST(MockLidar, WallAtOrBeyondZMinMeasured) {
    WallMap map{mapCfg(), xWall(25.0)};
    FakeGPS gps;
    MockLidar lidar{lidarCfg(20, 120, 2.5, 1), map, gps};

    const types::LidarScanResult result = lidar.scan(Orientation{});

    ASSERT_EQ(result.size(), 1u);
    EXPECT_NEAR(result[0].distance.force_numerical_value_in(cm), 25.0, 1.5);
    EXPECT_GT(result[0].distance.force_numerical_value_in(cm), 0.0);
}