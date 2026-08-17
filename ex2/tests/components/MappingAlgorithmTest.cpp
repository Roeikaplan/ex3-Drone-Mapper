#include <drone_mapper/MappingAlgorithmImpl.h>

#include <drone_mapper/IMap3D.h>

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <map>

namespace {

using namespace drone_mapper;

/**
 * @brief Read-only map with programmable per-voxel occupancy (default `Unmapped`).
 *
 * Lets a test simulate the output map's state "after scans" and drive the planner deterministically,
 * without depending on `Map3DImpl`. Voxel indices are derived from a world position exactly as the
 * algorithm samples them (`floor((pos - offset) / resolution)`), so `setVoxel(i,j,k,...)` lines up
 * with the cell the algorithm queries at that voxel's centre.
 */
class GridMap : public IMap3D {
public:
    explicit GridMap(types::MapConfig config) : config_(config) {}

    void setVoxel(std::int64_t i, std::int64_t j, std::int64_t k, types::VoxelOccupancy occ) {
        cells_[{i, j, k}] = occ;
    }

    [[nodiscard]] types::VoxelOccupancy atVoxel(const Position3D& pos) const override {
        const double res = config_.resolution.force_numerical_value_in(cm);
        if (!(res > 0.0)) {
            return types::VoxelOccupancy::OutOfBounds;
        }
        const auto index = [res](double coord, double origin) {
            return static_cast<std::int64_t>(std::floor((coord - origin) / res));
        };
        const std::array<std::int64_t, 3> key{
            index(pos.x.force_numerical_value_in(cm), config_.offset.x.force_numerical_value_in(cm)),
            index(pos.y.force_numerical_value_in(cm), config_.offset.y.force_numerical_value_in(cm)),
            index(pos.z.force_numerical_value_in(cm), config_.offset.z.force_numerical_value_in(cm)),
        };
        const auto it = cells_.find(key);
        return it == cells_.end() ? types::VoxelOccupancy::Unmapped : it->second;
    }
    [[nodiscard]] types::MapConfig getMapConfig() const override {
        return config_;
    }
    [[nodiscard]] bool isInBounds(const Position3D&) const override {
        return true;
    }

private:
    types::MapConfig config_;
    std::map<std::array<std::int64_t, 3>, types::VoxelOccupancy> cells_;
};

/**
 * @brief A 3x1x1 grid of 10-cm voxels at the origin (voxel indices 0..2 along X).
 */
[[nodiscard]] types::MapConfig gridConfig() {
    const types::MappingBounds bounds{
        XLength{}, 30.0 * x_extent[cm], YLength{}, 10.0 * y_extent[cm],
        ZLength{}, 10.0 * z_extent[cm],
    };
    return types::MapConfig{bounds, Position3D{}, 10.0 * cm};
}

[[nodiscard]] types::DroneConfigData droneCfg() {
    return types::DroneConfigData{5.0 * cm, 90.0 * horizontal_angle[deg], 10.0 * cm, 10.0 * cm};
}

[[nodiscard]] types::LidarConfigData lidarCfg() {
    return types::LidarConfigData{20.0 * cm, 120.0 * cm, 2.5 * cm, 1};
}

/**
 * @brief A 1x1x3 grid of 10-cm voxels at the origin (voxel indices 0..2 along Z).
 */
[[nodiscard]] types::MapConfig verticalGridConfig() {
    const types::MappingBounds bounds{
        XLength{}, 10.0 * x_extent[cm], YLength{}, 10.0 * y_extent[cm],
        ZLength{}, 30.0 * z_extent[cm],
    };
    return types::MapConfig{bounds, Position3D{}, 10.0 * cm};
}

/**
 * @brief A drone state at a world position (cm), heading east (0 deg).
 */
[[nodiscard]] types::DroneState stateAt(double x, double y, double z) {
    types::DroneState state{};
    state.position = Position3D{x * x_extent[cm], y * y_extent[cm], z * z_extent[cm]};
    state.heading = Orientation{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]};
    return state;
}

/**
 * @brief A drone state at a world position (cm) with an explicit horizontal heading.
 */
[[nodiscard]] types::DroneState stateFacing(double x, double y, double z, double heading_deg) {
    types::DroneState state = stateAt(x, y, z);
    state.heading = Orientation{heading_deg * horizontal_angle[deg], 0.0 * altitude_angle[deg]};
    return state;
}

/**
 * @brief Drain `count` commands from the algorithm (used to skip the initial scan sweep).
 */
void drain(MappingAlgorithmImpl& algo, const types::DroneState& state, int count) {
    for (int i = 0; i < count; ++i) {
        (void)algo.nextStep(state, nullptr);
    }
}

} // namespace

/**
 * @brief The very first step scans the current cell (a scan command, no movement).
 */
TEST(MappingAlgorithm, FirstStepScansCurrentCell) {
    GridMap map{gridConfig()};
    MappingAlgorithmImpl algo{types::MissionConfigData{}, lidarCfg(), droneCfg(), map};

    const types::MappingStepCommand command = algo.nextStep(stateAt(5, 5, 5), nullptr);

    EXPECT_TRUE(command.scan_orientation.has_value());
    EXPECT_FALSE(command.movement.has_value());
    EXPECT_EQ(command.status, types::AlgorithmStatus::Working);
}

/**
 * @brief Surveying a fresh cell issues a six-direction scan sweep before any movement.
 */
TEST(MappingAlgorithm, IssuesSixScanSweep) {
    GridMap map{gridConfig()};
    MappingAlgorithmImpl algo{types::MissionConfigData{}, lidarCfg(), droneCfg(), map};
    const types::DroneState state = stateAt(5, 5, 5);

    for (int i = 0; i < 6; ++i) {
        const types::MappingStepCommand command = algo.nextStep(state, nullptr);
        EXPECT_TRUE(command.scan_orientation.has_value()) << "sweep command " << i;
        EXPECT_FALSE(command.movement.has_value()) << "sweep command " << i;
    }
}

/**
 * @brief After surveying, the planner heads to the nearest frontier with a limit-respecting move.
 *
 * Cells (0,0,0) and (1,0,0) are Empty and (2,0,0) is still Unmapped, so (1,0,0) is a frontier
 * reachable through Empty space; the first travel command is an Advance no larger than max_advance.
 */
TEST(MappingAlgorithm, PlansMovementTowardFrontier) {
    GridMap map{gridConfig()};
    map.setVoxel(0, 0, 0, types::VoxelOccupancy::Empty);
    map.setVoxel(1, 0, 0, types::VoxelOccupancy::Empty);
    // (2,0,0) left Unmapped -> makes (1,0,0) a frontier.
    MappingAlgorithmImpl algo{types::MissionConfigData{}, lidarCfg(), droneCfg(), map};
    const types::DroneState state = stateAt(5, 5, 5);

    drain(algo, state, 6); // skip the scan sweep
    const types::MappingStepCommand command = algo.nextStep(state, nullptr);

    ASSERT_TRUE(command.movement.has_value());
    EXPECT_EQ(command.movement->type, types::MovementCommandType::Advance);
    EXPECT_LE(command.movement->distance.force_numerical_value_in(cm),
              droneCfg().max_advance.force_numerical_value_in(cm) + 1e-9);
}

/**
 * @brief With every in-bounds cell mapped, exploration ends cleanly with Finished.
 */
TEST(MappingAlgorithm, FinishedWhenAllMapped) {
    GridMap map{gridConfig()};
    map.setVoxel(0, 0, 0, types::VoxelOccupancy::Empty);
    map.setVoxel(1, 0, 0, types::VoxelOccupancy::Empty);
    map.setVoxel(2, 0, 0, types::VoxelOccupancy::Empty);
    MappingAlgorithmImpl algo{types::MissionConfigData{}, lidarCfg(), droneCfg(), map};
    const types::DroneState state = stateAt(5, 5, 5);

    drain(algo, state, 6);
    const types::MappingStepCommand command = algo.nextStep(state, nullptr);

    EXPECT_EQ(command.status, types::AlgorithmStatus::Finished);
    EXPECT_FALSE(command.movement.has_value());
    EXPECT_FALSE(command.scan_orientation.has_value());
}

/**
 * @brief An Unmapped cell reachable only through Occupied space ends with FinishedWithUnmappableVoxels.
 */
TEST(MappingAlgorithm, FinishedWithUnmappableWhenWalledOff) {
    GridMap map{gridConfig()};
    map.setVoxel(0, 0, 0, types::VoxelOccupancy::Empty);
    map.setVoxel(1, 0, 0, types::VoxelOccupancy::Occupied); // wall
    // (2,0,0) stays Unmapped and unreachable through Empty space.
    MappingAlgorithmImpl algo{types::MissionConfigData{}, lidarCfg(), droneCfg(), map};
    const types::DroneState state = stateAt(5, 5, 5);

    drain(algo, state, 6);
    const types::MappingStepCommand command = algo.nextStep(state, nullptr);

    EXPECT_EQ(command.status, types::AlgorithmStatus::FinishedWithUnmappableVoxels);
}

/**
 * @brief Once exploration is finished, further calls keep reporting Finished.
 */
TEST(MappingAlgorithm, FinishedLatches) {
    GridMap map{gridConfig()};
    map.setVoxel(0, 0, 0, types::VoxelOccupancy::Empty);
    map.setVoxel(1, 0, 0, types::VoxelOccupancy::Empty);
    map.setVoxel(2, 0, 0, types::VoxelOccupancy::Empty);
    MappingAlgorithmImpl algo{types::MissionConfigData{}, lidarCfg(), droneCfg(), map};
    const types::DroneState state = stateAt(5, 5, 5);

    drain(algo, state, 6);
    EXPECT_EQ(algo.nextStep(state, nullptr).status, types::AlgorithmStatus::Finished);
    EXPECT_EQ(algo.nextStep(state, nullptr).status, types::AlgorithmStatus::Finished);
}

/**
 * @brief A drone positioned outside the mappable grid finishes immediately.
 */
TEST(MappingAlgorithm, DroneOutsideGridFinishes) {
    GridMap map{gridConfig()};
    MappingAlgorithmImpl algo{types::MissionConfigData{}, lidarCfg(), droneCfg(), map};

    const types::MappingStepCommand command = algo.nextStep(stateAt(1000, 5, 5), nullptr);

    EXPECT_EQ(command.status, types::AlgorithmStatus::Finished);
}

/**
 * @brief The strategy is deterministic: identical inputs yield the identical first command.
 */
TEST(MappingAlgorithm, Deterministic) {
    GridMap map_a{gridConfig()};
    GridMap map_b{gridConfig()};
    MappingAlgorithmImpl algo_a{types::MissionConfigData{}, lidarCfg(), droneCfg(), map_a};
    MappingAlgorithmImpl algo_b{types::MissionConfigData{}, lidarCfg(), droneCfg(), map_b};
    const types::DroneState state = stateAt(5, 5, 5);

    const types::MappingStepCommand a = algo_a.nextStep(state, nullptr);
    const types::MappingStepCommand b = algo_b.nextStep(state, nullptr);

    ASSERT_TRUE(a.scan_orientation.has_value());
    ASSERT_TRUE(b.scan_orientation.has_value());
    EXPECT_DOUBLE_EQ(a.scan_orientation->horizontal.force_numerical_value_in(deg),
                     b.scan_orientation->horizontal.force_numerical_value_in(deg));
    EXPECT_DOUBLE_EQ(a.scan_orientation->altitude.force_numerical_value_in(deg),
                     b.scan_orientation->altitude.force_numerical_value_in(deg));
}

/**
 * @brief A turn larger than max_rotate is split into limit-respecting rotation micro-steps.
 *
 * The frontier lies at -X of the drone (desired heading 180 from heading 0) and max_rotate is 45:
 * the planner must emit four 45-deg rotations before the advance, never one 180-deg command.
 */
TEST(MappingAlgorithm, SplitsRotationByMaxRotate) {
    GridMap map{gridConfig()};
    map.setVoxel(1, 0, 0, types::VoxelOccupancy::Empty); // frontier: unscanned, neighbour (0,0,0) Unmapped
    map.setVoxel(2, 0, 0, types::VoxelOccupancy::Empty); // the drone's own (scanned) cell
    const types::DroneConfigData tight_turner{5.0 * cm, 45.0 * horizontal_angle[deg], 10.0 * cm,
                                              10.0 * cm};
    MappingAlgorithmImpl algo{types::MissionConfigData{}, lidarCfg(), tight_turner, map};
    const types::DroneState state = stateAt(25, 5, 5); // cell (2,0,0), heading 0

    drain(algo, state, 6); // skip the current cell's scan sweep

    for (int i = 0; i < 4; ++i) {
        const types::MappingStepCommand command = algo.nextStep(state, nullptr);
        ASSERT_TRUE(command.movement.has_value()) << "rotation micro-step " << i;
        EXPECT_EQ(command.movement->type, types::MovementCommandType::Rotate) << i;
        EXPECT_NEAR(command.movement->angle.force_numerical_value_in(deg), 45.0, 1e-9) << i;
    }
    const types::MappingStepCommand after = algo.nextStep(state, nullptr);
    ASSERT_TRUE(after.movement.has_value());
    EXPECT_EQ(after.movement->type, types::MovementCommandType::Advance); // then the travel begins
}

/**
 * @brief A one-voxel advance longer than max_advance is split into limit-respecting chunks.
 *
 * Voxels are 10 cm and max_advance is 5: travelling to the +X frontier must take two 5-cm advances.
 */
TEST(MappingAlgorithm, SplitsAdvanceByMaxAdvance) {
    GridMap map{gridConfig()};
    map.setVoxel(0, 0, 0, types::VoxelOccupancy::Empty); // the drone's own (scanned) cell
    map.setVoxel(1, 0, 0, types::VoxelOccupancy::Empty); // frontier: neighbour (2,0,0) Unmapped
    const types::DroneConfigData short_strider{5.0 * cm, 90.0 * horizontal_angle[deg], 5.0 * cm,
                                               10.0 * cm};
    MappingAlgorithmImpl algo{types::MissionConfigData{}, lidarCfg(), short_strider, map};
    const types::DroneState state = stateAt(5, 5, 5); // cell (0,0,0), already facing +X

    drain(algo, state, 6);

    for (int i = 0; i < 2; ++i) {
        const types::MappingStepCommand command = algo.nextStep(state, nullptr);
        ASSERT_TRUE(command.movement.has_value()) << "advance micro-step " << i;
        EXPECT_EQ(command.movement->type, types::MovementCommandType::Advance) << i;
        EXPECT_NEAR(command.movement->distance.force_numerical_value_in(cm), 5.0, 1e-9) << i;
    }
}

/**
 * @brief A frontier directly above is reached with an Elevate command (within max_elevate).
 */
TEST(MappingAlgorithm, ElevatesTowardVerticalFrontier) {
    GridMap map{verticalGridConfig()};
    map.setVoxel(0, 0, 0, types::VoxelOccupancy::Empty); // the drone's own (scanned) cell
    map.setVoxel(0, 0, 1, types::VoxelOccupancy::Empty); // frontier: neighbour (0,0,2) Unmapped
    MappingAlgorithmImpl algo{types::MissionConfigData{}, lidarCfg(), droneCfg(), map};
    const types::DroneState state = stateAt(5, 5, 5); // cell (0,0,0)

    drain(algo, state, 6);
    const types::MappingStepCommand command = algo.nextStep(state, nullptr);

    ASSERT_TRUE(command.movement.has_value());
    EXPECT_EQ(command.movement->type, types::MovementCommandType::Elevate);
    EXPECT_NEAR(command.movement->distance.force_numerical_value_in(cm), 10.0, 1e-9); // one voxel up
    EXPECT_LE(command.movement->distance.force_numerical_value_in(cm),
              droneCfg().max_elevate.force_numerical_value_in(cm) + 1e-9);
}

/**
 * @brief Sweep scans are heading-RELATIVE: at heading 90 the +X world scan is requested as -90.
 *
 * MockLidar composes the scan orientation with the sensor heading, so the algorithm must subtract
 * the heading when it wants a fixed world direction.
 */
TEST(MappingAlgorithm, SweepOrientationsAreHeadingRelative) {
    GridMap map{gridConfig()};
    MappingAlgorithmImpl algo{types::MissionConfigData{}, lidarCfg(), droneCfg(), map};
    const types::DroneState state = stateFacing(5, 5, 5, 90.0);

    const types::MappingStepCommand first = algo.nextStep(state, nullptr);

    ASSERT_TRUE(first.scan_orientation.has_value());
    // First sweep direction is +X (world 0 deg): relative request = 0 - 90 = -90.
    EXPECT_DOUBLE_EQ(first.scan_orientation->horizontal.force_numerical_value_in(deg), -90.0);
    EXPECT_DOUBLE_EQ(first.scan_orientation->altitude.force_numerical_value_in(deg), 0.0);
}

/**
 * @brief A zero-resolution map is degenerate geometry: the algorithm finishes immediately.
 */
TEST(MappingAlgorithm, ZeroResolutionFinishesImmediately) {
    GridMap map{types::MapConfig{types::MappingBounds{}, Position3D{}, 0.0 * cm}};
    MappingAlgorithmImpl algo{types::MissionConfigData{}, lidarCfg(), droneCfg(), map};

    const types::MappingStepCommand command = algo.nextStep(stateAt(5, 5, 5), nullptr);

    EXPECT_EQ(command.status, types::AlgorithmStatus::Finished);
    EXPECT_FALSE(command.movement.has_value());
    EXPECT_FALSE(command.scan_orientation.has_value());
}

/**
 * @brief The survey sweep covers all six axis directions (4 compass points, up, and down).
 *
 * At heading 0 the relative and world directions coincide, so the six scan orientations must be
 * exactly horizontals {0, 180, 90, 270, 0, 0} with altitudes {0, 0, 0, 0, 90, -90}.
 */
TEST(MappingAlgorithm, SweepCoversSixWorldDirections) {
    GridMap map{gridConfig()};
    MappingAlgorithmImpl algo{types::MissionConfigData{}, lidarCfg(), droneCfg(), map};
    const types::DroneState state = stateAt(5, 5, 5);

    constexpr std::array<double, 6> expected_horizontal{0.0, 180.0, 90.0, 270.0, 0.0, 0.0};
    constexpr std::array<double, 6> expected_altitude{0.0, 0.0, 0.0, 0.0, 90.0, -90.0};
    for (std::size_t i = 0; i < 6; ++i) {
        const types::MappingStepCommand command = algo.nextStep(state, nullptr);
        ASSERT_TRUE(command.scan_orientation.has_value()) << "sweep command " << i;
        EXPECT_DOUBLE_EQ(command.scan_orientation->horizontal.force_numerical_value_in(deg),
                         expected_horizontal[i])
            << i;
        EXPECT_DOUBLE_EQ(command.scan_orientation->altitude.force_numerical_value_in(deg),
                         expected_altitude[i])
            << i;
    }
}