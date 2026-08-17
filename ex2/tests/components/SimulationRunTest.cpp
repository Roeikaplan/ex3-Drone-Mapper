#include <drone_mapper/IDroneControl.h>
#include <drone_mapper/IDroneMovement.h>
#include <drone_mapper/IGPS.h>
#include <drone_mapper/ILidar.h>
#include <drone_mapper/IMappingAlgorithm.h>
#include <drone_mapper/IMissionControl.h>
#include <drone_mapper/IMutableMap3D.h>
#include <drone_mapper/MockGPS.h>
#include <drone_mapper/MockMovement.h>
#include <drone_mapper/SimulationRunImpl.h>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cmath>
#include <filesystem>
#include <functional>
#include <memory>
#include <utility>

// This file is the `SimulationRun` component suite. It also covers MockGPS and
// MockMovement, so every TEST here uses the `SimulationRun` suite name (a graded
// --gtest_filter=SimulationRun.* run therefore exercises all of it). It is organised into three
// sections:
//   A. run() result assembly (SimulationRunImpl::run)
//   B. MockGPS behaviour
//   C. MockMovement behaviour

namespace {

using namespace drone_mapper;
using ::testing::Return;

// gmock IMissionControl so each run() test scripts the mission outcome. run() is the only
// collaborator that drives behaviour; everything else is inert.
class MockMissionControl : public IMissionControl {
public:
    MOCK_METHOD(types::MissionRunResult, runMission, (), (override));
};

/**
 * @brief Map with uniform occupancy and a caller-supplied `MapConfig`.
 *
 * Lets a test dial in a known comparison grid and IoU without depending on `Map3DImpl`, so a bug
 * injected into `Map3DImpl` cannot fail this suite.
 */
class FakeMap : public IMutableMap3D {
public:
    FakeMap(types::MapConfig config, bool occupied_everywhere)
        : config_(config), occupied_(occupied_everywhere) {}

    [[nodiscard]] types::VoxelOccupancy atVoxel(const Position3D&) const override {
        return occupied_ ? types::VoxelOccupancy::Occupied : types::VoxelOccupancy::Unmapped;
    }
    [[nodiscard]] types::MapConfig getMapConfig() const override {
        return config_;
    }
    [[nodiscard]] bool isInBounds(const Position3D&) const override {
        return true;
    }
    void set(const Position3D&, types::VoxelOccupancy) override {}
    void save(const std::filesystem::path&) const override {}

private:
    types::MapConfig config_;
    bool occupied_;
};

/**
 * @brief Map whose occupancy is decided per-position by a predicate — used to craft a partial IoU.
 */
class PredicateMap : public IMutableMap3D {
public:
    PredicateMap(types::MapConfig config, std::function<bool(const Position3D&)> occupied)
        : config_(config), occupied_(std::move(occupied)) {}

    [[nodiscard]] types::VoxelOccupancy atVoxel(const Position3D& pos) const override {
        return occupied_(pos) ? types::VoxelOccupancy::Occupied : types::VoxelOccupancy::Unmapped;
    }
    [[nodiscard]] types::MapConfig getMapConfig() const override {
        return config_;
    }
    [[nodiscard]] bool isInBounds(const Position3D&) const override {
        return true;
    }
    void set(const Position3D&, types::VoxelOccupancy) override {}
    void save(const std::filesystem::path&) const override {}

private:
    types::MapConfig config_;
    std::function<bool(const Position3D&)> occupied_;
};

// Inert stubs for the components run() never touches; the SimulationRunImpl ctor only requires them
// to be non-null.
class StubGPS : public IGPS {
public:
    [[nodiscard]] Position3D position() const override {
        return {};
    }
    [[nodiscard]] Orientation heading() const override {
        return {};
    }
};

class StubMovement : public IDroneMovement {
public:
    types::MovementResult rotate(types::RotationDirection, HorizontalAngle) override {
        return {};
    }
    types::MovementResult advance(PhysicalLength) override {
        return {};
    }
    types::MovementResult elevate(PhysicalLength) override {
        return {};
    }
};

class StubLidar : public ILidar {
public:
    [[nodiscard]] types::LidarScanResult scan(Orientation) const override {
        return {};
    }
    [[nodiscard]] types::LidarConfigData config() const override {
        return {};
    }
};

class StubAlgorithm : public IMappingAlgorithm {
public:
    // Forward the interface's required constructor; the referenced map is never dereferenced here.
    StubAlgorithm(const types::MissionConfigData& mission, const types::LidarConfigData& lidar,
                  const types::DroneConfigData& drone, const IMap3D& output_map)
        : IMappingAlgorithm(mission, lidar, drone, output_map) {}

    [[nodiscard]] types::MappingStepCommand nextStep(const types::DroneState&,
                                                     const types::LidarScanResult*) override {
        return {};
    }
};

class StubDroneControl : public IDroneControl {
public:
    [[nodiscard]] types::DroneStepResult step() override {
        return {};
    }
    [[nodiscard]] types::DroneState state() const override {
        return {};
    }
};

/**
 * @brief A 1x1x1-cm comparison grid anchored at the origin.
 * @return A `MapConfig` with a single voxel, so `MapsComparison` walks exactly one cell.
 */
[[nodiscard]] types::MapConfig oneVoxelConfig() {
    const types::MappingBounds bounds{
        XLength{}, 1.0 * x_extent[cm],
        YLength{}, 1.0 * y_extent[cm],
        ZLength{}, 1.0 * z_extent[cm],
    };
    return types::MapConfig{bounds, Position3D{}, 1.0 * cm};
}

/**
 * @brief A 2x1x1-cm grid (two cells along X), for a controllable partial IoU.
 * @return A `MapConfig` whose X span holds two 1-cm voxels.
 */
[[nodiscard]] types::MapConfig twoVoxelXConfig() {
    const types::MappingBounds bounds{
        XLength{}, 2.0 * x_extent[cm],
        YLength{}, 1.0 * y_extent[cm],
        ZLength{}, 1.0 * z_extent[cm],
    };
    return types::MapConfig{bounds, Position3D{}, 1.0 * cm};
}

/**
 * @brief Assemble a `SimulationRunImpl` with the given mission/maps and inert stubs for the rest.
 * @param mission Mock mission controller driving the run outcome.
 * @param hidden Ground-truth map (comparison origin).
 * @param output Writable output map (comparison target; its config is echoed into the result).
 * @param sim Simulation config echoed into the result.
 * @param mission_cfg Mission config echoed into the result (and handed to the stub algorithm).
 * @param status Resolution status echoed into the result.
 * @param output_file Output-map path echoed into the result.
 * @return A ready-to-run `SimulationRunImpl`.
 * @note The stub algorithm binds a reference to `output` before it is moved in, so the reference
 *       stays valid for the run's lifetime — mirroring the real wiring.
 */
[[nodiscard]] SimulationRunImpl makeRun(std::unique_ptr<IMissionControl> mission,
                                        std::unique_ptr<const IMap3D> hidden,
                                        std::unique_ptr<IMutableMap3D> output,
                                        types::SimulationConfigData sim,
                                        types::MissionConfigData mission_cfg,
                                        types::ResolutionRequestStatus status,
                                        std::filesystem::path output_file) {
    IMutableMap3D& output_ref = *output;
    auto gps = std::make_unique<StubGPS>();
    auto movement = std::make_unique<StubMovement>();
    auto lidar = std::make_unique<StubLidar>();
    auto algorithm = std::make_unique<StubAlgorithm>(mission_cfg, types::LidarConfigData{},
                                                     types::DroneConfigData{}, output_ref);
    auto drone_control = std::make_unique<StubDroneControl>();

    return SimulationRunImpl{
        std::move(hidden),      std::move(output),      std::move(gps),
        std::move(movement),    std::move(lidar),       std::move(algorithm),
        std::move(drone_control), std::move(mission),   std::move(sim),
        std::move(mission_cfg), status,                 std::move(output_file)};
}

/**
 * @brief Build a completed MissionRunResult with the given step count.
 * @param steps Step count to report.
 * @return A `Completed` mission result.
 */
[[nodiscard]] types::MissionRunResult completedMission(std::size_t steps) {
    types::MissionRunResult result{};
    result.status = types::MissionRunStatus::Completed;
    result.steps = steps;
    return result;
}

} // namespace

// ============================================================================
// Section A — run() result assembly (SimulationRunImpl::run)
// ============================================================================

/**
 * @brief A completed mission is scored and every field of the result is populated.
 *
 * With one shared Occupied voxel the IoU is 100; the returned result must also echo the mission
 * result, both configs, the resolution status, the output path, and the output map's config.
 */
TEST(SimulationRun, AssemblesResultFromMissionAndScore) {
    auto mission = std::make_unique<MockMissionControl>();
    EXPECT_CALL(*mission, runMission()).WillOnce(Return(completedMission(3)));

    auto hidden = std::make_unique<FakeMap>(oneVoxelConfig(), /*occupied_everywhere=*/true);
    // Distinct output resolution (5 cm) so we can assert it is echoed back verbatim.
    const types::MapConfig output_config{types::MappingBounds{}, Position3D{}, 5.0 * cm};
    auto output = std::make_unique<FakeMap>(output_config, /*occupied_everywhere=*/true);

    types::SimulationConfigData sim{};
    sim.map_filename = "hidden.npy";
    types::MissionConfigData mission_cfg{};
    mission_cfg.max_steps = 42;

    SimulationRunImpl run = makeRun(std::move(mission), std::move(hidden), std::move(output),
                                    sim, mission_cfg, types::ResolutionRequestStatus::Accepted,
                                    "output_results/hidden_run0.npy");

    const types::SimulationResult result = run.run();

    EXPECT_DOUBLE_EQ(result.mission_score, 100.0);
    ASSERT_EQ(result.mission_results.size(), 1u);
    EXPECT_EQ(result.mission_results[0].status, types::MissionRunStatus::Completed);
    EXPECT_EQ(result.mission_results[0].steps, 3u);
    EXPECT_EQ(result.simulation_config.map_filename, "hidden.npy");
    EXPECT_EQ(result.mission_config.max_steps, 42u);
    EXPECT_EQ(result.resolution_request_status, types::ResolutionRequestStatus::Accepted);
    EXPECT_EQ(result.output_map_file, std::filesystem::path{"output_results/hidden_run0.npy"});
    EXPECT_DOUBLE_EQ(result.output_map_config.resolution.force_numerical_value_in(cm), 5.0);
}

/**
 * @brief A disjoint output scores the IoU floor (0), proving the score flows from the comparison.
 */
TEST(SimulationRun, ScoresIoUFloorFromMaps) {
    auto mission = std::make_unique<MockMissionControl>();
    EXPECT_CALL(*mission, runMission()).WillOnce(Return(completedMission(1)));

    auto hidden = std::make_unique<FakeMap>(oneVoxelConfig(), /*occupied_everywhere=*/true);
    auto output = std::make_unique<FakeMap>(oneVoxelConfig(), /*occupied_everywhere=*/false);

    SimulationRunImpl run = makeRun(std::move(mission), std::move(hidden), std::move(output),
                                    types::SimulationConfigData{}, types::MissionConfigData{},
                                    types::ResolutionRequestStatus::Accepted, "out.npy");

    EXPECT_DOUBLE_EQ(run.run().mission_score, 0.0);
}

/**
 * @brief A half-overlapping output scores 50, proving a non-trivial IoU is plumbed through.
 *
 * On a two-voxel grid the hidden map is Occupied in both cells and the output only in the first, so
 * intersection/union = 1/2 = 50%.
 */
TEST(SimulationRun, ScoresPartialIoU) {
    auto mission = std::make_unique<MockMissionControl>();
    EXPECT_CALL(*mission, runMission()).WillOnce(Return(completedMission(1)));

    auto hidden = std::make_unique<FakeMap>(twoVoxelXConfig(), /*occupied_everywhere=*/true);
    auto output = std::make_unique<PredicateMap>(twoVoxelXConfig(), [](const Position3D& p) {
        return p.x.force_numerical_value_in(cm) < 1.0; // only the first cell
    });

    SimulationRunImpl run = makeRun(std::move(mission), std::move(hidden), std::move(output),
                                    types::SimulationConfigData{}, types::MissionConfigData{},
                                    types::ResolutionRequestStatus::Accepted, "out.npy");

    EXPECT_NEAR(run.run().mission_score, 50.0, 1e-9);
}

/**
 * @brief An errored mission short-circuits scoring and reports the -1 error sentinel.
 *
 * Both maps are Occupied (they would score 100 if compared), so a -1 result proves the mission
 * error skips comparison entirely.
 */
TEST(SimulationRun, MissionErrorScoresMinusOne) {
    auto mission = std::make_unique<MockMissionControl>();
    types::MissionRunResult mission_result{};
    mission_result.status = types::MissionRunStatus::Error;
    EXPECT_CALL(*mission, runMission()).WillOnce(Return(mission_result));

    auto hidden = std::make_unique<FakeMap>(oneVoxelConfig(), /*occupied_everywhere=*/true);
    auto output = std::make_unique<FakeMap>(oneVoxelConfig(), /*occupied_everywhere=*/true);

    SimulationRunImpl run = makeRun(std::move(mission), std::move(hidden), std::move(output),
                                    types::SimulationConfigData{}, types::MissionConfigData{},
                                    types::ResolutionRequestStatus::Accepted, "out.npy");

    const types::SimulationResult result = run.run();
    EXPECT_DOUBLE_EQ(result.mission_score, -1.0);
    ASSERT_EQ(result.mission_results.size(), 1u);
    EXPECT_EQ(result.mission_results[0].status, types::MissionRunStatus::Error);
}

/**
 * @brief run() drives the mission exactly once.
 */
TEST(SimulationRun, CallsRunMissionOnce) {
    auto mission = std::make_unique<MockMissionControl>();
    EXPECT_CALL(*mission, runMission()).Times(1).WillOnce(Return(completedMission(1)));

    auto hidden = std::make_unique<FakeMap>(oneVoxelConfig(), /*occupied_everywhere=*/false);
    auto output = std::make_unique<FakeMap>(oneVoxelConfig(), /*occupied_everywhere=*/false);

    SimulationRunImpl run = makeRun(std::move(mission), std::move(hidden), std::move(output),
                                    types::SimulationConfigData{}, types::MissionConfigData{},
                                    types::ResolutionRequestStatus::Accepted, "out.npy");
    (void)run.run();
}

/**
 * @brief The factory-decided resolution status is echoed verbatim (not hard-coded).
 */
TEST(SimulationRun, ResolutionStatusEchoed) {
    auto mission = std::make_unique<MockMissionControl>();
    EXPECT_CALL(*mission, runMission()).WillOnce(Return(completedMission(1)));

    auto hidden = std::make_unique<FakeMap>(oneVoxelConfig(), /*occupied_everywhere=*/false);
    auto output = std::make_unique<FakeMap>(oneVoxelConfig(), /*occupied_everywhere=*/false);

    SimulationRunImpl run = makeRun(std::move(mission), std::move(hidden), std::move(output),
                                    types::SimulationConfigData{}, types::MissionConfigData{},
                                    types::ResolutionRequestStatus::Ignored, "out.npy");

    EXPECT_EQ(run.run().resolution_request_status, types::ResolutionRequestStatus::Ignored);
}

/**
 * @brief The output map's full geometry (offset, not just resolution) is echoed into the result.
 */
TEST(SimulationRun, EchoesOutputMapConfigOffset) {
    auto mission = std::make_unique<MockMissionControl>();
    EXPECT_CALL(*mission, runMission()).WillOnce(Return(completedMission(1)));

    auto hidden = std::make_unique<FakeMap>(oneVoxelConfig(), /*occupied_everywhere=*/false);
    const types::MapConfig output_config{
        types::MappingBounds{}, Position3D{1.0 * x_extent[cm], 2.0 * y_extent[cm], 3.0 * z_extent[cm]},
        5.0 * cm};
    auto output = std::make_unique<FakeMap>(output_config, /*occupied_everywhere=*/false);

    SimulationRunImpl run = makeRun(std::move(mission), std::move(hidden), std::move(output),
                                    types::SimulationConfigData{}, types::MissionConfigData{},
                                    types::ResolutionRequestStatus::Accepted, "out.npy");

    const types::SimulationResult result = run.run();
    EXPECT_DOUBLE_EQ(result.output_map_config.offset.x.force_numerical_value_in(cm), 1.0);
    EXPECT_DOUBLE_EQ(result.output_map_config.offset.y.force_numerical_value_in(cm), 2.0);
    EXPECT_DOUBLE_EQ(result.output_map_config.offset.z.force_numerical_value_in(cm), 3.0);
}

/**
 * @brief A MaxSteps mission is still scored — only an Error mission takes the -1 sentinel.
 *
 * Identical occupied maps would score 100; a MaxSteps outcome must not be treated like an error.
 */
TEST(SimulationRun, MaxStepsMissionStillScored) {
    auto mission = std::make_unique<MockMissionControl>();
    types::MissionRunResult mission_result{};
    mission_result.status = types::MissionRunStatus::MaxSteps;
    mission_result.steps = 9;
    EXPECT_CALL(*mission, runMission()).WillOnce(Return(mission_result));

    auto hidden = std::make_unique<FakeMap>(oneVoxelConfig(), /*occupied_everywhere=*/true);
    auto output = std::make_unique<FakeMap>(oneVoxelConfig(), /*occupied_everywhere=*/true);

    SimulationRunImpl run = makeRun(std::move(mission), std::move(hidden), std::move(output),
                                    types::SimulationConfigData{}, types::MissionConfigData{},
                                    types::ResolutionRequestStatus::Accepted, "out.npy");

    const types::SimulationResult result = run.run();
    EXPECT_DOUBLE_EQ(result.mission_score, 100.0);
    EXPECT_EQ(result.mission_results.front().status, types::MissionRunStatus::MaxSteps);
}

/**
 * @brief Mission-level errors are propagated verbatim (codes and messages) into the result.
 */
TEST(SimulationRun, MissionErrorsPropagatedVerbatim) {
    auto mission = std::make_unique<MockMissionControl>();
    types::MissionRunResult mission_result{};
    mission_result.status = types::MissionRunStatus::Error;
    mission_result.errors = {types::ErrorRef{"DRONE_STEP_ERROR", "clipped a wall"},
                             types::ErrorRef{"SECOND_CODE", "extra detail"}};
    EXPECT_CALL(*mission, runMission()).WillOnce(Return(mission_result));

    auto hidden = std::make_unique<FakeMap>(oneVoxelConfig(), /*occupied_everywhere=*/false);
    auto output = std::make_unique<FakeMap>(oneVoxelConfig(), /*occupied_everywhere=*/false);

    SimulationRunImpl run = makeRun(std::move(mission), std::move(hidden), std::move(output),
                                    types::SimulationConfigData{}, types::MissionConfigData{},
                                    types::ResolutionRequestStatus::Accepted, "out.npy");

    const types::SimulationResult result = run.run();
    ASSERT_EQ(result.mission_results.size(), 1u);
    ASSERT_EQ(result.mission_results[0].errors.size(), 2u);
    EXPECT_EQ(result.mission_results[0].errors[0].code, "DRONE_STEP_ERROR");
    EXPECT_EQ(result.mission_results[0].errors[0].message, "clipped a wall");
    EXPECT_EQ(result.mission_results[0].errors[1].code, "SECOND_CODE");
}

/**
 * @brief A zero-resolution hidden map yields the comparison floor (0), not a crash or -1.
 *
 * MapsComparison cannot walk a grid without a positive resolution; the run must still assemble a
 * result with the floor score rather than dividing by zero or reporting the error sentinel.
 */
TEST(SimulationRun, ZeroResolutionOriginScoresFloor) {
    auto mission = std::make_unique<MockMissionControl>();
    EXPECT_CALL(*mission, runMission()).WillOnce(Return(completedMission(1)));

    const types::MapConfig degenerate{types::MappingBounds{}, Position3D{}, 0.0 * cm};
    auto hidden = std::make_unique<FakeMap>(degenerate, /*occupied_everywhere=*/true);
    auto output = std::make_unique<FakeMap>(degenerate, /*occupied_everywhere=*/true);

    SimulationRunImpl run = makeRun(std::move(mission), std::move(hidden), std::move(output),
                                    types::SimulationConfigData{}, types::MissionConfigData{},
                                    types::ResolutionRequestStatus::Accepted, "out.npy");

    EXPECT_DOUBLE_EQ(run.run().mission_score, 0.0);
}

// ============================================================================
// Section B — MockGPS behaviour (bundled into the SimulationRun suite per pc:162)
// ============================================================================

/**
 * @brief MockGPS reports the exact injected pose, including the altitude angle.
 */
TEST(SimulationRun, Gps_ReportsInjectedPoseExactly) {
    const Position3D pos{1.0 * x_extent[cm], 2.0 * y_extent[cm], 3.0 * z_extent[cm]};
    const Orientation heading{90.0 * horizontal_angle[deg], 15.0 * altitude_angle[deg]};
    MockGPS gps{pos, heading, 10.0 * cm};

    EXPECT_DOUBLE_EQ(gps.position().x.force_numerical_value_in(cm), 1.0);
    EXPECT_DOUBLE_EQ(gps.position().y.force_numerical_value_in(cm), 2.0);
    EXPECT_DOUBLE_EQ(gps.position().z.force_numerical_value_in(cm), 3.0);
    EXPECT_DOUBLE_EQ(gps.heading().horizontal.force_numerical_value_in(deg), 90.0);
    EXPECT_DOUBLE_EQ(gps.heading().altitude.force_numerical_value_in(deg), 15.0);
}

/**
 * @brief setPosition overwrites the reported position.
 */
TEST(SimulationRun, Gps_SetPositionUpdates) {
    MockGPS gps{Position3D{}, Orientation{}, 1.0 * cm};
    gps.setPosition(Position3D{5.0 * x_extent[cm], 6.0 * y_extent[cm], 7.0 * z_extent[cm]});

    EXPECT_DOUBLE_EQ(gps.position().x.force_numerical_value_in(cm), 5.0);
    EXPECT_DOUBLE_EQ(gps.position().y.force_numerical_value_in(cm), 6.0);
    EXPECT_DOUBLE_EQ(gps.position().z.force_numerical_value_in(cm), 7.0);
}

/**
 * @brief setHeading overwrites the reported heading.
 */
TEST(SimulationRun, Gps_SetHeadingUpdates) {
    MockGPS gps{Position3D{}, Orientation{}, 1.0 * cm};
    gps.setHeading(Orientation{45.0 * horizontal_angle[deg], 10.0 * altitude_angle[deg]});

    EXPECT_DOUBLE_EQ(gps.heading().horizontal.force_numerical_value_in(deg), 45.0);
    EXPECT_DOUBLE_EQ(gps.heading().altitude.force_numerical_value_in(deg), 10.0);
}

/**
 * @brief MockGPS reports the true position and does NOT quantize to the GPS resolution.
 *
 * A 3.7 cm position with a 10 cm resolution must read back as exactly 3.7 — the mock deliberately
 * bypasses hardware quantization.
 */
TEST(SimulationRun, Gps_DoesNotQuantizeToResolution) {
    MockGPS gps{Position3D{3.7 * x_extent[cm], 0.0 * y_extent[cm], 0.0 * z_extent[cm]},
                Orientation{}, 10.0 * cm};
    EXPECT_DOUBLE_EQ(gps.position().x.force_numerical_value_in(cm), 3.7);
}

// ============================================================================
// Section C — MockMovement behaviour (bundled into the SimulationRun suite per pc:162)
// ============================================================================

/**
 * @brief Advance at heading 0 moves +X only (angle convention 0 deg = +X east).
 */
TEST(SimulationRun, Movement_AdvanceEastAtHeading0) {
    MockGPS gps{Position3D{}, Orientation{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]},
                1.0 * cm};
    MockMovement movement{gps};

    EXPECT_TRUE(static_cast<bool>(movement.advance(5.0 * cm)));
    EXPECT_NEAR(gps.position().x.force_numerical_value_in(cm), 5.0, 1e-9);
    EXPECT_NEAR(gps.position().y.force_numerical_value_in(cm), 0.0, 1e-9);
    EXPECT_NEAR(gps.position().z.force_numerical_value_in(cm), 0.0, 1e-9);
}

/**
 * @brief Advance at heading 90 moves +Y only (angle convention 90 deg = +Y south).
 */
TEST(SimulationRun, Movement_AdvanceSouthAtHeading90) {
    MockGPS gps{Position3D{}, Orientation{90.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]},
                1.0 * cm};
    MockMovement movement{gps};

    movement.advance(5.0 * cm);
    EXPECT_NEAR(gps.position().x.force_numerical_value_in(cm), 0.0, 1e-9);
    EXPECT_NEAR(gps.position().y.force_numerical_value_in(cm), 5.0, 1e-9);
}

/**
 * @brief A negative advance travels backward along the heading.
 */
TEST(SimulationRun, Movement_AdvanceNegative) {
    MockGPS gps{Position3D{}, Orientation{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]},
                1.0 * cm};
    MockMovement movement{gps};

    movement.advance(-5.0 * cm);
    EXPECT_NEAR(gps.position().x.force_numerical_value_in(cm), -5.0, 1e-9);
}

/**
 * @brief Elevate changes Z only (up and down); X/Y are untouched.
 */
TEST(SimulationRun, Movement_ElevateChangesZOnly) {
    MockGPS gps{Position3D{2.0 * x_extent[cm], 3.0 * y_extent[cm], 0.0 * z_extent[cm]},
                Orientation{}, 1.0 * cm};
    MockMovement movement{gps};

    movement.elevate(5.0 * cm);
    EXPECT_NEAR(gps.position().z.force_numerical_value_in(cm), 5.0, 1e-9);
    movement.elevate(-9.0 * cm);
    EXPECT_NEAR(gps.position().z.force_numerical_value_in(cm), -4.0, 1e-9);
    EXPECT_NEAR(gps.position().x.force_numerical_value_in(cm), 2.0, 1e-9);
    EXPECT_NEAR(gps.position().y.force_numerical_value_in(cm), 3.0, 1e-9);
}

/**
 * @brief Rotate Left adds to the heading; altitude is unchanged.
 */
TEST(SimulationRun, Movement_RotateLeftAdds) {
    MockGPS gps{Position3D{}, Orientation{30.0 * horizontal_angle[deg], 5.0 * altitude_angle[deg]},
                1.0 * cm};
    MockMovement movement{gps};

    movement.rotate(types::RotationDirection::Left, 45.0 * horizontal_angle[deg]);
    EXPECT_NEAR(gps.heading().horizontal.force_numerical_value_in(deg), 75.0, 1e-9);
    EXPECT_NEAR(gps.heading().altitude.force_numerical_value_in(deg), 5.0, 1e-9);
}

/**
 * @brief Rotate Right subtracts from the heading.
 */
TEST(SimulationRun, Movement_RotateRightSubtracts) {
    MockGPS gps{Position3D{}, Orientation{90.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]},
                1.0 * cm};
    MockMovement movement{gps};

    movement.rotate(types::RotationDirection::Right, 30.0 * horizontal_angle[deg]);
    EXPECT_NEAR(gps.heading().horizontal.force_numerical_value_in(deg), 60.0, 1e-9);
}

/**
 * @brief Every movement command reports success.
 */
TEST(SimulationRun, Movement_AllReturnSuccess) {
    MockGPS gps{Position3D{}, Orientation{}, 1.0 * cm};
    MockMovement movement{gps};

    EXPECT_TRUE(static_cast<bool>(movement.advance(1.0 * cm)));
    EXPECT_TRUE(static_cast<bool>(movement.elevate(1.0 * cm)));
    EXPECT_TRUE(static_cast<bool>(movement.rotate(types::RotationDirection::Left,
                                                  1.0 * horizontal_angle[deg])));
}

/**
 * @brief Advance at a 45-deg heading decomposes into equal X and Y components (cos/sin split).
 */
TEST(SimulationRun, Movement_AdvanceDiagonalAt45) {
    MockGPS gps{Position3D{}, Orientation{45.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]},
                1.0 * cm};
    MockMovement movement{gps};

    movement.advance(10.0 * cm);

    const double expected = 10.0 * std::sqrt(0.5); // 10 * cos(45 deg) == 10 * sin(45 deg)
    EXPECT_NEAR(gps.position().x.force_numerical_value_in(cm), expected, 1e-9);
    EXPECT_NEAR(gps.position().y.force_numerical_value_in(cm), expected, 1e-9);
    EXPECT_NEAR(gps.position().z.force_numerical_value_in(cm), 0.0, 1e-9);
}

/**
 * @brief A rotation changes the heading that the NEXT advance travels along.
 *
 * Rotate left 90 from heading 0, then advance: the drone must move +Y (south), not +X — proving
 * advance reads the live heading rather than a stale one.
 */
TEST(SimulationRun, Movement_RotateThenAdvanceUsesNewHeading) {
    MockGPS gps{Position3D{}, Orientation{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]},
                1.0 * cm};
    MockMovement movement{gps};

    movement.rotate(types::RotationDirection::Left, 90.0 * horizontal_angle[deg]);
    movement.advance(5.0 * cm);

    EXPECT_NEAR(gps.position().x.force_numerical_value_in(cm), 0.0, 1e-9);
    EXPECT_NEAR(gps.position().y.force_numerical_value_in(cm), 5.0, 1e-9);
}