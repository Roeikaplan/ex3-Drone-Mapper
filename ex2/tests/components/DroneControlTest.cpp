#include <drone_mapper/DroneControlImpl.h>

#include <drone_mapper/IDroneMovement.h>
#include <drone_mapper/IGPS.h>
#include <drone_mapper/ILidar.h>
#include <drone_mapper/IMappingAlgorithm.h>
#include <drone_mapper/IMutableMap3D.h>

#include <gtest/gtest.h>

#include <deque>
#include <filesystem>
#include <functional>
#include <string>
#include <vector>

namespace {

using namespace drone_mapper;

// ---- Fakes (all collaborators are hand-fakes so a bug in another component can't fail this suite) --

// Scripts the commands step() reacts to and records whether latest_scan was null on each call.
class FakeAlgorithm : public IMappingAlgorithm {
public:
    FakeAlgorithm(const types::MissionConfigData& mission, const types::LidarConfigData& lidar,
                  const types::DroneConfigData& drone, const IMap3D& map)
        : IMappingAlgorithm(mission, lidar, drone, map) {}

    [[nodiscard]] types::MappingStepCommand nextStep(const types::DroneState&,
                                                     const types::LidarScanResult* scan) override {
        scan_null_on_call.push_back(scan == nullptr);
        if (commands.empty()) {
            return {};
        }
        const types::MappingStepCommand command = commands.front();
        commands.pop_front();
        return command;
    }

    std::deque<types::MappingStepCommand> commands;
    std::vector<bool> scan_null_on_call;
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

// Always fails with a fixed message — models a dead actuator so error forwarding can be asserted.
class FailingMovement : public IDroneMovement {
public:
    types::MovementResult rotate(types::RotationDirection, HorizontalAngle) override {
        return {false, "motor stall"};
    }
    types::MovementResult advance(PhysicalLength) override {
        return {false, "motor stall"};
    }
    types::MovementResult elevate(PhysicalLength) override {
        return {false, "motor stall"};
    }
};

// Records "move" in a shared event log; always succeeds.
class FakeMovement : public IDroneMovement {
public:
    explicit FakeMovement(std::vector<std::string>& log) : log_(log) {}
    types::MovementResult rotate(types::RotationDirection, HorizontalAngle) override {
        log_.push_back("move");
        return {true, {}};
    }
    types::MovementResult advance(PhysicalLength) override {
        log_.push_back("move");
        return {true, {}};
    }
    types::MovementResult elevate(PhysicalLength) override {
        log_.push_back("move");
        return {true, {}};
    }

private:
    std::vector<std::string>& log_;
};

// Records "scan" in the shared log, captures the requested orientation, returns a canned result.
class FakeLidar : public ILidar {
public:
    FakeLidar(std::vector<std::string>& log, types::LidarConfigData config,
              types::LidarScanResult scan)
        : log_(log), config_(config), scan_(std::move(scan)) {}

    [[nodiscard]] types::LidarScanResult scan(Orientation orientation) const override {
        log_.push_back("scan");
        last_orientation = orientation;
        ++scan_calls;
        return scan_;
    }
    [[nodiscard]] types::LidarConfigData config() const override {
        return config_;
    }

    mutable Orientation last_orientation{};
    mutable int scan_calls = 0;

private:
    std::vector<std::string>& log_;
    types::LidarConfigData config_;
    types::LidarScanResult scan_;
};

// Programmable occupancy (for the known-occupied rejection) and a `set()` counter.
class FakeMap : public IMutableMap3D {
public:
    explicit FakeMap(types::MapConfig config) : config_(config) {}
    [[nodiscard]] types::VoxelOccupancy atVoxel(const Position3D& pos) const override {
        return occupied_ && occupied_(pos) ? types::VoxelOccupancy::Occupied
                                           : types::VoxelOccupancy::Unmapped;
    }
    [[nodiscard]] types::MapConfig getMapConfig() const override {
        return config_;
    }
    [[nodiscard]] bool isInBounds(const Position3D&) const override {
        return true;
    }
    void set(const Position3D&, types::VoxelOccupancy) override {
        ++set_calls;
    }
    void save(const std::filesystem::path&) const override {}

    std::function<bool(const Position3D&)> occupied_{};
    int set_calls = 0;

private:
    types::MapConfig config_;
};

// ---- Config helpers ----

/**
 * @brief A drone with explicit per-command limits (degrees / cm).
 */
[[nodiscard]] types::DroneConfigData drone(double max_rotate_deg, double max_advance_cm,
                                           double max_elevate_cm) {
    return types::DroneConfigData{5.0 * cm, max_rotate_deg * horizontal_angle[deg],
                                  max_advance_cm * cm, max_elevate_cm * cm};
}

/**
 * @brief A generous drone (limits large enough that per-command checks always pass).
 */
[[nodiscard]] types::DroneConfigData generousDrone() {
    return drone(360.0, 1000.0, 1000.0);
}

/**
 * @brief A mission whose bounds are a cube `[0, max]` on every axis.
 */
[[nodiscard]] types::MissionConfigData missionBounds(double max_cm) {
    types::MissionConfigData mission{};
    mission.mission_bounds = types::MappingBounds{
        XLength{}, max_cm * x_extent[cm], YLength{}, max_cm * y_extent[cm],
        ZLength{}, max_cm * z_extent[cm],
    };
    return mission;
}

/**
 * @brief A 10-cm-resolution output map spanning [0,100] cm on each axis.
 */
[[nodiscard]] types::MapConfig mapConfig() {
    const types::MappingBounds bounds{
        XLength{}, 100.0 * x_extent[cm], YLength{}, 100.0 * y_extent[cm],
        ZLength{}, 100.0 * z_extent[cm],
    };
    return types::MapConfig{bounds, Position3D{}, 10.0 * cm};
}

[[nodiscard]] types::LidarConfigData lidarConfig() {
    return types::LidarConfigData{20.0 * cm, 120.0 * cm, 2.5 * cm, 1};
}

/**
 * @brief A one-hit scan so a scan step has something for ScanResultToVoxels to apply.
 */
[[nodiscard]] types::LidarScanResult oneHitScan() {
    return {types::LidarHit{30.0 * cm, Orientation{0.0 * horizontal_angle[deg],
                                                   0.0 * altitude_angle[deg]}}};
}

/**
 * @brief A movement-only step command.
 */
[[nodiscard]] types::MappingStepCommand moveCmd(types::MovementCommand move) {
    types::MappingStepCommand command{};
    command.movement = move;
    command.status = types::AlgorithmStatus::Working;
    return command;
}

/**
 * @brief A scan-only step command at the given orientation.
 */
[[nodiscard]] types::MappingStepCommand scanCmd(Orientation orientation) {
    types::MappingStepCommand command{};
    command.scan_orientation = orientation;
    command.status = types::AlgorithmStatus::Working;
    return command;
}

/**
 * @brief A step command carrying a status only (no movement or scan).
 */
[[nodiscard]] types::MappingStepCommand statusCmd(types::AlgorithmStatus status) {
    types::MappingStepCommand command{};
    command.status = status;
    return command;
}

// Fixture owning the fakes; each test builds a controller with the drone/mission it needs (those are
// copied by value into DroneControlImpl, so they must be chosen at construction).
class DroneControl : public ::testing::Test {
protected:
    std::vector<std::string> log;
    FakeGPS gps;
    FakeMap map{mapConfig()};
    FakeMovement movement{log};
    FakeLidar lidar{log, lidarConfig(), oneHitScan()};
    FakeAlgorithm algo{types::MissionConfigData{}, lidarConfig(), generousDrone(), map};

    [[nodiscard]] DroneControlImpl make(types::DroneConfigData d, types::MissionConfigData m) {
        return DroneControlImpl{d, m, lidar, gps, movement, map, algo};
    }
};

} // namespace

/**
 * @brief The first step hands the algorithm a null latest_scan (no LiDAR data yet).
 */
TEST_F(DroneControl, FirstStepPassesNullScan) {
    algo.commands.push_back(statusCmd(types::AlgorithmStatus::Working));
    auto control = make(generousDrone(), missionBounds(100.0));

    (void)control.step();

    ASSERT_FALSE(algo.scan_null_on_call.empty());
    EXPECT_TRUE(algo.scan_null_on_call[0]);
}

/**
 * @brief After a scan step, the next step forwards the stored scan (non-null) to the algorithm.
 */
TEST_F(DroneControl, LatestScanForwarded) {
    algo.commands.push_back(scanCmd(Orientation{}));
    algo.commands.push_back(statusCmd(types::AlgorithmStatus::Working));
    auto control = make(generousDrone(), missionBounds(100.0));

    (void)control.step(); // performs a scan -> stores latest_scan_
    (void)control.step(); // should receive the stored scan

    ASSERT_EQ(algo.scan_null_on_call.size(), 2u);
    EXPECT_TRUE(algo.scan_null_on_call[0]);
    EXPECT_FALSE(algo.scan_null_on_call[1]);
}

/**
 * @brief A rotation beyond the drone's max_rotate is rejected and not executed.
 */
TEST_F(DroneControl, RotateOverLimitErrors) {
    algo.commands.push_back(moveCmd(types::MovementCommand{types::MovementCommandType::Rotate,
                                                           types::RotationDirection::Left,
                                                           90.0 * horizontal_angle[deg], 0.0 * cm}));
    auto control = make(drone(45.0, 1000.0, 1000.0), missionBounds(100.0));

    const types::DroneStepResult result = control.step();

    EXPECT_EQ(result.status, types::DroneStepStatus::Error);
    EXPECT_EQ(result.message, "Rotation exceeds drone max_rotate.");
    EXPECT_TRUE(log.empty());
}

/**
 * @brief An advance beyond the drone's max_advance is rejected and not executed.
 */
TEST_F(DroneControl, AdvanceOverLimitErrors) {
    algo.commands.push_back(moveCmd(types::MovementCommand{types::MovementCommandType::Advance,
                                                           types::RotationDirection::Left,
                                                           0.0 * horizontal_angle[deg], 100.0 * cm}));
    auto control = make(drone(360.0, 50.0, 1000.0), missionBounds(1000.0));

    const types::DroneStepResult result = control.step();

    EXPECT_EQ(result.status, types::DroneStepStatus::Error);
    EXPECT_EQ(result.message, "Advance exceeds drone max_advance.");
    EXPECT_TRUE(log.empty());
}

/**
 * @brief An elevate beyond the drone's max_elevate is rejected and not executed.
 */
TEST_F(DroneControl, ElevateOverLimitErrors) {
    algo.commands.push_back(moveCmd(types::MovementCommand{types::MovementCommandType::Elevate,
                                                           types::RotationDirection::Left,
                                                           0.0 * horizontal_angle[deg], 100.0 * cm}));
    auto control = make(drone(360.0, 1000.0, 40.0), missionBounds(1000.0));

    const types::DroneStepResult result = control.step();

    EXPECT_EQ(result.status, types::DroneStepStatus::Error);
    EXPECT_EQ(result.message, "Elevate exceeds drone max_elevate.");
    EXPECT_TRUE(log.empty());
}

/**
 * @brief A movement whose target leaves the mission bounds is rejected.
 */
TEST_F(DroneControl, MovementLeavingBoundsErrors) {
    // Advance 10 cm from the origin (heading 0 -> +X) lands at x=10, outside a [0,5] cube.
    algo.commands.push_back(moveCmd(types::MovementCommand{types::MovementCommandType::Advance,
                                                           types::RotationDirection::Left,
                                                           0.0 * horizontal_angle[deg], 10.0 * cm}));
    auto control = make(generousDrone(), missionBounds(5.0));

    const types::DroneStepResult result = control.step();

    EXPECT_EQ(result.status, types::DroneStepStatus::Error);
    EXPECT_EQ(result.message, "Movement leaves mission boundaries.");
    EXPECT_TRUE(log.empty());
}

/**
 * @brief A movement into a voxel already known to be Occupied is rejected.
 */
TEST_F(DroneControl, MovementIntoKnownOccupiedErrors) {
    map.occupied_ = [](const Position3D& p) { return p.x.force_numerical_value_in(cm) >= 5.0; };
    algo.commands.push_back(moveCmd(types::MovementCommand{types::MovementCommandType::Advance,
                                                           types::RotationDirection::Left,
                                                           0.0 * horizontal_angle[deg], 10.0 * cm}));
    auto control = make(generousDrone(), missionBounds(100.0));

    const types::DroneStepResult result = control.step();

    EXPECT_EQ(result.status, types::DroneStepStatus::Error);
    EXPECT_EQ(result.message, "Movement enters a known-occupied voxel.");
    EXPECT_TRUE(log.empty());
}

/**
 * @brief A valid movement is executed and the step continues.
 */
TEST_F(DroneControl, ValidMovementExecutes) {
    algo.commands.push_back(moveCmd(types::MovementCommand{types::MovementCommandType::Advance,
                                                           types::RotationDirection::Left,
                                                           0.0 * horizontal_angle[deg], 10.0 * cm}));
    auto control = make(generousDrone(), missionBounds(100.0));

    const types::DroneStepResult result = control.step();

    EXPECT_EQ(result.status, types::DroneStepStatus::Continue);
    ASSERT_EQ(log.size(), 1u);
    EXPECT_EQ(log[0], "move");
}

/**
 * @brief A scan command dispatches the LiDAR at the requested orientation and applies it.
 */
TEST_F(DroneControl, ScanDispatch) {
    const Orientation orientation{45.0 * horizontal_angle[deg], 10.0 * altitude_angle[deg]};
    algo.commands.push_back(scanCmd(orientation));
    auto control = make(generousDrone(), missionBounds(100.0));

    (void)control.step();

    EXPECT_EQ(lidar.scan_calls, 1);
    EXPECT_DOUBLE_EQ(lidar.last_orientation.horizontal.force_numerical_value_in(deg), 45.0);
    EXPECT_DOUBLE_EQ(lidar.last_orientation.altitude.force_numerical_value_in(deg), 10.0);
    EXPECT_GT(map.set_calls, 0); // the converted scan wrote voxels into the output map
}

/**
 * @brief When a command carries both a movement and a scan, the movement runs first.
 */
TEST_F(DroneControl, MovementBeforeScanOrder) {
    types::MappingStepCommand command{};
    command.movement = types::MovementCommand{types::MovementCommandType::Advance,
                                              types::RotationDirection::Left,
                                              0.0 * horizontal_angle[deg], 10.0 * cm};
    command.scan_orientation = Orientation{};
    command.status = types::AlgorithmStatus::Working;
    algo.commands.push_back(command);
    auto control = make(generousDrone(), missionBounds(100.0));

    (void)control.step();

    ASSERT_EQ(log.size(), 2u);
    EXPECT_EQ(log[0], "move");
    EXPECT_EQ(log[1], "scan");
}

/**
 * @brief Algorithm status maps to the step status: Working->Continue, Finished*->Completed.
 */
TEST_F(DroneControl, StatusMapping) {
    algo.commands.push_back(statusCmd(types::AlgorithmStatus::Working));
    algo.commands.push_back(statusCmd(types::AlgorithmStatus::Finished));
    algo.commands.push_back(statusCmd(types::AlgorithmStatus::FinishedWithUnmappableVoxels));
    auto control = make(generousDrone(), missionBounds(100.0));

    EXPECT_EQ(control.step().status, types::DroneStepStatus::Continue);
    EXPECT_EQ(control.step().status, types::DroneStepStatus::Completed);
    EXPECT_EQ(control.step().status, types::DroneStepStatus::Completed);
}

/**
 * @brief state() reports the live GPS pose and a step index that advances each step.
 */
TEST_F(DroneControl, StateReflectsPoseAndStep) {
    gps.position_ = Position3D{1.0 * x_extent[cm], 2.0 * y_extent[cm], 3.0 * z_extent[cm]};
    algo.commands.push_back(statusCmd(types::AlgorithmStatus::Working));
    auto control = make(generousDrone(), missionBounds(100.0));

    const types::DroneState before = control.state();
    EXPECT_DOUBLE_EQ(before.position.x.force_numerical_value_in(cm), 1.0);
    EXPECT_EQ(before.step_index, 0u);

    (void)control.step();
    EXPECT_EQ(control.state().step_index, 1u);
}

/**
 * @brief A Hover movement is accepted without touching any actuator.
 */
TEST_F(DroneControl, HoverAcceptedWithoutActuatorCall) {
    algo.commands.push_back(moveCmd(types::MovementCommand{types::MovementCommandType::Hover,
                                                           types::RotationDirection::Left,
                                                           0.0 * horizontal_angle[deg], 0.0 * cm}));
    auto control = make(generousDrone(), missionBounds(100.0));

    const types::DroneStepResult result = control.step();

    EXPECT_EQ(result.status, types::DroneStepStatus::Continue);
    EXPECT_TRUE(log.empty()); // no rotate/advance/elevate was dispatched
}

/**
 * @brief The per-command limits are inclusive: a command exactly AT the limit executes.
 *
 * Guards against an off-by-one (`>=` instead of `>`) in the limit checks — rotating exactly
 * max_rotate and advancing exactly max_advance are both legal.
 */
TEST_F(DroneControl, LimitsAreInclusive) {
    algo.commands.push_back(moveCmd(types::MovementCommand{types::MovementCommandType::Rotate,
                                                           types::RotationDirection::Left,
                                                           45.0 * horizontal_angle[deg], 0.0 * cm}));
    algo.commands.push_back(moveCmd(types::MovementCommand{types::MovementCommandType::Advance,
                                                           types::RotationDirection::Left,
                                                           0.0 * horizontal_angle[deg], 50.0 * cm}));
    auto control = make(drone(45.0, 50.0, 40.0), missionBounds(100.0));

    EXPECT_EQ(control.step().status, types::DroneStepStatus::Continue);
    EXPECT_EQ(control.step().status, types::DroneStepStatus::Continue);
    EXPECT_EQ(log.size(), 2u); // both commands reached the actuator
}

/**
 * @brief A negative advance within the absolute limit is legal (backwards flight).
 */
TEST_F(DroneControl, NegativeAdvanceWithinLimitExecutes) {
    gps.position_ = Position3D{50.0 * x_extent[cm], 50.0 * y_extent[cm], 50.0 * z_extent[cm]};
    algo.commands.push_back(moveCmd(types::MovementCommand{types::MovementCommandType::Advance,
                                                           types::RotationDirection::Left,
                                                           0.0 * horizontal_angle[deg], -5.0 * cm}));
    auto control = make(drone(360.0, 10.0, 10.0), missionBounds(100.0));

    const types::DroneStepResult result = control.step();

    EXPECT_EQ(result.status, types::DroneStepStatus::Continue);
    ASSERT_EQ(log.size(), 1u);
    EXPECT_EQ(log[0], "move");
}

/**
 * @brief An actuator that reports failure turns the step into an Error with the message forwarded.
 */
TEST_F(DroneControl, ActuatorFailureSurfacesError) {
    FailingMovement failing;
    algo.commands.push_back(moveCmd(types::MovementCommand{types::MovementCommandType::Advance,
                                                           types::RotationDirection::Left,
                                                           0.0 * horizontal_angle[deg], 10.0 * cm}));
    DroneControlImpl control{generousDrone(), missionBounds(100.0), lidar, gps,
                             failing,        map,                  algo};

    const types::DroneStepResult result = control.step();

    EXPECT_EQ(result.status, types::DroneStepStatus::Error);
    EXPECT_EQ(result.message, "motor stall"); // the actuator's message, verbatim
}

/**
 * @brief The stored scan survives non-scan steps: the algorithm keeps receiving the last result.
 *
 * Step 1 scans; step 2 issues no scan. On step 3 the algorithm must still be handed the (retained)
 * scan from step 1 — not a null pointer.
 */
TEST_F(DroneControl, LatestScanRetainedAcrossNonScanSteps) {
    algo.commands.push_back(scanCmd(Orientation{}));
    algo.commands.push_back(statusCmd(types::AlgorithmStatus::Working)); // no scan this step
    algo.commands.push_back(statusCmd(types::AlgorithmStatus::Working));
    auto control = make(generousDrone(), missionBounds(100.0));

    (void)control.step();
    (void)control.step();
    (void)control.step();

    ASSERT_EQ(algo.scan_null_on_call.size(), 3u);
    EXPECT_TRUE(algo.scan_null_on_call[0]);  // bootstrap: nothing scanned yet
    EXPECT_FALSE(algo.scan_null_on_call[1]); // step 1's scan
    EXPECT_FALSE(algo.scan_null_on_call[2]); // still step 1's scan, retained
}

/**
 * @brief A scan-only command dispatches the LiDAR and nothing else.
 */
TEST_F(DroneControl, ScanOnlyStepDoesNotMove) {
    algo.commands.push_back(scanCmd(Orientation{}));
    auto control = make(generousDrone(), missionBounds(100.0));

    (void)control.step();

    ASSERT_EQ(log.size(), 1u);
    EXPECT_EQ(log[0], "scan"); // no "move" entry
}