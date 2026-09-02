/**
 * @file DroneControlTest.cpp
 * @brief Coverage of the step ordering, the validation rules, and what they can and cannot catch.
 * @note Two cases here exist because of bugs that are otherwise invisible: the swept-path check
 *       (a move can cross many voxels while its endpoint is clear) and the first-step null scan.
 */

#include <MissionControl/DroneControlImpl.h>

#include "Fakes.h"

#include <gtest/gtest.h>

#include <limits>

namespace {

using namespace common;
using mission_control::DroneControlImpl;
using mission_control::testing::FakeGPS;
using mission_control::testing::FakeLidar;
using mission_control::testing::FakeMap;
using mission_control::testing::FakeMovement;
using mission_control::testing::ScriptedAlgorithm;

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
 * @brief A drone whose limits comfortably permit the moves these tests issue.
 * @return The configuration.
 */
[[nodiscard]] types::DroneConfigData permissiveDrone() {
    types::DroneConfigData drone{};
    drone.radius = 4.0 * cm;
    drone.max_rotate = 90.0 * horizontal_angle[deg];
    drone.max_advance = 30.0 * cm;
    drone.max_elevate = 20.0 * cm;
    return drone;
}

/**
 * @brief A mission whose bounds span a cube anchored at the origin.
 * @param span_cm Extent of each axis.
 * @return The configuration.
 */
[[nodiscard]] types::MissionConfigData missionSpanning(double span_cm) {
    types::MissionConfigData mission{};
    mission.max_steps = 100;
    mission.mission_bounds.max_x = span_cm * x_extent[cm];
    mission.mission_bounds.max_y = span_cm * y_extent[cm];
    mission.mission_bounds.max_height = span_cm * z_extent[cm];
    return mission;
}

/**
 * @brief A movement command.
 * @param type Which primitive.
 * @param magnitude Distance in centimetres, or angle in degrees for a rotation.
 * @return The command.
 */
[[nodiscard]] types::MovementCommand move(types::MovementCommandType type, double magnitude) {
    types::MovementCommand command{};
    command.type = type;
    command.distance = magnitude * cm;
    command.angle = magnitude * horizontal_angle[deg];
    return command;
}

/**
 * @brief Everything a drone controller needs, wired together.
 * @note Kept as one object so the references handed to the controller cannot outlive their owners,
 *       and so each test reads as a scenario rather than as six lines of setup.
 */
struct Rig {
    /**
     * @brief Build a rig around a scripted list of commands.
     * @param script Commands the algorithm will issue in order.
     * @param map_span_cm Extent of the map and the mission bounds.
     */
    explicit Rig(std::vector<types::MappingStepCommand> script, double map_span_cm = 100.0)
        : map(map_span_cm, 1.0),
          gps(pos(50.0, 50.0, 50.0), facing(0.0)),
          movement(gps),
          lidar(types::LidarConfigData{5.0 * cm, 50.0 * cm, 1.0 * cm, 1},
                types::LidarScanResult{types::LidarHit{20.0 * cm, facing(0.0)}}),
          algorithm(MappingAlgorithmDependencies{mission, lidar.config(), drone, map},
                    std::move(script)),
          control(drone, missionSpanning(map_span_cm), lidar, gps, movement, map, algorithm) {}

    types::DroneConfigData drone = permissiveDrone();
    types::MissionConfigData mission = missionSpanning(100.0);
    FakeMap map;
    FakeGPS gps;
    FakeMovement movement;
    FakeLidar lidar;
    ScriptedAlgorithm algorithm;
    DroneControlImpl control;
};

/**
 * @brief A command carrying only a scan.
 * @return The command.
 */
[[nodiscard]] types::MappingStepCommand scanOnly() {
    types::MappingStepCommand command{};
    command.scan_orientation = facing(0.0);
    return command;
}

/**
 * @brief The first call to the algorithm receives a null scan pointer, not an empty scan.
 */
TEST(DroneControl, TheFirstStepIsHandedANullScan) {
    /**
     * @note An empty scan would tell the algorithm it looked and saw nothing, which is a different
     *       claim from having not looked yet - and it is the claim that would make a frontier
     *       planner mark the whole world explored on step one.
     */
    Rig rig{{scanOnly(), scanOnly()}};

    EXPECT_EQ(rig.control.step().status, types::DroneStepStatus::Continue);
    EXPECT_TRUE(rig.algorithm.firstScanWasNull());
}

/**
 * @brief When one command carries both a move and a scan, the move is executed first.
 */
TEST(DroneControl, MovementHappensBeforeTheScanInTheSameCommand) {
    /**
     * @note The sensor reads the same GPS the controller does, so a scan taken first would describe
     *       a pose the drone is about to leave. Asserting the *algorithm* saw the moved pose on the
     *       following step is the observable consequence.
     */
    types::MappingStepCommand both{};
    both.movement = move(types::MovementCommandType::Advance, 10.0);
    both.scan_orientation = facing(0.0);

    Rig rig{{both, scanOnly()}};

    ASSERT_EQ(rig.control.step().status, types::DroneStepStatus::Continue);
    EXPECT_EQ(rig.lidar.scanCount(), 1u);
    EXPECT_NEAR(rig.gps.position().x.force_numerical_value_in(cm), 60.0, 1e-9);

    ASSERT_EQ(rig.control.step().status, types::DroneStepStatus::Continue);
    ASSERT_EQ(rig.algorithm.observedStates().size(), 2u);
    EXPECT_NEAR(rig.algorithm.observedStates()[1].position.x.force_numerical_value_in(cm), 60.0,
                1e-9);
}

/**
 * @brief An advance beyond `max_advance` is refused, and the actuator is never called.
 * @note The call-count assertion is the real content. Refusing only after commanding would leave the
 *       drone somewhere the mission never approved, reported by an error claiming it had not moved.
 */
TEST(DroneControl, AnOverLargeAdvanceIsRefusedBeforeItIsCommanded) {
    types::MappingStepCommand command{};
    command.movement = move(types::MovementCommandType::Advance, 100.0);

    Rig rig{{command}};

    const types::DroneStepResult result = rig.control.step();
    EXPECT_EQ(result.status, types::DroneStepStatus::Error);
    EXPECT_NE(result.message.find("max_advance"), std::string::npos);
    EXPECT_EQ(rig.movement.callCount(), 0u) << "refusal must happen before the actuator is touched";
}

/**
 * @brief A rotation beyond `max_rotate` is refused, naming the limit it broke.
 * @note The message is asserted, not just the status: a plugin author reading the error log needs to
 *       know which of the three limits was exceeded.
 */
TEST(DroneControl, AnOverLargeRotationIsRefused) {
    types::MappingStepCommand command{};
    command.movement = move(types::MovementCommandType::Rotate, 180.0);

    Rig rig{{command}};

    const types::DroneStepResult result = rig.control.step();
    EXPECT_EQ(result.status, types::DroneStepStatus::Error);
    EXPECT_NE(result.message.find("max_rotate"), std::string::npos);
}

/**
 * @brief An elevation beyond `max_elevate` is refused.
 * @note The third of the three per-command limits, each checked separately so that one shared bound
 *       cannot accidentally stand in for all of them.
 */
TEST(DroneControl, AnOverLargeElevationIsRefused) {
    types::MappingStepCommand command{};
    command.movement = move(types::MovementCommandType::Elevate, 50.0);

    Rig rig{{command}};

    const types::DroneStepResult result = rig.control.step();
    EXPECT_EQ(result.status, types::DroneStepStatus::Error);
    EXPECT_NE(result.message.find("max_elevate"), std::string::npos);
}

/**
 * @brief A move within the drone's limits is still refused when it leaves the mission bounds.
 * @note A different class of mistake from exceeding a drone limit - this command is one the drone
 *       could physically perform - which is why the two checks report distinct messages.
 */
TEST(DroneControl, AMoveLeavingTheMissionBoundsIsRefused) {
    types::MappingStepCommand command{};
    command.movement = move(types::MovementCommandType::Advance, 30.0);

    Rig rig{{command}, 60.0};

    const types::DroneStepResult result = rig.control.step();
    EXPECT_EQ(result.status, types::DroneStepStatus::Error);
    EXPECT_NE(result.message.find("boundaries"), std::string::npos);
    EXPECT_EQ(rig.movement.callCount(), 0u);
}

/**
 * @brief A move ending in a cell already observed `Occupied` is refused.
 * @note The straightforward half of the obstacle check. The swept-path case below is the half an
 *       endpoint-only implementation would silently fail.
 */
TEST(DroneControl, AMoveEndingInAKnownObstacleIsRefused) {
    types::MappingStepCommand command{};
    command.movement = move(types::MovementCommandType::Advance, 10.0);

    Rig rig{{command}};
    rig.map.set(pos(60.0, 50.0, 50.0), types::VoxelOccupancy::Occupied);

    const types::DroneStepResult result = rig.control.step();
    EXPECT_EQ(result.status, types::DroneStepStatus::Error);
    EXPECT_EQ(rig.movement.callCount(), 0u);
}

/**
 * @brief A move whose destination is clear but whose path crosses a known wall is refused.
 * @note One of the two cases the file header calls out as existing because of an otherwise invisible
 *       bug - it passes trivially against an endpoint-only check.
 */
TEST(DroneControl, AMovePassingThroughAKnownObstacleIsRefused) {
    /**
     * @note The whole point of checking the swept path rather than the destination. A 20 cm advance
     *       over a 1 cm grid crosses twenty cells; with a wall at 55 and clear space at 70, an
     *       endpoint-only check sees nothing wrong and flies the drone straight through it. The real
     *       configurations are no kinder - 30 cm of travel over a 5 cm grid is six cells.
     */
    types::MappingStepCommand command{};
    command.movement = move(types::MovementCommandType::Advance, 20.0);

    Rig rig{{command}};
    rig.map.set(pos(55.0, 50.0, 50.0), types::VoxelOccupancy::Occupied);
    ASSERT_EQ(rig.map.atVoxel(pos(70.0, 50.0, 50.0)), types::VoxelOccupancy::Unmapped)
        << "the destination itself is clear, which is what makes this the interesting case";

    const types::DroneStepResult result = rig.control.step();
    EXPECT_EQ(result.status, types::DroneStepStatus::Error);
    EXPECT_NE(result.message.find("known-occupied"), std::string::npos);
    EXPECT_EQ(rig.movement.callCount(), 0u);
}

/**
 * @brief A wall the drone has never scanned is *not* refused - and that is the correct behaviour.
 * @note Asserts a limitation rather than a guarantee, so the boundary of what this class can enforce
 *       is recorded here instead of being rediscovered from a collision in a full run.
 */
TEST(DroneControl, AnUnobservedObstacleCannotBeRefused) {
    /**
     * @note Documenting the boundary of what this class can enforce. It has no ground truth, so a
     *       wall the drone has never scanned is invisible here - which is exactly why the mapping
     *       algorithm carries the real collision guarantee by routing only through observed-`Empty`
     *       space.
     */
    types::MappingStepCommand command{};
    command.movement = move(types::MovementCommandType::Advance, 20.0);

    Rig rig{{command}};

    EXPECT_EQ(rig.control.step().status, types::DroneStepStatus::Continue);
    EXPECT_EQ(rig.movement.callCount(), 1u);
}

/**
 * @brief A scan command reaches the map, not merely the sensor.
 * @note Covers the wiring from `ILidar::scan` through `ScanResultToVoxels` into the output map. A
 *       controller that scanned but never applied the result would leave the map permanently blank
 *       while every step still reported success.
 */
TEST(DroneControl, AScanIsWrittenIntoTheMap) {
    Rig rig{{scanOnly()}};

    ASSERT_EQ(rig.control.step().status, types::DroneStepStatus::Continue);

    EXPECT_GT(rig.map.countOf(types::VoxelOccupancy::Empty), 0u);
    EXPECT_EQ(rig.map.countOf(types::VoxelOccupancy::Occupied), 1u)
        << "the canned scan reports a single hit";
}

/**
 * @brief `AlgorithmStatus::Finished` is translated to `DroneStepStatus::Completed`.
 * @note The seam between the algorithm's vocabulary and the mission loop's. The loop breaks on
 *       `Completed`, so a mistranslation would run the mission on to `max_steps` instead.
 */
TEST(DroneControl, FinishingMapsToCompleted) {
    types::MappingStepCommand command{};
    command.status = types::AlgorithmStatus::Finished;

    Rig rig{{command}};

    EXPECT_EQ(rig.control.step().status, types::DroneStepStatus::Completed);
}

/**
 * @brief `FinishedWithUnmappableVoxels` also completes rather than erroring.
 * @note Giving up on cells that were never reachable is a finished mission, not a failed one.
 *       Mapping it to `Error` would score the run -1 and discard a perfectly good map.
 */
TEST(DroneControl, FinishingWithUnmappableVoxelsAlsoCompletes) {
    types::MappingStepCommand command{};
    command.status = types::AlgorithmStatus::FinishedWithUnmappableVoxels;

    Rig rig{{command}};

    EXPECT_EQ(rig.control.step().status, types::DroneStepStatus::Completed)
        << "an algorithm that gave up on unreachable cells still finished its mission";
}

/**
 * @brief The step index advances once per step and is reflected in the reported state.
 * @note `state()` is what the algorithm is handed on every call, so a counter that never advanced
 *       would leave it unable to tell one step from the next.
 */
TEST(DroneControl, TheStepIndexAdvancesWithEachStep) {
    Rig rig{{scanOnly(), scanOnly(), scanOnly()}};

    EXPECT_EQ(rig.control.state().step_index, 0u);
    ASSERT_EQ(rig.control.step().status, types::DroneStepStatus::Continue);
    EXPECT_EQ(rig.control.state().step_index, 1u);
    ASSERT_EQ(rig.control.step().status, types::DroneStepStatus::Continue);
    EXPECT_EQ(rig.control.state().step_index, 2u);
}

} // namespace
