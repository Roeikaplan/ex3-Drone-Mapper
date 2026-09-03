/**
 * @file MissionControlTest.cpp
 * @brief Coverage of the mission loop's stop conditions, its save discipline, and the verbose trace.
 */

#include <MissionControl/MissionControlImpl.h>

#include "Fakes.h"

#include <gtest/gtest.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

namespace {

namespace fs = std::filesystem;

using namespace common;
using mission_control_323998450_211633813::MissionControlImpl;
using mission_control_323998450_211633813::testing::FakeGPS;
using mission_control_323998450_211633813::testing::FakeLidar;
using mission_control_323998450_211633813::testing::FakeMap;
using mission_control_323998450_211633813::testing::FakeMovement;
using mission_control_323998450_211633813::testing::ScriptedAlgorithm;

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
 * @brief A command carrying only a scan.
 * @return The command.
 */
[[nodiscard]] types::MappingStepCommand scanOnly() {
    types::MappingStepCommand command{};
    command.scan_orientation = facing(0.0);
    return command;
}

/**
 * @brief Everything a mission control needs, wired together.
 * @note The dependencies struct holds references, so the objects they point at must outlive the
 *       mission control. Bundling them here is what guarantees that.
 */
struct Rig {
    /**
     * @brief Build a rig.
     * @param script Commands the algorithm will issue in order.
     * @param max_steps The mission's step budget.
     * @param output_map_file Where the mission should save; empty disables saving.
     * @param verbose Whether to request a verbose trace.
     */
    Rig(std::vector<types::MappingStepCommand> script, std::size_t max_steps,
        fs::path output_map_file, bool verbose)
        : map(100.0, 1.0),
          gps(pos(50.0, 50.0, 50.0), facing(0.0)),
          movement(gps),
          lidar(types::LidarConfigData{5.0 * cm, 50.0 * cm, 1.0 * cm, 1},
                types::LidarScanResult{types::LidarHit{20.0 * cm, facing(0.0)}}),
          algorithm(MappingAlgorithmDependencies{mission, lidar.config(), drone, map},
                    std::move(script)) {
        mission.max_steps = max_steps;
        mission.mission_bounds.max_x = 100.0 * x_extent[cm];
        mission.mission_bounds.max_y = 100.0 * y_extent[cm];
        mission.mission_bounds.max_height = 100.0 * z_extent[cm];

        drone.max_rotate = 90.0 * horizontal_angle[deg];
        drone.max_advance = 30.0 * cm;
        drone.max_elevate = 20.0 * cm;

        control = std::make_unique<MissionControlImpl>(MissionControlDependencies{
            mission, drone, lidar, gps, movement, map, algorithm, std::move(output_map_file),
            verbose});
    }

    types::MissionConfigData mission{};
    types::DroneConfigData drone{};
    FakeMap map;
    FakeGPS gps;
    FakeMovement movement;
    FakeLidar lidar;
    ScriptedAlgorithm algorithm;
    std::unique_ptr<MissionControlImpl> control{};
};

/**
 * @brief Gives each test its own scratch directory.
 */
class MissionControlTest : public ::testing::Test {
protected:
    /**
     * @brief Create a uniquely named scratch directory.
     */
    void SetUp() override {
        const ::testing::TestInfo* const info =
            ::testing::UnitTest::GetInstance()->current_test_info();
        dir_ = fs::temp_directory_path() /
               ("ex3_mission_" + std::string{info->name()} + "_" + std::to_string(::getpid()));
        std::error_code ec;
        fs::remove_all(dir_, ec);
        fs::create_directories(dir_, ec);
    }

    /**
     * @brief Remove the scratch directory.
     */
    void TearDown() override {
        std::error_code ec;
        fs::remove_all(dir_, ec);
    }

    fs::path dir_{};
};

/**
 * @brief The mission ends the moment the algorithm reports `Finished`, well inside its budget.
 * @note The step count matters as much as the status: the finishing step is counted, and nothing
 *       runs after it.
 */
TEST_F(MissionControlTest, StopsWhenTheAlgorithmReportsFinished) {
    types::MappingStepCommand finished{};
    finished.status = types::AlgorithmStatus::Finished;

    Rig rig{{scanOnly(), scanOnly(), finished}, 100, dir_ / "map.npy", false};
    const types::MissionRunResult result = rig.control->runMission();

    EXPECT_EQ(result.status, types::MissionRunStatus::Completed);
    EXPECT_EQ(result.steps, 3u);
    EXPECT_TRUE(result.errors.empty());
}

/**
 * @brief An algorithm that never finishes is stopped by `max_steps`, reported as `MaxSteps`.
 * @note The budget is the only thing standing between a non-terminating plugin and a run that never
 *       returns, so `MaxSteps` is a normal outcome rather than a failure.
 */
TEST_F(MissionControlTest, RunsToTheStepBudgetWhenTheAlgorithmNeverFinishes) {
    Rig rig{{scanOnly()}, 5, dir_ / "map.npy", false};
    const types::MissionRunResult result = rig.control->runMission();

    EXPECT_EQ(result.status, types::MissionRunStatus::MaxSteps);
    EXPECT_EQ(result.steps, 5u);
}

/**
 * @brief A refused command ends the mission and surfaces as a `DRONE_STEP_ERROR`.
 * @note The mission does not skip the bad step and carry on. Replanning around a command the
 *       algorithm should not have issued would hide the fault from the report entirely.
 */
TEST_F(MissionControlTest, ADroneErrorEndsTheMissionAndIsReported) {
    types::MappingStepCommand illegal{};
    illegal.movement = types::MovementCommand{};
    illegal.movement->type = types::MovementCommandType::Advance;
    illegal.movement->distance = 500.0 * cm;

    Rig rig{{scanOnly(), illegal, scanOnly()}, 100, dir_ / "map.npy", false};
    const types::MissionRunResult result = rig.control->runMission();

    EXPECT_EQ(result.status, types::MissionRunStatus::Error);
    EXPECT_EQ(result.steps, 2u) << "the mission stops at the offending step, it does not replan";
    ASSERT_EQ(result.errors.size(), 1u);
    EXPECT_EQ(result.errors.front().code, "DRONE_STEP_ERROR");
}

/**
 * @brief The output map is written once, at the end, not once per step.
 */
TEST_F(MissionControlTest, TheMapIsSavedExactlyOnce) {
    /**
     * @note Saving per step would be thousands of writes per mission for a map only the final state
     *       of which is ever scored.
     */
    Rig rig{{scanOnly()}, 20, dir_ / "map.npy", false};
    const types::MissionRunResult result = rig.control->runMission();

    EXPECT_EQ(result.steps, 20u);
    EXPECT_EQ(rig.map.saveCount(), 1u);
    EXPECT_EQ(rig.map.lastSavePath(), dir_ / "map.npy");
}

/**
 * @brief A mission that ends in error still saves its partial map.
 * @note The run scores -1 either way, so the map is kept for inspection rather than for scoring -
 *       a failed run is exactly the one someone will want to look at.
 */
TEST_F(MissionControlTest, TheMapIsStillSavedAfterAFailedMission) {
    types::MappingStepCommand illegal{};
    illegal.movement = types::MovementCommand{};
    illegal.movement->type = types::MovementCommandType::Advance;
    illegal.movement->distance = 500.0 * cm;

    Rig rig{{illegal}, 20, dir_ / "map.npy", false};
    const types::MissionRunResult result = rig.control->runMission();

    ASSERT_EQ(result.status, types::MissionRunStatus::Error);
    EXPECT_EQ(rig.map.saveCount(), 1u)
        << "a partial map is still worth inspecting even though the run scores -1";
}

/**
 * @brief Without `-verbose` no trace file is created at all.
 * @note The negative case is what gives the positive one meaning: a trace that always appeared would
 *       make the flag decorative.
 */
TEST_F(MissionControlTest, NoTraceFileWithoutVerbose) {
    Rig rig{{scanOnly()}, 3, dir_ / "map.npy", false};
    (void)rig.control->runMission();

    EXPECT_FALSE(fs::exists(dir_ / "map__steps.csv"))
        << "the file's absence is what makes its presence meaningful";
}

/**
 * @brief With `-verbose` the trace carries a header plus exactly one row per step.
 * @note Also pins the column set and the command encoding, since the trace is a file other tools and
 *       people read - the earlier scan-only row must show `none` rather than an empty field.
 */
TEST_F(MissionControlTest, VerboseWritesOneRowPerStep) {
    types::MappingStepCommand advance{};
    advance.movement = types::MovementCommand{};
    advance.movement->type = types::MovementCommandType::Advance;
    advance.movement->distance = 10.0 * cm;

    Rig rig{{scanOnly(), advance, scanOnly()}, 3, dir_ / "map.npy", true};
    const types::MissionRunResult result = rig.control->runMission();
    ASSERT_EQ(result.steps, 3u);

    const fs::path trace = dir_ / "map__steps.csv";
    ASSERT_TRUE(fs::exists(trace));

    std::ifstream stream(trace);
    std::string line;
    std::vector<std::string> lines;
    while (std::getline(stream, line)) {
        lines.push_back(line);
    }

    ASSERT_EQ(lines.size(), 4u) << "a header plus one row per step";
    EXPECT_EQ(lines[0], "step,x_cm,y_cm,z_cm,heading_deg,command,status");
    EXPECT_NE(lines[2].find("advance_"), std::string::npos)
        << "the command column records what was asked for";
    EXPECT_NE(lines[1].find("none"), std::string::npos)
        << "a scan-only step carries no movement";
}

/**
 * @brief A command that was refused still appears in the trace, marked as an error.
 */
TEST_F(MissionControlTest, TheTraceRecordsARefusedCommand) {
    /**
     * @note The command is recorded before validation, so the row that explains a stalled mission is
     *       exactly the one a naive implementation would omit.
     */
    types::MappingStepCommand illegal{};
    illegal.movement = types::MovementCommand{};
    illegal.movement->type = types::MovementCommandType::Advance;
    illegal.movement->distance = 500.0 * cm;

    Rig rig{{illegal}, 5, dir_ / "map.npy", true};
    (void)rig.control->runMission();

    std::ifstream stream(dir_ / "map__steps.csv");
    std::stringstream contents;
    contents << stream.rdbuf();

    EXPECT_NE(contents.str().find("advance_"), std::string::npos);
    EXPECT_NE(contents.str().find("error"), std::string::npos);
}

/**
 * @brief A `max_steps` of zero calls the algorithm not once, yet still saves a map.
 * @note The degenerate budget has to produce a well-formed run rather than a special case: the
 *       simulator scores every combination the composition described, so a missing output file
 *       would look like a crash rather than like a mission that was given nothing to do.
 */
TEST_F(MissionControlTest, AZeroStepBudgetDoesNothingButStillSaves) {
    Rig rig{{scanOnly()}, 0, dir_ / "map.npy", false};
    const types::MissionRunResult result = rig.control->runMission();

    EXPECT_EQ(result.steps, 0u);
    EXPECT_EQ(result.status, types::MissionRunStatus::MaxSteps);
    EXPECT_EQ(rig.algorithm.callCount(), 0u);
    EXPECT_EQ(rig.map.saveCount(), 1u);
}

} // namespace
