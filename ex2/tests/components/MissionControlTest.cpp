#include <drone_mapper/IDroneControl.h>
#include <drone_mapper/IMutableMap3D.h>
#include <drone_mapper/MissionControlImpl.h>

#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <cstddef>
#include <filesystem>
#include <string>
#include <utility>

namespace {

using namespace drone_mapper;
using ::testing::Return;

// gmock IDroneControl so each test can script the exact sequence of step() outcomes the mission loop
// should react to; the two-WillOnce cases also assert step() is not called again after termination.
class MockDroneControl : public IDroneControl {
public:
    MOCK_METHOD(types::DroneStepResult, step, (), (override));
    MOCK_METHOD(types::DroneState, state, (), (const, override));
};

/**
 * @brief Minimal in-memory map that records how many times it was saved and queried.
 *
 * Used for both the hidden and output map arguments. It deliberately does no real geometry so a bug
 * injected into `Map3DImpl` cannot make the MissionControl component test fail — the tests only care
 * that the loop saves the output map exactly once (to the configured path) and never reads voxels
 * itself (querying maps is DroneControl/algorithm territory).
 */
class FakeMap : public IMutableMap3D {
public:
    [[nodiscard]] types::VoxelOccupancy atVoxel(const Position3D&) const override {
        ++at_voxel_count;
        return types::VoxelOccupancy::Unmapped;
    }
    [[nodiscard]] types::MapConfig getMapConfig() const override {
        return {};
    }
    [[nodiscard]] bool isInBounds(const Position3D&) const override {
        return true;
    }
    void set(const Position3D&, types::VoxelOccupancy) override {}
    void save(const std::filesystem::path& path) const override {
        ++save_count;
        last_path = path;
    }

    mutable int at_voxel_count = 0;
    mutable int save_count = 0;
    mutable std::filesystem::path last_path{};
};

/**
 * @brief Build a `DroneStepResult` with a given status and optional message.
 * @param status Step status the mocked controller should report.
 * @param message Error message (only meaningful for `Error`).
 * @return The assembled `DroneStepResult`.
 */
[[nodiscard]] types::DroneStepResult stepResult(types::DroneStepStatus status,
                                                std::string message = {}) {
    return types::DroneStepResult{status, std::move(message)};
}

/**
 * @brief Fixture for the MissionControl suite: owns the scripted drone, the fake maps, and the
 *        output path, and builds a `MissionControlImpl` for a given step budget.
 *
 * @note The class name must stay exactly `MissionControl` — a gtest fixture's class name is the
 *       suite name, and the assignment mandates the `MissionControl.*` filter.
 */
class MissionControl : public ::testing::Test {
protected:
    MockDroneControl drone;
    FakeMap hidden;
    FakeMap output;
    std::filesystem::path out{std::filesystem::path(::testing::TempDir()) / "mission_control_out.npy"};

    /**
     * @brief A mission controller with the given step budget over the fixture's collaborators.
     * @param max_steps Maximum number of steps the loop may execute.
     * @return A ready-to-run `MissionControlImpl`.
     */
    [[nodiscard]] MissionControlImpl make(std::size_t max_steps) {
        types::MissionConfigData mission{};
        mission.max_steps = max_steps;
        return MissionControlImpl{mission, types::DroneConfigData{}, hidden, output, drone, out};
    }
};

} // namespace

/**
 * @brief The loop finishes as soon as the drone reports `Completed`.
 *
 * With a generous budget the algorithm works for two steps then reports it is finished; the mission
 * must stop there, count the three executed steps, carry no errors, and save the map exactly once.
 */
TEST_F(MissionControl, CompletesWhenDroneReportsFinished) {
    // Two Continues then a Completed: the completing step is still counted.
    EXPECT_CALL(drone, step())
        .WillOnce(Return(stepResult(types::DroneStepStatus::Continue)))
        .WillOnce(Return(stepResult(types::DroneStepStatus::Continue)))
        .WillOnce(Return(stepResult(types::DroneStepStatus::Completed)));

    MissionControlImpl mission = make(10);
    const types::MissionRunResult result = mission.runMission();

    EXPECT_EQ(result.status, types::MissionRunStatus::Completed);
    EXPECT_EQ(result.steps, 3u); // includes the step that reported Completed
    EXPECT_TRUE(result.errors.empty());
    EXPECT_EQ(output.save_count, 1);  // saved once, after the loop
    EXPECT_EQ(output.last_path, out); // to the configured path
}

/**
 * @brief The loop stops at `max_steps` when the drone never reports completion.
 *
 * The algorithm keeps returning `Continue`, so a budget of two must terminate with `MaxSteps` after
 * exactly two steps, with the map still saved.
 */
TEST_F(MissionControl, ReportsMaxStepsWhenBudgetExhausted) {
    // Budget of 2 caps the loop even though the drone would keep working forever.
    EXPECT_CALL(drone, step())
        .Times(2)
        .WillRepeatedly(Return(stepResult(types::DroneStepStatus::Continue)));

    MissionControlImpl mission = make(2);
    const types::MissionRunResult result = mission.runMission();

    EXPECT_EQ(result.status, types::MissionRunStatus::MaxSteps);
    EXPECT_EQ(result.steps, 2u);
    EXPECT_TRUE(result.errors.empty());
    EXPECT_EQ(output.save_count, 1);
}

/**
 * @brief A fatal drone-step `Error` stops the mission immediately.
 *
 * After one good step the drone reports `Error`; the loop must terminate on the spot (not replan),
 * surface the message under the `DRONE_STEP_ERROR` code, and still save the map. The two-WillOnce
 * cardinality is itself the "stops immediately" assertion — a third step() call would fail the mock.
 */
TEST_F(MissionControl, StopsOnFirstDroneStepError) {
    // Exactly two WillOnce ⇒ cardinality 2: if the loop kept stepping after the Error, gmock would
    // fail on the over-saturated third call, proving we stop immediately.
    EXPECT_CALL(drone, step())
        .WillOnce(Return(stepResult(types::DroneStepStatus::Continue)))
        .WillOnce(Return(stepResult(types::DroneStepStatus::Error, "boom")));

    MissionControlImpl mission = make(10);
    const types::MissionRunResult result = mission.runMission();

    EXPECT_EQ(result.status, types::MissionRunStatus::Error);
    EXPECT_EQ(result.steps, 2u); // the erroring step is counted, then the loop breaks
    ASSERT_EQ(result.errors.size(), 1u);
    EXPECT_EQ(result.errors[0].code, "DRONE_STEP_ERROR");
    EXPECT_EQ(result.errors[0].message, "boom"); // the failing step's message is forwarded verbatim
    EXPECT_EQ(output.save_count, 1);
}

/**
 * @brief A zero-step budget runs no steps but still persists the (empty) map.
 *
 * Edge case: `max_steps == 0` must never call step(), report `MaxSteps` with zero steps, and still
 * save the output map once so downstream scoring always has a file to read.
 */
TEST_F(MissionControl, ZeroBudgetNeverStepsButStillSaves) {
    // A zero budget must never touch the drone controller.
    EXPECT_CALL(drone, step()).Times(0);

    MissionControlImpl mission = make(0);
    const types::MissionRunResult result = mission.runMission();

    EXPECT_EQ(result.status, types::MissionRunStatus::MaxSteps);
    EXPECT_EQ(result.steps, 0u);
    EXPECT_TRUE(result.errors.empty());
    EXPECT_EQ(output.save_count, 1); // saved even with no steps executed
}

/**
 * @brief Completion on the very last budgeted step still reports `Completed`, not `MaxSteps`.
 *
 * Boundary between the two terminal outcomes: a budget of 3 with Completed arriving on step 3 must
 * be a clean completion — exhausting the budget only matters when the drone never completes.
 */
TEST_F(MissionControl, CompletedOnFinalBudgetStep) {
    EXPECT_CALL(drone, step())
        .WillOnce(Return(stepResult(types::DroneStepStatus::Continue)))
        .WillOnce(Return(stepResult(types::DroneStepStatus::Continue)))
        .WillOnce(Return(stepResult(types::DroneStepStatus::Completed)));

    MissionControlImpl mission = make(3);
    const types::MissionRunResult result = mission.runMission();

    EXPECT_EQ(result.status, types::MissionRunStatus::Completed);
    EXPECT_EQ(result.steps, 3u);
}

/**
 * @brief An error on the very first step terminates with exactly one counted step.
 */
TEST_F(MissionControl, ErrorOnFirstStepStopsImmediately) {
    EXPECT_CALL(drone, step())
        .WillOnce(Return(stepResult(types::DroneStepStatus::Error, "dead on arrival")));

    MissionControlImpl mission = make(10);
    const types::MissionRunResult result = mission.runMission();

    EXPECT_EQ(result.status, types::MissionRunStatus::Error);
    EXPECT_EQ(result.steps, 1u);
    ASSERT_EQ(result.errors.size(), 1u);
    EXPECT_EQ(result.errors[0].message, "dead on arrival");
}

/**
 * @brief Even an errored mission saves the output map to the configured path.
 *
 * Downstream scoring/reporting always needs the map file; an error must not skip persistence or
 * redirect it elsewhere.
 */
TEST_F(MissionControl, SavesToConfiguredPathOnError) {
    EXPECT_CALL(drone, step())
        .WillOnce(Return(stepResult(types::DroneStepStatus::Error, "boom")));

    MissionControlImpl mission = make(10);
    (void)mission.runMission();

    EXPECT_EQ(output.save_count, 1);
    EXPECT_EQ(output.last_path, out);
}

/**
 * @brief A single-step budget with an immediately-completing drone is a clean completion.
 */
TEST_F(MissionControl, SingleStepBudgetCompletes) {
    EXPECT_CALL(drone, step()).WillOnce(Return(stepResult(types::DroneStepStatus::Completed)));

    MissionControlImpl mission = make(1);
    const types::MissionRunResult result = mission.runMission();

    EXPECT_EQ(result.status, types::MissionRunStatus::Completed);
    EXPECT_EQ(result.steps, 1u);
    EXPECT_TRUE(result.errors.empty());
}

/**
 * @brief A long budget counts every executed step exactly (no off-by-one drift over many steps).
 */
TEST_F(MissionControl, LongBudgetCountsEveryStep) {
    EXPECT_CALL(drone, step())
        .Times(100)
        .WillRepeatedly(Return(stepResult(types::DroneStepStatus::Continue)));

    MissionControlImpl mission = make(100);
    const types::MissionRunResult result = mission.runMission();

    EXPECT_EQ(result.status, types::MissionRunStatus::MaxSteps);
    EXPECT_EQ(result.steps, 100u);
    EXPECT_TRUE(result.errors.empty());
}

/**
 * @brief MissionControl never reads voxels itself — map queries belong to DroneControl/algorithm.
 *
 * The loop's responsibilities are stepping, termination, and saving; if it starts sampling maps, the
 * architectural boundary has been violated (and a Map3DImpl bug could leak into this suite).
 */
TEST_F(MissionControl, NeverQueriesMaps) {
    EXPECT_CALL(drone, step())
        .WillOnce(Return(stepResult(types::DroneStepStatus::Continue)))
        .WillOnce(Return(stepResult(types::DroneStepStatus::Completed)));

    MissionControlImpl mission = make(10);
    (void)mission.runMission();

    EXPECT_EQ(hidden.at_voxel_count, 0);
    EXPECT_EQ(output.at_voxel_count, 0);
}
