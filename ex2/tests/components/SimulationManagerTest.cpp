#include <drone_mapper/ErrorLogger.h>
#include <drone_mapper/ISimulationRun.h>
#include <drone_mapper/ISimulationRunFactory.h>
#include <drone_mapper/SimulationManager.h>

#include <gtest/gtest.h>

#include <cstddef>
#include <filesystem>
#include <fstream>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace {

using namespace drone_mapper;

// A run that simply returns a preset result — enough to exercise the manager's aggregation.
class FakeRun : public ISimulationRun {
public:
    explicit FakeRun(types::SimulationResult result) : result_(std::move(result)) {}
    [[nodiscard]] types::SimulationResult run() override {
        return result_;
    }

private:
    types::SimulationResult result_;
};

// Factory that counts create() calls and hands back a completed run carrying the given configs.
class CountingFactory : public ISimulationRunFactory {
public:
    [[nodiscard]] std::unique_ptr<ISimulationRun>
    create(const types::SimulationConfigData& simulation, const types::MissionConfigData& mission,
           const types::DroneConfigData&, const types::LidarConfigData&,
           const std::filesystem::path&) override {
        ++create_count;
        types::SimulationResult result{};
        result.simulation_config = simulation;
        result.mission_config = mission;
        result.mission_score = 50.0;
        result.mission_results = {types::MissionRunResult{types::MissionRunStatus::Completed, 5, {}}};
        return std::make_unique<FakeRun>(std::move(result));
    }

    int create_count = 0;
};

// Factory returning a run whose result reports a mission error and an ignored resolution request,
// so the manager's immediate error logging (from the returned result) can be observed.
class ErrorReportingFactory : public ISimulationRunFactory {
public:
    [[nodiscard]] std::unique_ptr<ISimulationRun>
    create(const types::SimulationConfigData& simulation, const types::MissionConfigData& mission,
           const types::DroneConfigData&, const types::LidarConfigData&,
           const std::filesystem::path&) override {
        types::SimulationResult result{};
        result.simulation_config = simulation;
        result.mission_config = mission;
        result.resolution_request_status = types::ResolutionRequestStatus::IgnoredTooSmall;
        result.mission_results = {types::MissionRunResult{
            types::MissionRunStatus::Error, 3, {types::ErrorRef{"DRONE_STEP_ERROR", "hit a wall"}}}};
        result.mission_score = -1.0;
        return std::make_unique<FakeRun>(std::move(result));
    }
};

// Factory that always throws — models a group-level failure (e.g. a bad map file).
class ThrowingFactory : public ISimulationRunFactory {
public:
    [[nodiscard]] std::unique_ptr<ISimulationRun>
    create(const types::SimulationConfigData&, const types::MissionConfigData&,
           const types::DroneConfigData&, const types::LidarConfigData&,
           const std::filesystem::path&) override {
        throw std::runtime_error("bad map file");
    }
};

/**
 * @brief A clean, scored, Completed result (the shape every well-behaved run returns).
 * @return A `SimulationResult` with score 50, one Completed mission, and an Accepted resolution.
 */
[[nodiscard]] types::SimulationResult cleanResult() {
    types::SimulationResult result{};
    result.resolution_request_status = types::ResolutionRequestStatus::Accepted;
    result.mission_score = 50.0;
    result.mission_results = {types::MissionRunResult{types::MissionRunStatus::Completed, 1, {}}};
    return result;
}

// Factory that records the arguments of every create() call and returns clean scored runs — lets a
// test assert exactly what the manager forwards and in which order.
class RecordingFactory : public ISimulationRunFactory {
public:
    struct Call {
        std::size_t mission_max_steps;
        double drone_radius_cm;
        double lidar_z_min_cm;
        std::filesystem::path output_path;
    };

    [[nodiscard]] std::unique_ptr<ISimulationRun>
    create(const types::SimulationConfigData&, const types::MissionConfigData& mission,
           const types::DroneConfigData& drone, const types::LidarConfigData& lidar,
           const std::filesystem::path& output_path) override {
        calls.push_back(Call{mission.max_steps, drone.radius.force_numerical_value_in(cm),
                             lidar.z_min.force_numerical_value_in(cm), output_path});
        return std::make_unique<FakeRun>(cleanResult());
    }

    std::vector<Call> calls;
};

// Factory that throws only on the N-th create() call (1-based) — models a single bad combination in
// the middle of an otherwise healthy batch.
class ThrowOnNthFactory : public ISimulationRunFactory {
public:
    explicit ThrowOnNthFactory(int failing_call) : failing_call_(failing_call) {}

    [[nodiscard]] std::unique_ptr<ISimulationRun>
    create(const types::SimulationConfigData&, const types::MissionConfigData&,
           const types::DroneConfigData&, const types::LidarConfigData&,
           const std::filesystem::path&) override {
        if (++call_index_ == failing_call_) {
            throw std::runtime_error("wiring failed mid-batch");
        }
        return std::make_unique<FakeRun>(cleanResult());
    }

private:
    int failing_call_;
    int call_index_ = 0;
};

// Factory whose runs are entirely clean (no errors, Accepted resolution) — for asserting that a
// healthy batch writes nothing to the error log.
class CleanFactory : public ISimulationRunFactory {
public:
    [[nodiscard]] std::unique_ptr<ISimulationRun>
    create(const types::SimulationConfigData&, const types::MissionConfigData&,
           const types::DroneConfigData&, const types::LidarConfigData&,
           const std::filesystem::path&) override {
        return std::make_unique<FakeRun>(cleanResult());
    }
};

/**
 * @brief A composition with one simulation and the requested counts of missions/drones/lidars.
 * @param missions,drones,lidars Sizes whose product is the expected number of runs.
 * @return A `SimulationCompositionData` of default-constructed configs (values are irrelevant here).
 */
[[nodiscard]] types::SimulationCompositionData makeComposition(std::size_t missions,
                                                               std::size_t drones,
                                                               std::size_t lidars) {
    types::SimulationCompositionData composition{};
    composition.simulation_mission_groups.emplace_back(
        types::SimulationConfigData{}, std::vector<types::MissionConfigData>(missions));
    composition.drones.resize(drones);
    composition.lidars.resize(lidars);
    return composition;
}

/**
 * @brief A fresh temp path (removed if it already exists) for a per-test error-log file.
 */
[[nodiscard]] std::filesystem::path freshTempPath(const std::string& name) {
    const std::filesystem::path path = std::filesystem::path(::testing::TempDir()) / name;
    std::filesystem::remove(path);
    return path;
}

/**
 * @brief Read a whole text file into a string (empty if absent).
 */
[[nodiscard]] std::string readAll(const std::filesystem::path& path) {
    std::ifstream in(path);
    std::ostringstream os;
    os << in.rdbuf();
    return os.str();
}

} // namespace

/**
 * @brief The manager runs the full cartesian product and stamps the report metadata.
 *
 * 1 simulation × 2 missions × 2 drones × 2 lidars must produce 8 runs, with score_range {0,100},
 * error_score -1, and non-empty metric/timestamp.
 */
TEST(SimulationManager, RunsCartesianProductAndStampsMetadata) {
    auto factory = std::make_unique<CountingFactory>();
    CountingFactory* factory_ptr = factory.get();
    ErrorLogger logger; // stderr only
    SimulationManager manager{std::move(factory), logger};

    const types::SimulationManagerReport report =
        manager.run(makeComposition(2, 2, 2), std::filesystem::path{"/tmp"});

    EXPECT_EQ(factory_ptr->create_count, 8);
    EXPECT_EQ(report.runs.size(), 8u);
    EXPECT_DOUBLE_EQ(std::get<0>(report.score_range), 0.0);
    EXPECT_DOUBLE_EQ(std::get<1>(report.score_range), 100.0);
    EXPECT_EQ(report.error_score, -1);
    EXPECT_FALSE(report.metric.empty());
    EXPECT_FALSE(report.generated_at_utc.empty());
}

/**
 * @brief A run that throws is caught, logged, and scored -1 without crashing the batch.
 *
 * With a throwing factory the single combination must yield one errored SimulationResult (score -1,
 * status Error, RUN_FACTORY_ERROR code) rather than propagating the exception.
 */
TEST(SimulationManager, GroupFailureScoresMinusOneAndContinues) {
    auto factory = std::make_unique<ThrowingFactory>();
    ErrorLogger logger; // stderr only
    SimulationManager manager{std::move(factory), logger};

    const types::SimulationManagerReport report =
        manager.run(makeComposition(1, 1, 1), std::filesystem::path{"/tmp"});

    ASSERT_EQ(report.runs.size(), 1u);
    EXPECT_DOUBLE_EQ(report.runs[0].mission_score, -1.0);
    ASSERT_EQ(report.runs[0].mission_results.size(), 1u);
    EXPECT_EQ(report.runs[0].mission_results[0].status, types::MissionRunStatus::Error);
    ASSERT_FALSE(report.runs[0].mission_results[0].errors.empty());
    EXPECT_EQ(report.runs[0].mission_results[0].errors[0].code, "RUN_FACTORY_ERROR");
}

/**
 * @brief A run that reports errors by status has them written to the error log immediately.
 *
 * The manager must surface a completed run's mission-level error (DRONE_STEP_ERROR) and its ignored
 * resolution request to the error log, not just leave them in the returned report.
 */
TEST(SimulationManager, LogsRunErrorsAndIgnoredResolutionToErrorLog) {
    const std::filesystem::path errors_log = freshTempPath("mgr_errors.log");
    ErrorLogger logger{errors_log};
    SimulationManager manager{std::make_unique<ErrorReportingFactory>(), logger};

    const types::SimulationManagerReport report =
        manager.run(makeComposition(1, 1, 1), std::filesystem::path{"/tmp"});

    ASSERT_EQ(report.runs.size(), 1u);
    const std::string text = readAll(errors_log);
    EXPECT_NE(text.find("DRONE_STEP_ERROR"), std::string::npos);
    EXPECT_NE(text.find("hit a wall"), std::string::npos);       // the message is forwarded verbatim
    EXPECT_NE(text.find("RESOLUTION_IGNORED"), std::string::npos);
}

/**
 * @brief An empty composition produces zero runs but a fully stamped report.
 */
TEST(SimulationManager, EmptyCompositionYieldsNoRunsButStampsMetadata) {
    auto factory = std::make_unique<CountingFactory>();
    CountingFactory* factory_ptr = factory.get();
    ErrorLogger logger;
    SimulationManager manager{std::move(factory), logger};

    const types::SimulationManagerReport report =
        manager.run(types::SimulationCompositionData{}, std::filesystem::path{"/tmp"});

    EXPECT_EQ(factory_ptr->create_count, 0);
    EXPECT_TRUE(report.runs.empty());
    EXPECT_FALSE(report.metric.empty());
    EXPECT_FALSE(report.generated_at_utc.empty());
    EXPECT_EQ(report.error_score, -1);
}

/**
 * @brief With no drones the cartesian product is empty: the factory is never asked for a run.
 */
TEST(SimulationManager, NoDronesYieldsNoRuns) {
    auto factory = std::make_unique<CountingFactory>();
    CountingFactory* factory_ptr = factory.get();
    ErrorLogger logger;
    SimulationManager manager{std::move(factory), logger};

    const types::SimulationManagerReport report =
        manager.run(makeComposition(2, 0, 3), std::filesystem::path{"/tmp"});

    EXPECT_EQ(factory_ptr->create_count, 0);
    EXPECT_TRUE(report.runs.empty());
}

/**
 * @brief The manager forwards each combination's configs and the output path to the factory verbatim.
 */
TEST(SimulationManager, ForwardsConfigsAndOutputPathToFactory) {
    types::SimulationCompositionData composition{};
    types::MissionConfigData mission{};
    mission.max_steps = 42;
    composition.simulation_mission_groups.emplace_back(
        types::SimulationConfigData{}, std::vector<types::MissionConfigData>{mission});
    types::DroneConfigData drone{};
    drone.radius = 7.0 * cm;
    composition.drones.push_back(drone);
    types::LidarConfigData lidar{};
    lidar.z_min = 21.0 * cm;
    composition.lidars.push_back(lidar);

    auto factory = std::make_unique<RecordingFactory>();
    RecordingFactory* factory_ptr = factory.get();
    ErrorLogger logger;
    SimulationManager manager{std::move(factory), logger};

    (void)manager.run(composition, std::filesystem::path{"/tmp/fwd_out"});

    ASSERT_EQ(factory_ptr->calls.size(), 1u);
    EXPECT_EQ(factory_ptr->calls[0].mission_max_steps, 42u);
    EXPECT_DOUBLE_EQ(factory_ptr->calls[0].drone_radius_cm, 7.0);
    EXPECT_DOUBLE_EQ(factory_ptr->calls[0].lidar_z_min_cm, 21.0);
    EXPECT_EQ(factory_ptr->calls[0].output_path, std::filesystem::path{"/tmp/fwd_out"});
}

/**
 * @brief Combinations run in the nested-loop order: missions outer, then drones, then lidars.
 *
 * 2 missions (max_steps 1,2) x 2 drones (radius 1,2 cm) must produce the sequence
 * (m1,d1), (m1,d2), (m2,d1), (m2,d2).
 */
TEST(SimulationManager, RunsFollowNestedLoopOrder) {
    types::SimulationCompositionData composition{};
    types::MissionConfigData mission1{};
    mission1.max_steps = 1;
    types::MissionConfigData mission2{};
    mission2.max_steps = 2;
    composition.simulation_mission_groups.emplace_back(
        types::SimulationConfigData{}, std::vector<types::MissionConfigData>{mission1, mission2});
    for (const double radius_cm : {1.0, 2.0}) {
        types::DroneConfigData drone{};
        drone.radius = radius_cm * cm;
        composition.drones.push_back(drone);
    }
    composition.lidars.resize(1);

    auto factory = std::make_unique<RecordingFactory>();
    RecordingFactory* factory_ptr = factory.get();
    ErrorLogger logger;
    SimulationManager manager{std::move(factory), logger};

    (void)manager.run(composition, std::filesystem::path{"/tmp"});

    ASSERT_EQ(factory_ptr->calls.size(), 4u);
    EXPECT_EQ(factory_ptr->calls[0].mission_max_steps, 1u);
    EXPECT_DOUBLE_EQ(factory_ptr->calls[0].drone_radius_cm, 1.0);
    EXPECT_EQ(factory_ptr->calls[1].mission_max_steps, 1u);
    EXPECT_DOUBLE_EQ(factory_ptr->calls[1].drone_radius_cm, 2.0);
    EXPECT_EQ(factory_ptr->calls[2].mission_max_steps, 2u);
    EXPECT_DOUBLE_EQ(factory_ptr->calls[2].drone_radius_cm, 1.0);
    EXPECT_EQ(factory_ptr->calls[3].mission_max_steps, 2u);
    EXPECT_DOUBLE_EQ(factory_ptr->calls[3].drone_radius_cm, 2.0);
}

/**
 * @brief A failure in the middle of the batch scores that run -1; the rest still run and score.
 *
 * 1 mission x 2 drones x 2 lidars = 4 combinations; the factory throws only on call 2. The batch
 * must not stop: 4 results, exactly the second errored.
 */
TEST(SimulationManager, ContinuesAfterMidBatchFailure) {
    auto factory = std::make_unique<ThrowOnNthFactory>(2);
    ErrorLogger logger;
    SimulationManager manager{std::move(factory), logger};

    const types::SimulationManagerReport report =
        manager.run(makeComposition(1, 2, 2), std::filesystem::path{"/tmp"});

    ASSERT_EQ(report.runs.size(), 4u);
    EXPECT_DOUBLE_EQ(report.runs[0].mission_score, 50.0);
    EXPECT_DOUBLE_EQ(report.runs[1].mission_score, -1.0); // the failed combination
    EXPECT_DOUBLE_EQ(report.runs[2].mission_score, 50.0);
    EXPECT_DOUBLE_EQ(report.runs[3].mission_score, 50.0);
    ASSERT_FALSE(report.runs[1].mission_results.empty());
    EXPECT_EQ(report.runs[1].mission_results.front().status, types::MissionRunStatus::Error);
}

/**
 * @brief Clean, accepted runs write nothing to the error log.
 *
 * The manager logs mission errors and ignored resolutions the moment a run returns; a batch of
 * error-free Accepted runs must therefore leave the error log empty.
 */
TEST(SimulationManager, CleanRunsLogNothing) {
    const std::filesystem::path errors_log = freshTempPath("mgr_clean_errors.log");
    ErrorLogger logger{errors_log};
    SimulationManager manager{std::make_unique<CleanFactory>(), logger};

    const types::SimulationManagerReport report =
        manager.run(makeComposition(1, 1, 1), std::filesystem::path{"/tmp"});

    ASSERT_EQ(report.runs.size(), 1u);
    EXPECT_TRUE(readAll(errors_log).empty()); // no errors, no resolution notices
}