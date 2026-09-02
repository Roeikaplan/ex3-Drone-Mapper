/**
 * @file SimulationOutputWriterTest.cpp
 * @brief Coverage of the score report's structure, statistics, labelling, and degraded mode.
 * @note Reports are built by hand rather than produced by a pipeline: the writer takes a plain
 *       `SimulationManagerReport`, so nothing here needs a plugin, a map, or a mission.
 * @note Assertions read the file back through yaml-cpp rather than matching emitted text. Key order
 *       and quoting are yaml-cpp's business, and a test that pins them breaks on the first upgrade.
 */

#include <Simulator/SimulationOutputWriter.h>

#include <yaml-cpp/yaml.h>

#include <gtest/gtest.h>

#include <cstdlib>
#include <filesystem>
#include <string>

namespace {

namespace fs = std::filesystem;

/**
 * @brief Build one run result.
 * @param score Score to report; negative marks it as errored.
 * @param steps Steps the mission took.
 * @param output_map Filename the run produced.
 * @param resolution_cm Output map resolution in centimetres.
 * @return The assembled result.
 */
[[nodiscard]] simulator::types::SimulationResult makeRun(double score, std::size_t steps,
                                                         const std::string& output_map,
                                                         double resolution_cm) {
    simulator::types::SimulationResult result{};
    result.mission_score = score;
    result.output_map_file = output_map;
    result.output_map_config.resolution = resolution_cm * common::cm;
    result.resolution_request_status = simulator::types::ResolutionRequestStatus::Accepted;

    common::types::MissionRunResult mission{};
    mission.steps = steps;
    if (score < 0.0) {
        mission.status = common::types::MissionRunStatus::Error;
        mission.errors.push_back(common::types::ErrorRef{"RUN_FAILED", "deliberate"});
    } else {
        mission.status = common::types::MissionRunStatus::Completed;
    }
    result.mission_results = {mission};
    return result;
}

/**
 * @brief Paths describing one simulation, one mission, two drones, and one lidar.
 * @return Two runs' worth of labelling.
 */
[[nodiscard]] simulator::CompositionPaths twoRunPaths() {
    simulator::CompositionPaths paths;
    paths.simulation_paths = {"simulation/house.yaml"};
    paths.mission_paths = {{"mission/lower.yaml"}};
    paths.drone_paths = {"drone/small.yaml", "drone/large.yaml"};
    paths.lidar_paths = {"lidar/long.yaml"};
    return paths;
}

/**
 * @brief Gives each test its own scratch directory.
 */
class SimulationOutputWriterTest : public ::testing::Test {
protected:
    /**
     * @brief Create a uniquely named scratch directory under the system temp folder.
     */
    void SetUp() override {
        const ::testing::TestInfo* const info =
            ::testing::UnitTest::GetInstance()->current_test_info();
        dir_ = fs::temp_directory_path() /
               ("ex3_report_" + std::string{info->name()} + "_" + std::to_string(::getpid()));
        std::error_code ec;
        fs::remove_all(dir_, ec);
        fs::create_directories(dir_, ec);

        report_.composition_file = "inputs/sim_compose.yaml";
        report_.generated_at_utc = "2026-08-22T10:00:00Z";
        report_.metric = "occupied_voxel_iou";
        report_.score_range = {0.0, 100.0};
        report_.error_score = -1;
    }

    /**
     * @brief Remove the scratch directory.
     */
    void TearDown() override {
        std::error_code ec;
        fs::remove_all(dir_, ec);
    }

    /**
     * @brief Write the report and read it back.
     * @param paths Labelling to use.
     * @return The `score_report` node of the re-parsed document.
     */
    [[nodiscard]] YAML::Node writeAndReload(const simulator::CompositionPaths& paths) {
        simulator::writeSimulationOutput(report_, dir_, "PluginX", paths);
        const fs::path file = dir_ / "PluginX__simulation_output.yaml";
        EXPECT_TRUE(fs::exists(file)) << file.string();
        return YAML::LoadFile(file.string())["score_report"];
    }

    simulator::types::SimulationManagerReport report_{};
    fs::path dir_{};
};

/**
 * @brief The document has a single `score_report` root carrying the run's metadata.
 * @note `metric`, `score_range` and `error_score` are written so the file explains its own numbers -
 *       a reader can tell -1 is a sentinel without knowing the simulator's conventions.
 */
TEST_F(SimulationOutputWriterTest, EmitsTheMetadataUnderOneRoot) {
    report_.runs = {makeRun(50.0, 10, "a.npy", 5.0), makeRun(70.0, 20, "b.npy", 5.0)};

    const YAML::Node node = writeAndReload(twoRunPaths());

    ASSERT_TRUE(node) << "the document must have a score_report root";
    EXPECT_EQ(node["plugin"].as<std::string>(), "PluginX");
    EXPECT_EQ(node["composition_file"].as<std::string>(), "inputs/sim_compose.yaml");
    EXPECT_EQ(node["generated_at_utc"].as<std::string>(), "2026-08-22T10:00:00Z");
    EXPECT_EQ(node["metric"].as<std::string>(), "occupied_voxel_iou");
    EXPECT_DOUBLE_EQ(node["score_range"]["max"].as<double>(), 100.0);
    EXPECT_EQ(node["error_score"].as<int>(), -1);
}

/**
 * @brief Summary statistics are computed over scored runs only, while the counts report all of them.
 */
TEST_F(SimulationOutputWriterTest, SummaryStatisticsExcludeTheErrorSentinel) {
    /**
     * @note Two scored runs at 40 and 60 plus two failures. The average must be 50, not 24.5 - a
     *       -1 folded in would sit outside the score_range this same document declares.
     */
    simulator::CompositionPaths paths = twoRunPaths();
    paths.drone_paths = {"drone/a.yaml", "drone/b.yaml"};
    paths.lidar_paths = {"lidar/a.yaml", "lidar/b.yaml"};
    report_.runs = {makeRun(40.0, 1, "a.npy", 5.0), makeRun(60.0, 2, "b.npy", 5.0),
                    makeRun(-1.0, 0, "c.npy", 0.0), makeRun(-1.0, 0, "d.npy", 0.0)};

    const YAML::Node summary = writeAndReload(paths)["summary"];

    EXPECT_EQ(summary["total_runs"].as<std::size_t>(), 4u);
    EXPECT_EQ(summary["scored_runs"].as<std::size_t>(), 2u);
    EXPECT_EQ(summary["error_runs"].as<std::size_t>(), 2u);
    EXPECT_DOUBLE_EQ(summary["average_score"].as<double>(), 50.0);
    EXPECT_DOUBLE_EQ(summary["min_score"].as<double>(), 40.0);
    EXPECT_DOUBLE_EQ(summary["max_score"].as<double>(), 60.0);
}

/**
 * @brief Runs nest under their simulation and mission, each labelled by its source YAML file.
 * @note The path-based labelling the whole `CompositionPaths` side-channel exists to supply, since a
 *       filename cannot be recovered from a parsed config struct.
 */
TEST_F(SimulationOutputWriterTest, RunsAreNestedAndLabelledBySourceFile) {
    report_.runs = {makeRun(50.0, 10, "a.npy", 5.0), makeRun(70.0, 20, "b.npy", 5.0)};

    const YAML::Node simulations = writeAndReload(twoRunPaths())["simulations"];

    ASSERT_EQ(simulations.size(), 1u);
    EXPECT_EQ(simulations[0]["simulation_config"].as<std::string>(), "simulation/house.yaml");

    const YAML::Node missions = simulations[0]["missions"];
    ASSERT_EQ(missions.size(), 1u);
    EXPECT_EQ(missions[0]["mission_config"].as<std::string>(), "mission/lower.yaml");

    const YAML::Node runs = missions[0]["runs"];
    ASSERT_EQ(runs.size(), 2u);
    EXPECT_EQ(runs[0]["drone_config"].as<std::string>(), "drone/small.yaml");
    EXPECT_EQ(runs[1]["drone_config"].as<std::string>(), "drone/large.yaml");
    EXPECT_EQ(runs[0]["lidar_config"].as<std::string>(), "lidar/long.yaml");
    EXPECT_EQ(runs[0]["status"].as<std::string>(), "completed");
    EXPECT_EQ(runs[0]["steps"].as<std::size_t>(), 10u);
    EXPECT_DOUBLE_EQ(runs[1]["score"].as<double>(), 70.0);
}

/**
 * @brief The nesting holds across several simulations, each with its own mission list.
 * @note Also pins the consumption order: runs are taken in the manager's expansion order, which is
 *       the unverifiable contract the positional labelling rests on.
 */
TEST_F(SimulationOutputWriterTest, NestingSpansSeveralSimulationsAndMissions) {
    simulator::CompositionPaths paths;
    paths.simulation_paths = {"simulation/a.yaml", "simulation/b.yaml"};
    paths.mission_paths = {{"mission/a1.yaml", "mission/a2.yaml"},
                           {"mission/b1.yaml", "mission/b2.yaml"}};
    paths.drone_paths = {"drone/one.yaml", "drone/two.yaml"};
    paths.lidar_paths = {"lidar/one.yaml"};

    for (int i = 0; i < 8; ++i) {
        report_.runs.push_back(makeRun(static_cast<double>(i), 1, "m.npy", 5.0));
    }

    const YAML::Node simulations = writeAndReload(paths)["simulations"];

    ASSERT_EQ(simulations.size(), 2u);
    ASSERT_EQ(simulations[0]["missions"].size(), 2u);
    ASSERT_EQ(simulations[0]["missions"][0]["runs"].size(), 2u);
    EXPECT_EQ(simulations[1]["missions"][1]["mission_config"].as<std::string>(),
              "mission/b2.yaml");
    EXPECT_DOUBLE_EQ(simulations[1]["missions"][1]["runs"][1]["score"].as<double>(), 7.0)
        << "runs are consumed in the manager's expansion order";
}

/**
 * @brief A mission's reported resolution is taken from a run that actually scored.
 */
TEST_F(SimulationOutputWriterTest, MissionResolutionComesFromAScoredRun) {
    /**
     * @note The first run of this mission failed and therefore carries a default map config with
     *       resolution 0. Reporting that as the mission's resolution would be wrong, so the scored
     *       run is preferred.
     */
    report_.runs = {makeRun(-1.0, 0, "a.npy", 0.0), makeRun(80.0, 5, "b.npy", 5.0)};

    const YAML::Node mission = writeAndReload(twoRunPaths())["simulations"][0]["missions"][0];

    EXPECT_DOUBLE_EQ(mission["resolution_cm"].as<double>(), 5.0);
    EXPECT_EQ(mission["resolution_request_status"].as<std::string>(), "ACCEPTED");
}

/**
 * @brief A failed run still appears, with its sentinel score and the code that explains it.
 * @note Dropping it would leave the report describing fewer runs than the composition asked for,
 *       which reads as though the combination was never requested.
 */
TEST_F(SimulationOutputWriterTest, ErroredRunsAppearWithTheirCode) {
    report_.runs = {makeRun(-1.0, 0, "a.npy", 0.0), makeRun(80.0, 5, "b.npy", 5.0)};

    const YAML::Node runs = writeAndReload(twoRunPaths())["simulations"][0]["missions"][0]["runs"];

    ASSERT_EQ(runs.size(), 2u) << "a failed run is reported, never dropped";
    EXPECT_EQ(runs[0]["status"].as<std::string>(), "error");
    EXPECT_DOUBLE_EQ(runs[0]["score"].as<double>(), -1.0);
    EXPECT_EQ(runs[0]["error_ref"]["code"].as<std::string>(), "RUN_FAILED");
}

/**
 * @brief When the paths do not describe the runs, the writer emits a flat list instead of guessing.
 */
TEST_F(SimulationOutputWriterTest, MismatchedPathsFallBackToAFlatListRatherThanWrongLabels) {
    /**
     * @note The paths describe two runs but the report holds three. Labelling positionally would
     *       attribute runs to configs they did not use, so the writer degrades to a flat list where
     *       each run is identified by the output map it produced.
     */
    report_.runs = {makeRun(10.0, 1, "a.npy", 5.0), makeRun(20.0, 2, "b.npy", 5.0),
                    makeRun(30.0, 3, "c.npy", 5.0)};

    const YAML::Node node = writeAndReload(twoRunPaths());

    EXPECT_FALSE(node["simulations"]);
    ASSERT_TRUE(node["runs"]);
    ASSERT_EQ(node["runs"].size(), 3u) << "every run still appears";
    EXPECT_EQ(node["runs"][2]["output_map"].as<std::string>(), "c.npy");
}

/**
 * @brief An empty paths structure takes the same fallback as a mismatched one.
 * @note The case component tests hit, since `loadComposition` may be called without a paths sink at
 *       all - the writer must not require the side-channel to exist.
 */
TEST_F(SimulationOutputWriterTest, EmptyPathsAlsoDegradeGracefully) {
    report_.runs = {makeRun(10.0, 1, "a.npy", 5.0)};

    const YAML::Node node = writeAndReload(simulator::CompositionPaths{});

    ASSERT_TRUE(node["runs"]);
    EXPECT_EQ(node["runs"].size(), 1u);
}

} // namespace
