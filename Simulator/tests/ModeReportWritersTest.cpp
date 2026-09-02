/**
 * @file ModeReportWritersTest.cpp
 * @brief Coverage of the ranking, the behavioural grouping, and what lands under `errors:`.
 * @note Reports are built by hand: the writers take results rather than fetching them, so nothing
 *       here needs a plugin, a map, or a mission. Assertions re-parse the written file rather than
 *       matching emitted text.
 */

#include <Simulator/ModeReportWriters.h>

#include <yaml-cpp/yaml.h>

#include <gtest/gtest.h>

#include <cstdlib>
#include <filesystem>
#include <string>
#include <vector>

namespace {

namespace fs = std::filesystem;

/**
 * @brief Build one run result.
 * @param score Score to report; negative marks it errored.
 * @param steps Steps the mission took.
 * @return The result.
 */
[[nodiscard]] simulator::types::SimulationResult run(double score, std::size_t steps) {
    simulator::types::SimulationResult result{};
    result.mission_score = score;

    common::types::MissionRunResult mission{};
    mission.steps = steps;
    mission.status = score < 0.0 ? common::types::MissionRunStatus::Error
                                 : common::types::MissionRunStatus::Completed;
    result.mission_results = {mission};
    return result;
}

/**
 * @brief Build one plugin's outcome.
 * @param name The plugin's filename.
 * @param runs Its runs, in expansion order.
 * @return The outcome.
 */
[[nodiscard]] simulator::PluginOutcome outcome(
    const std::string& name, std::vector<simulator::types::SimulationResult> runs) {
    simulator::PluginOutcome entry{};
    entry.name = name;
    entry.report.runs = std::move(runs);
    return entry;
}

/**
 * @brief Gives each test its own scratch directory.
 */
class ModeReportWritersTest : public ::testing::Test {
protected:
    /**
     * @brief Create a uniquely named scratch directory.
     */
    void SetUp() override {
        const ::testing::TestInfo* const info =
            ::testing::UnitTest::GetInstance()->current_test_info();
        dir_ = fs::temp_directory_path() /
               ("ex3_mode_" + std::string{info->name()} + "_" + std::to_string(::getpid()));
        std::error_code ec;
        fs::remove_all(dir_, ec);
        fs::create_directories(dir_, ec);

        input_.composition_file = "inputs/sim_compose.yaml";
        input_.fixed_plugin_file = "plugins/Fixed.so";
        input_.varied_plugin_folder = "plugins/varied";
    }

    /**
     * @brief Remove the scratch directory.
     */
    void TearDown() override {
        std::error_code ec;
        fs::remove_all(dir_, ec);
    }

    /**
     * @brief Write the competitive report and read it back.
     * @return Its `competitive_report` node.
     */
    [[nodiscard]] YAML::Node writeCompetitive() {
        simulator::writeCompetitiveReport(input_, dir_);
        const fs::path file = dir_ / "competitive_report.yaml";
        EXPECT_TRUE(fs::exists(file));
        return YAML::LoadFile(file.string())["competitive_report"];
    }

    /**
     * @brief Write the comparative report and read it back.
     * @return Its `comparative_report` node.
     */
    [[nodiscard]] YAML::Node writeComparative() {
        simulator::writeComparativeReport(input_, dir_);
        const fs::path file = dir_ / "comparative_report.yaml";
        EXPECT_TRUE(fs::exists(file));
        return YAML::LoadFile(file.string())["comparative_report"];
    }

    simulator::ModeReportInput input_{};
    fs::path dir_{};
};

/**
 * @brief Competitive ranking is by total score, highest first.
 */
TEST_F(ModeReportWritersTest, CompetitiveRanksByScoreDescending) {
    input_.outcomes = {outcome("low.so", {run(10.0, 5)}), outcome("high.so", {run(90.0, 5)}),
                       outcome("mid.so", {run(50.0, 5)})};

    const YAML::Node summary = writeCompetitive()["results_summary"];

    ASSERT_EQ(summary.size(), 3u);
    EXPECT_EQ(summary[0]["algorithm"].as<std::string>(), "high.so");
    EXPECT_EQ(summary[1]["algorithm"].as<std::string>(), "mid.so");
    EXPECT_EQ(summary[2]["algorithm"].as<std::string>(), "low.so");
    EXPECT_DOUBLE_EQ(summary[0]["total_score"].as<double>(), 90.0);
}

/**
 * @brief A tie on score is broken by fewer steps, so the faster plugin wins.
 */
TEST_F(ModeReportWritersTest, CompetitiveBreaksTiesByFewerSteps) {
    /**
     * @note The one place a slower plugin loses to an equally accurate faster one. Steps do no
     *       ranking work otherwise.
     */
    input_.outcomes = {outcome("slow.so", {run(50.0, 900)}), outcome("fast.so", {run(50.0, 100)})};

    const YAML::Node summary = writeCompetitive()["results_summary"];

    ASSERT_EQ(summary.size(), 2u);
    EXPECT_EQ(summary[0]["algorithm"].as<std::string>(), "fast.so");
    EXPECT_EQ(summary[0]["total_steps"].as<std::size_t>(), 100u);
}

/**
 * @brief Totals include the -1 sentinel rather than skipping failed runs.
 */
TEST_F(ModeReportWritersTest, TotalsSumEveryRunIncludingFailures) {
    /**
     * @note A plugin that crashed on one run of three must rank below one that completed all three,
     *       so the sentinel is summed rather than filtered out.
     */
    input_.outcomes = {outcome("partial.so", {run(40.0, 10), run(-1.0, 0), run(40.0, 10)})};

    const YAML::Node summary = writeCompetitive()["results_summary"];

    ASSERT_EQ(summary.size(), 1u);
    EXPECT_DOUBLE_EQ(summary[0]["total_score"].as<double>(), 79.0);
    EXPECT_EQ(summary[0]["total_steps"].as<std::size_t>(), 20u);
}

/**
 * @brief A plugin that never loaded is named under `errors:` and absent from the ranking.
 * @note It has no report to summarise, so naming it here is the only way the document accounts for
 *       a folder entry that was asked for and produced nothing.
 */
TEST_F(ModeReportWritersTest, APluginThatCouldNotLoadIsNamedUnderErrors) {
    input_.failed_to_load = {"NotAPlugin.so"};
    input_.outcomes = {outcome("good.so", {run(50.0, 5)})};

    const YAML::Node report = writeCompetitive();

    ASSERT_EQ(report["results_summary"].size(), 1u);
    EXPECT_EQ(report["results_summary"][0]["algorithm"].as<std::string>(), "good.so");
    ASSERT_EQ(report["errors"].size(), 1u);
    EXPECT_EQ(report["errors"][0].as<std::string>(), "NotAPlugin.so");
}

/**
 * @brief A plugin that loaded but failed every run is also moved to `errors:`, not ranked last.
 */
TEST_F(ModeReportWritersTest, APluginWhoseEveryRunFailedIsNamedUnderErrors) {
    /**
     * @note It did not score poorly; it did not function. Ranking it last would suggest it merely
     *       performed badly.
     */
    input_.outcomes = {outcome("broken.so", {run(-1.0, 0), run(-1.0, 0)}),
                       outcome("good.so", {run(50.0, 5)})};

    const YAML::Node report = writeCompetitive();

    ASSERT_EQ(report["results_summary"].size(), 1u);
    EXPECT_EQ(report["results_summary"][0]["algorithm"].as<std::string>(), "good.so");
    ASSERT_EQ(report["errors"].size(), 1u);
    EXPECT_EQ(report["errors"][0].as<std::string>(), "broken.so");
}

/**
 * @brief A plugin that failed only some runs stays in the ranking, carrying its sentinels.
 * @note The boundary of the previous rule: partial failure is a bad result, total failure is not a
 *       result at all, and only the latter leaves the ranking.
 */
TEST_F(ModeReportWritersTest, APluginWithSomeFailedRunsStaysRanked) {
    input_.outcomes = {outcome("partial.so", {run(-1.0, 0), run(50.0, 5)})};

    const YAML::Node report = writeCompetitive();

    EXPECT_EQ(report["results_summary"].size(), 1u);
    EXPECT_EQ(report["errors"].size(), 0u);
}

/**
 * @brief Plugins whose runs match one for one are grouped, largest group first.
 */
TEST_F(ModeReportWritersTest, ComparativeGroupsPluginsThatMatchRunByRun) {
    input_.outcomes = {outcome("a.so", {run(10.0, 5), run(20.0, 7)}),
                       outcome("b.so", {run(10.0, 5), run(20.0, 7)}),
                       outcome("c.so", {run(99.0, 1), run(1.0, 1)})};

    const YAML::Node summary = writeComparative()["results_summary"];

    ASSERT_EQ(summary.size(), 2u);
    ASSERT_EQ(summary[0]["same_results"].size(), 2u) << "larger groups come first";
    EXPECT_EQ(summary[0]["same_results"][0].as<std::string>(), "a.so");
    EXPECT_EQ(summary[0]["same_results"][1].as<std::string>(), "b.so");
    EXPECT_EQ(summary[1]["same_results"][0].as<std::string>(), "c.so");
}

/**
 * @brief Two plugins with equal totals but different per-run results are kept apart.
 */
TEST_F(ModeReportWritersTest, AnEqualTotalIsNotEnoughToShareAGroup) {
    /**
     * @note The assignment's own counterexample: its sample shows `total_score: 495` in two
     *       different groups. Both plugins here total 30 over two runs, but they reached it
     *       differently, so a score-only implementation would wrongly merge them.
     */
    input_.outcomes = {outcome("a.so", {run(10.0, 5), run(20.0, 5)}),
                       outcome("b.so", {run(20.0, 5), run(10.0, 5)})};

    const YAML::Node summary = writeComparative()["results_summary"];

    ASSERT_EQ(summary.size(), 2u);
    EXPECT_EQ(summary[0]["same_results"].size(), 1u);
    EXPECT_EQ(summary[1]["same_results"].size(), 1u);
    EXPECT_DOUBLE_EQ(summary[0]["total_score"].as<double>(), 30.0);
    EXPECT_DOUBLE_EQ(summary[1]["total_score"].as<double>(), 30.0);
}

/**
 * @brief Identical scores with different step counts are still different behaviour.
 */
TEST_F(ModeReportWritersTest, DifferingStepsAloneSeparateTwoPlugins) {
    /**
     * @note Exactly the shipped fixtures: identical maps and identical scores, 1 step versus 2. If
     *       steps were dropped from the key these would merge, and comparative mode would report two
     *       demonstrably different mission controls as identical.
     */
    input_.outcomes = {outcome("a.so", {run(10.0, 1)}), outcome("b.so", {run(10.0, 2)})};

    const YAML::Node summary = writeComparative()["results_summary"];

    EXPECT_EQ(summary.size(), 2u);
}

/**
 * @brief Two writes of the same data produce byte-identical grouping.
 */
TEST_F(ModeReportWritersTest, GroupingIsReproducibleAcrossWrites) {
    /**
     * @note Comparative mode exists to answer whether two plugins agree. A report that ordered
     *       itself differently between identical runs would undermine that, and the failure would be
     *       far harder to see once execution is concurrent.
     */
    input_.outcomes = {outcome("z.so", {run(10.0, 5)}), outcome("a.so", {run(20.0, 5)}),
                       outcome("m.so", {run(10.0, 5)})};

    simulator::writeComparativeReport(input_, dir_);
    const std::string first = YAML::Dump(YAML::LoadFile((dir_ / "comparative_report.yaml").string())
                                             ["comparative_report"]["results_summary"]);

    simulator::writeComparativeReport(input_, dir_);
    const std::string second = YAML::Dump(YAML::LoadFile((dir_ / "comparative_report.yaml").string())
                                              ["comparative_report"]["results_summary"]);

    EXPECT_EQ(first, second);
}

/**
 * @brief The comparative report names the composition, the varied folder and the fixed plugin.
 * @note The fixed plugin appears as a filename rather than a path, so a report is comparable between
 *       machines that keep their plugins in different places.
 */
TEST_F(ModeReportWritersTest, ComparativeCarriesItsIdentifyingPaths) {
    input_.outcomes = {outcome("a.so", {run(10.0, 5)})};

    const YAML::Node report = writeComparative();

    EXPECT_EQ(report["composition_file"].as<std::string>(), "inputs/sim_compose.yaml");
    EXPECT_EQ(report["mission_control_folder"].as<std::string>(), "plugins/varied");
    EXPECT_EQ(report["algorithm"].as<std::string>(), "Fixed.so") << "filename, not path";
    EXPECT_FALSE(report["generated_at_utc"].as<std::string>().empty());
}

/**
 * @brief The competitive report names the same three things under its own mode's keys.
 * @note The keys differ per mode - `mission_control` and `algorithms_folder` here - because which
 *       side is fixed and which is varied is exactly what distinguishes the two modes.
 */
TEST_F(ModeReportWritersTest, CompetitiveCarriesItsIdentifyingPaths) {
    input_.outcomes = {outcome("a.so", {run(10.0, 5)})};

    const YAML::Node report = writeCompetitive();

    EXPECT_EQ(report["mission_control"].as<std::string>(), "Fixed.so");
    EXPECT_EQ(report["algorithms_folder"].as<std::string>(), "plugins/varied");
}

/**
 * @brief A mode that ran nothing still produces a complete, parseable document.
 */
TEST_F(ModeReportWritersTest, AnEmptyModeStillProducesAReadableDocument) {
    const YAML::Node report = writeCompetitive();

    ASSERT_TRUE(report);
    EXPECT_EQ(report["results_summary"].size(), 0u);
    EXPECT_EQ(report["errors"].size(), 0u);
}

/**
 * @brief Empty lists emit as sequences, never as YAML null.
 */
TEST_F(ModeReportWritersTest, EmptyListsAreSequencesNotNull) {
    /**
     * @note A default-constructed node that never receives an element emits as `~`, which parses
     *       back as null rather than as an empty list. The successful run - nothing under `errors:` -
     *       is the common case, so it is the one that must not force a reader to special-case the
     *       type.
     */
    input_.outcomes = {outcome("good.so", {run(50.0, 5)})};

    const YAML::Node report = writeCompetitive();

    EXPECT_TRUE(report["errors"].IsSequence());
    EXPECT_TRUE(report["results_summary"].IsSequence());
}

} // namespace
