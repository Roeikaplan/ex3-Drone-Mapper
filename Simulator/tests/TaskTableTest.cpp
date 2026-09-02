/**
 * @file TaskTableTest.cpp
 * @brief The properties concurrent execution will rest on: disjoint slots, index-addressed results,
 *        and contiguous per-plugin ranges.
 * @note These cases run no real mission and read no map file. The factory interface is the seam, so
 *       a fake implementation of it is enough to exercise enumeration, dispatch, and slicing on their
 *       own - which is the point of having split them apart.
 */

#include <Simulator/SimulationManager.h>
#include <Simulator/SimulationTaskTable.h>
#include <Simulator/TaskExecutor.h>

#include <gtest/gtest.h>

#include <atomic>
#include <cstddef>
#include <memory>
#include <string>
#include <vector>

namespace {

using common::cm;

/**
 * @brief A run that reports the order in which it was executed.
 * @note `steps` carries a completion sequence number rather than anything mission-like. That is what
 *       makes the difference between *completion* order and *index* order observable.
 */
class OrderStampingRun final : public simulator::ISimulationRun {
public:
    /**
     * @brief Construct over a shared counter.
     * @param counter Incremented once per run; its prior value is the stamp.
     * @param mission The mission config the cell named, echoed back for identity checks.
     */
    OrderStampingRun(std::atomic<std::size_t>& counter,
                     const common::types::MissionConfigData& mission)
        : counter_(counter), mission_(mission) {}

    /**
     * @brief Execute the run.
     * @return A result stamped with this run's completion position.
     */
    [[nodiscard]] simulator::types::SimulationResult run() override {
        simulator::types::SimulationResult result{};
        result.mission_config = mission_;
        result.resolution_request_status = simulator::types::ResolutionRequestStatus::Accepted;
        result.mission_results = {common::types::MissionRunResult{
            common::types::MissionRunStatus::Completed, counter_.fetch_add(1), {}}};
        return result;
    }

private:
    std::atomic<std::size_t>& counter_;
    common::types::MissionConfigData mission_;
};

/**
 * @brief A factory that builds `OrderStampingRun`s and remembers nothing else.
 */
class OrderStampingFactory final : public simulator::ISimulationRunFactory {
public:
    /**
     * @brief Construct over a shared counter.
     * @param counter Passed to every run this factory creates.
     */
    explicit OrderStampingFactory(std::atomic<std::size_t>& counter) : counter_(counter) {}

    /**
     * @brief Build one run.
     * @param mission The mission config of the combination.
     * @return A run that stamps its completion order.
     */
    [[nodiscard]] std::unique_ptr<simulator::ISimulationRun> create(
        const simulator::types::SimulationConfigData&,
        const common::types::MissionConfigData& mission, const common::types::DroneConfigData&,
        const common::types::LidarConfigData&, const std::filesystem::path&) override {
        return std::make_unique<OrderStampingRun>(counter_, mission);
    }

private:
    std::atomic<std::size_t>& counter_;
};

/**
 * @brief An executor that runs the body from the last index down to the first.
 * @note Stands in for any scheduler whose completion order is not index order - which is every
 *       concurrent one. If the reports survive this, they survive a thread pool.
 */
class ReverseExecutor final : public simulator::ITaskExecutor {
public:
    /**
     * @brief Invoke the body once per index, in descending order.
     * @param count How many indices.
     * @param body What to run.
     */
    void forEach(std::size_t count, const std::function<void(std::size_t)>& body) override {
        for (std::size_t i = count; i > 0; --i) {
            body(i - 1);
        }
    }
};

/**
 * @brief A composition with distinguishable cells and no files behind it.
 * @return One simulation with two missions, crossed with two drones and one lidar: four cells.
 * @note The missions differ in `max_steps` only, which is enough to tell one cell's result from
 *       another's without touching the filesystem.
 */
[[nodiscard]] simulator::types::SimulationCompositionData tinyComposition() {
    simulator::types::SimulationCompositionData composition{};
    composition.composition_file = "tiny.yaml";

    simulator::types::SimulationConfigData simulation{};
    simulation.map_resolution = 10.0 * cm;

    common::types::MissionConfigData first{};
    first.max_steps = 100;
    common::types::MissionConfigData second{};
    second.max_steps = 200;

    composition.simulation_mission_groups.emplace_back(
        simulation, std::vector<common::types::MissionConfigData>{first, second});
    composition.drone_configs.resize(2);
    composition.lidar_configs.resize(1);
    return composition;
}

/**
 * @brief Two plugins enumerating into one table each own a contiguous, correctly-tagged block.
 * @note Contiguity is what makes `resultsForPlugin` a slice rather than a filter, and the
 *       `plugin_index` tag is what lets one flat dispatch serve every plugin at once.
 */
TEST(SimulationTaskTable, EachPluginOwnsAContiguousRangeOfCells) {
    std::atomic<std::size_t> counter{0};
    simulator::ErrorLogger logger;

    simulator::SimulationManager first{std::make_unique<OrderStampingFactory>(counter), "PluginA",
                                       logger};
    simulator::SimulationManager second{std::make_unique<OrderStampingFactory>(counter), "PluginB",
                                        logger};

    const simulator::types::SimulationCompositionData composition = tinyComposition();
    simulator::SimulationTaskTable table;
    first.enumerate(composition, "out", table);
    second.enumerate(composition, "out", table);
    table.seal();

    ASSERT_EQ(table.size(), 8u) << "2 plugins x 2 missions x 2 drones x 1 lidar";
    EXPECT_EQ(table.pluginCount(), 2u);

    for (std::size_t i = 0; i < 4; ++i) {
        EXPECT_EQ(table.cell(i).plugin_index, 0u);
    }
    for (std::size_t i = 4; i < 8; ++i) {
        EXPECT_EQ(table.cell(i).plugin_index, 1u);
    }

    /**
     * @note The two halves must carry *different* factories. A cell reaching the wrong plugin's
     *       factory would still produce a plausible report, attributed to the wrong plugin.
     */
    EXPECT_NE(table.cell(0).factory, table.cell(4).factory);
    EXPECT_EQ(table.cell(0).factory, table.cell(3).factory);
    EXPECT_EQ(table.cell(4).factory, table.cell(7).factory);
}

/**
 * @brief Slicing a shared table returns exactly one plugin's results, and nothing for a bad index.
 * @note An out-of-range plugin yields an empty slice rather than throwing, so report assembly can
 *       ask for a plugin that produced nothing without special-casing it.
 */
TEST(SimulationTaskTable, SlicingReturnsOnlyThatPluginsResults) {
    std::atomic<std::size_t> counter{0};
    simulator::ErrorLogger logger;

    simulator::SimulationManager first{std::make_unique<OrderStampingFactory>(counter), "PluginA",
                                       logger};
    simulator::SimulationManager second{std::make_unique<OrderStampingFactory>(counter), "PluginB",
                                        logger};

    const simulator::types::SimulationCompositionData composition = tinyComposition();
    simulator::SimulationTaskTable table;
    first.enumerate(composition, "out", table);
    second.enumerate(composition, "out", table);
    table.seal();

    simulator::InlineExecutor executor;
    executor.forEach(table.size(), [&](std::size_t index) {
        (table.cell(index).plugin_index == 0 ? first : second).runCell(table, index);
    });

    EXPECT_EQ(table.resultsForPlugin(0).size(), 4u);
    EXPECT_EQ(table.resultsForPlugin(1).size(), 4u);
    EXPECT_TRUE(table.resultsForPlugin(2).empty()) << "an out-of-range plugin yields nothing";
}

/**
 * @brief The serial executor visits every index exactly once.
 * @note The baseline the threaded executor is checked against: same contract, no threads, so a
 *       failure here is a bug in the dispatch rather than a race.
 */
TEST(InlineExecutor, VisitsEveryIndexExactlyOnce) {
    simulator::InlineExecutor executor;
    std::vector<std::size_t> visits(5, 0);
    executor.forEach(visits.size(), [&](std::size_t index) { ++visits[index]; });

    for (const std::size_t count : visits) {
        EXPECT_EQ(count, 1u);
    }
}

/**
 * @brief A zero-length table simply runs nothing.
 * @note Reachable in practice - a composition whose simulations were all rejected leaves no cells -
 *       so it has to be a normal outcome rather than an edge case.
 */
TEST(InlineExecutor, AnEmptyTableIsNotAnError) {
    simulator::InlineExecutor executor;
    std::size_t calls = 0;
    executor.forEach(0, [&](std::size_t) { ++calls; });
    EXPECT_EQ(calls, 0u);
}

/**
 * @brief Results occupy their own slot by index, regardless of the order runs complete.
 */
TEST(SimulationTaskTable, ResultsLandInIndexOrderWhateverTheCompletionOrder) {
    /**
     * @note The assertion that actually protects phase 08. Executed backwards, cell 0 finishes
     *       *last* - so if results were appended as runs completed rather than written to their own
     *       slot, every report would come out reversed.
     */
    std::atomic<std::size_t> counter{0};
    simulator::ErrorLogger logger;
    simulator::SimulationManager manager{std::make_unique<OrderStampingFactory>(counter), "Plugin",
                                         logger};

    const simulator::types::SimulationCompositionData composition = tinyComposition();
    simulator::SimulationTaskTable table;
    manager.enumerate(composition, "out", table);
    table.seal();

    ReverseExecutor executor;
    executor.forEach(table.size(), [&](std::size_t index) { manager.runCell(table, index); });

    const std::vector<simulator::types::SimulationResult> results = table.resultsForPlugin(0);
    ASSERT_EQ(results.size(), 4u);

    const std::size_t last = results.size() - 1;
    EXPECT_EQ(results.front().mission_results.front().steps, last)
        << "the first cell completed last, and still occupies slot 0";
    EXPECT_EQ(results.back().mission_results.front().steps, 0u);

    /**
     * @note Enumeration order is simulation, mission, drone, lidar - so with two drones the mission
     *       changes every second cell. The report writer walks the source paths in exactly this
     *       order to label runs, which is why the order is asserted rather than assumed.
     */
    EXPECT_EQ(results[0].mission_config.max_steps, 100u);
    EXPECT_EQ(results[1].mission_config.max_steps, 100u);
    EXPECT_EQ(results[2].mission_config.max_steps, 200u);
    EXPECT_EQ(results[3].mission_config.max_steps, 200u);
}

/**
 * @brief Running the three steps by hand yields the same report as `run()` produces in one call.
 */
TEST(SimulationTaskTable, TheSplitStepsReproduceWhatRunReturns) {
    /**
     * @note The property the orchestrator depends on: doing the three steps by hand at a wider scope
     *       must give the same report as `ISimulation::run` gives at single-plugin scope. If these
     *       ever diverge, every multi-plugin report is quietly wrong.
     */
    std::atomic<std::size_t> whole_counter{0};
    std::atomic<std::size_t> split_counter{0};
    simulator::ErrorLogger logger;

    const simulator::types::SimulationCompositionData composition = tinyComposition();

    simulator::SimulationManager whole{std::make_unique<OrderStampingFactory>(whole_counter),
                                       "Plugin", logger};
    const simulator::types::SimulationManagerReport expected = whole.run(composition, "out");

    simulator::SimulationManager split{std::make_unique<OrderStampingFactory>(split_counter),
                                       "Plugin", logger};
    simulator::SimulationTaskTable table;
    split.enumerate(composition, "out", table);
    table.seal();
    simulator::InlineExecutor executor;
    executor.forEach(table.size(), [&](std::size_t index) { split.runCell(table, index); });
    const simulator::types::SimulationManagerReport actual =
        split.assemble(composition, table.resultsForPlugin(0));

    ASSERT_EQ(actual.runs.size(), expected.runs.size());
    EXPECT_EQ(actual.metric, expected.metric);
    EXPECT_EQ(actual.error_score, expected.error_score);
    EXPECT_EQ(actual.composition_file, expected.composition_file);

    for (std::size_t i = 0; i < expected.runs.size(); ++i) {
        EXPECT_EQ(actual.runs[i].mission_config.max_steps, expected.runs[i].mission_config.max_steps)
            << "at index " << i;
        EXPECT_EQ(actual.runs[i].mission_results.front().steps,
                  expected.runs[i].mission_results.front().steps)
            << "at index " << i;
    }
}

} // namespace
