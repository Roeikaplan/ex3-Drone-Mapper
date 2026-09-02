/**
 * @file SimulationPipelineTest.cpp
 * @brief Coverage of config identity, run wiring, scoring, and configuration expansion.
 * @note Not one `.so` is loaded here. The factory takes `std::function`s, so a test supplies lambdas
 *       returning hand-written fakes - which is a direct payoff of binding the plugin pair through
 *       the constructor rather than through a global.
 */

#include <Simulator/CompositionLoader.h>
#include <Simulator/ConfigIdentityIndex.h>
#include <Simulator/SimulationManager.h>
#include <Simulator/SimulationRunFactoryImpl.h>

#include <gtest/gtest.h>

#include <cstdlib>
#include <filesystem>
#include <memory>
#include <string>

namespace {

namespace fs = std::filesystem;

using common::cm;

/**
 * @brief The repository root, injected by CMake.
 * @return Path to the directory holding `inputs/`.
 */
[[nodiscard]] fs::path sourceRoot() {
    return fs::path{DRONE_SOURCE_DIR};
}

/**
 * @brief An algorithm that reports completion without planning anything.
 */
class FakeAlgorithm final : public common::IMappingAlgorithm {
public:
    using common::IMappingAlgorithm::IMappingAlgorithm;

    /**
     * @brief Decide the next command.
     * @return A hover, with status `Finished`.
     */
    [[nodiscard]] common::types::MappingStepCommand nextStep(
        const common::types::DroneState&, const common::types::LidarScanResult*) override {
        common::types::MappingStepCommand command{};
        command.status = common::types::AlgorithmStatus::Finished;
        return command;
    }
};

/**
 * @brief A mission control that reports a fixed outcome and optionally saves the map.
 */
class FakeMissionControl final : public common::IMissionControl {
public:
    /**
     * @brief Construct from the host-supplied dependencies.
     * @param dependencies What the run factory wired up.
     * @param status Outcome to report.
     * @param occupy Whether to mark one voxel occupied before finishing.
     */
    FakeMissionControl(common::MissionControlDependencies dependencies,
                       common::types::MissionRunStatus status, bool occupy)
        : output_map_(dependencies.output_map), status_(status), occupy_(occupy) {}

    /**
     * @brief Run the mission.
     * @return The configured status, with one step.
     */
    [[nodiscard]] common::types::MissionRunResult runMission() override {
        if (occupy_) {
            output_map_.set(common::Position3D{}, common::types::VoxelOccupancy::Occupied);
        }
        return common::types::MissionRunResult{status_, 1, {}};
    }

private:
    common::IMutableMap3D& output_map_;
    common::types::MissionRunStatus status_;
    bool occupy_;
};

/**
 * @brief Load the shipped composition.
 * @param logger Sink for recoverable defects.
 * @param paths Receives the source paths.
 * @return The parsed composition.
 */
[[nodiscard]] simulator::CompositionLoadResult shippedComposition(simulator::ErrorLogger& logger,
                                                                  simulator::CompositionPaths& paths) {
    return simulator::loadComposition(sourceRoot() / "inputs" / "sim_compose.yaml", logger, &paths);
}

/**
 * @brief Every config in the shipped composition resolves to its source file's stem.
 * @note The index resolves by *address*, which is what lets the run factory name a config it holds
 *       only a reference to - and why the stems, not the full paths, end up in output filenames.
 */
TEST(ConfigIdentityIndex, ResolvesEachConfigToItsSourceStem) {
    simulator::ErrorLogger logger;
    simulator::CompositionPaths paths;
    const simulator::CompositionLoadResult composition = shippedComposition(logger, paths);
    ASSERT_TRUE(composition.ok()) << composition.error;

    const simulator::ConfigIdentityIndex identity{composition.composition, paths};

    const auto& group = composition.composition.simulation_mission_groups.front();
    EXPECT_EQ(identity.nameOf(std::get<0>(group)), "house_simulation");
    EXPECT_EQ(identity.nameOf(std::get<1>(group).front()), "house_mission_lower");
    EXPECT_EQ(identity.nameOf(composition.composition.drone_configs.front()), "drone_small");
    EXPECT_EQ(identity.nameOf(composition.composition.lidar_configs.front()), "lidar_long");
}

/**
 * @brief A config the index never saw resolves to a conspicuous fallback rather than a wrong name.
 * @note Because lookup is by address, a *copy* of an indexed config is a stranger too - which is
 *       precisely why the report writer cannot use this index and labels runs positionally instead.
 */
TEST(ConfigIdentityIndex, AnUnindexedConfigFallsBackConspicuously) {
    simulator::ErrorLogger logger;
    simulator::CompositionPaths paths;
    const simulator::CompositionLoadResult composition = shippedComposition(logger, paths);
    ASSERT_TRUE(composition.ok()) << composition.error;

    const simulator::ConfigIdentityIndex identity{composition.composition, paths};

    /**
     * @note A config that is not part of the indexed composition - which is also what a copied
     *       composition would look like from the index's point of view.
     */
    const common::types::DroneConfigData stranger{};
    EXPECT_EQ(identity.nameOf(stranger), "unknown");
}

/**
 * @brief With no paths recorded, configs still get distinct positional names.
 * @note `loadComposition` may be called without a paths sink, and output map filenames must still
 *       differ between runs - colliding names would have runs overwrite each other's maps.
 */
TEST(ConfigIdentityIndex, WithoutRecordedPathsNamesStayDistinguishable) {
    simulator::ErrorLogger logger;
    const simulator::CompositionLoadResult composition =
        simulator::loadComposition(sourceRoot() / "inputs" / "sim_compose.yaml", logger, nullptr);
    ASSERT_TRUE(composition.ok()) << composition.error;

    const simulator::ConfigIdentityIndex identity{composition.composition,
                                                  simulator::CompositionPaths{}};

    EXPECT_EQ(identity.nameOf(composition.composition.drone_configs.front()), "drone0");
    EXPECT_EQ(identity.nameOf(composition.composition.drone_configs.back()), "drone1");
}

/**
 * @brief Gives each pipeline test a scratch results directory and the shipped composition.
 */
class SimulationPipelineTest : public ::testing::Test {
protected:
    /**
     * @brief Create the scratch directory and load the composition.
     */
    void SetUp() override {
        const ::testing::TestInfo* const info =
            ::testing::UnitTest::GetInstance()->current_test_info();
        dir_ = fs::temp_directory_path() /
               ("ex3_pipeline_" + std::string{info->name()} + "_" + std::to_string(::getpid()));
        std::error_code ec;
        fs::remove_all(dir_, ec);
        fs::create_directories(dir_, ec);

        composition_ = shippedComposition(logger_, paths_);
        ASSERT_TRUE(composition_.ok()) << composition_.error;
        identity_ = std::make_unique<simulator::ConfigIdentityIndex>(composition_.composition,
                                                                     paths_);
    }

    /**
     * @brief Remove the scratch directory.
     */
    void TearDown() override {
        std::error_code ec;
        fs::remove_all(dir_, ec);
    }

    /**
     * @brief Build a factory bound to fakes.
     * @param status Outcome the fake mission control reports.
     * @param occupy Whether the fake marks a voxel occupied.
     * @return The bound factory.
     */
    [[nodiscard]] std::unique_ptr<simulator::SimulationRunFactoryImpl> makeFactory(
        common::types::MissionRunStatus status, bool occupy) {
        return std::make_unique<simulator::SimulationRunFactoryImpl>(
            [status, occupy](common::MissionControlDependencies dependencies)
                -> std::unique_ptr<common::IMissionControl> {
                return std::make_unique<FakeMissionControl>(std::move(dependencies), status, occupy);
            },
            [](common::MappingAlgorithmDependencies dependencies)
                -> std::unique_ptr<common::IMappingAlgorithm> {
                return std::make_unique<FakeAlgorithm>(std::move(dependencies));
            },
            "FakePlugin", *identity_, false);
    }

    /**
     * @brief The first simulation, mission, drone, and lidar of the shipped composition.
     * @return References suitable for a single `create()` call.
     */
    [[nodiscard]] const simulator::types::SimulationConfigData& firstSimulation() const {
        return std::get<0>(composition_.composition.simulation_mission_groups.front());
    }

    simulator::ErrorLogger logger_{};
    simulator::CompositionPaths paths_{};
    simulator::CompositionLoadResult composition_{};
    std::unique_ptr<simulator::ConfigIdentityIndex> identity_{};
    fs::path dir_{};
};

/**
 * @brief The output map filename names the plugin and all four configs of the run.
 * @note One results directory holds every plugin's every run, so the filename is what keeps them
 *       apart - and what the flat-report fallback later relies on to identify a run at all.
 */
TEST_F(SimulationPipelineTest, OutputMapFilenameNamesEveryDimensionOfTheRun) {
    const auto factory = makeFactory(common::types::MissionRunStatus::Completed, false);
    const auto& group = composition_.composition.simulation_mission_groups.front();

    const std::unique_ptr<simulator::ISimulationRun> run =
        factory->create(std::get<0>(group), std::get<1>(group).front(),
                        composition_.composition.drone_configs.front(),
                        composition_.composition.lidar_configs.front(), dir_);
    ASSERT_NE(run, nullptr);

    const simulator::types::SimulationResult result = run->run();
    EXPECT_EQ(result.output_map_file.filename().string(),
              "FakePlugin__house_simulation__house_mission_lower__drone_small__lidar_long.npy");
}

/**
 * @brief The hidden map is built with real boundaries, so scoring compares a genuine grid.
 */
TEST_F(SimulationPipelineTest, TheHiddenMapGetsRealBoundsSoScoringIsMeaningful) {
    /**
     * @note If the hidden map were built with a default `MapConfig`, scoring would walk an empty
     *       grid, find no occupied voxels in either map, and hand back 100 for a run that mapped
     *       nothing. Marking one voxel occupied in the *output* map must therefore score below 100.
     */
    const auto factory = makeFactory(common::types::MissionRunStatus::Completed, true);
    const auto& group = composition_.composition.simulation_mission_groups.front();

    const std::unique_ptr<simulator::ISimulationRun> run =
        factory->create(std::get<0>(group), std::get<1>(group).front(),
                        composition_.composition.drone_configs.front(),
                        composition_.composition.lidar_configs.front(), dir_);
    const simulator::types::SimulationResult result = run->run();

    EXPECT_GE(result.mission_score, 0.0);
    EXPECT_LT(result.mission_score, 100.0)
        << "a real ground-truth grid must disagree with a map holding one stray voxel";
}

/**
 * @brief An errored mission takes the -1 sentinel and is not compared at all.
 * @note A partial map from a failed run would still yield a plausible-looking number, so failure has
 *       to be distinguishable from a poor score rather than folded into one.
 */
TEST_F(SimulationPipelineTest, AnErroredMissionScoresTheSentinel) {
    const auto factory = makeFactory(common::types::MissionRunStatus::Error, true);
    const auto& group = composition_.composition.simulation_mission_groups.front();

    const std::unique_ptr<simulator::ISimulationRun> run =
        factory->create(std::get<0>(group), std::get<1>(group).front(),
                        composition_.composition.drone_configs.front(),
                        composition_.composition.lidar_configs.front(), dir_);
    const simulator::types::SimulationResult result = run->run();

    EXPECT_DOUBLE_EQ(result.mission_score, -1.0)
        << "a failed run must be distinguishable from a poor score";
    ASSERT_EQ(result.mission_results.size(), 1u);
    EXPECT_EQ(result.mission_results.front().status, common::types::MissionRunStatus::Error);
}

/**
 * @brief A missing ground-truth map throws from `create` rather than producing a scoreable run.
 * @note This is the throw the manager catches to log `RUN_FAILED`. Building an empty hidden map
 *       instead would score every affected run against nothing and hand back a perfect result.
 */
TEST_F(SimulationPipelineTest, AMissingMapFileThrowsRatherThanScoringSilently) {
    simulator::types::SimulationConfigData broken = firstSimulation();
    broken.map_filename = dir_ / "absent.npy";

    const auto factory = makeFactory(common::types::MissionRunStatus::Completed, false);
    const auto& group = composition_.composition.simulation_mission_groups.front();

    EXPECT_THROW((void)factory->create(broken, std::get<1>(group).front(),
                                       composition_.composition.drone_configs.front(),
                                       composition_.composition.lidar_configs.front(), dir_),
                 std::runtime_error);
}

/**
 * @brief The manager expands the full cartesian product and stamps the report's metadata.
 */
TEST_F(SimulationPipelineTest, TheManagerExpandsEveryCombination) {
    simulator::SimulationManager manager{makeFactory(common::types::MissionRunStatus::Completed,
                                                     false),
                                         "FakePlugin", logger_};

    const simulator::types::SimulationManagerReport report =
        manager.run(composition_.composition, dir_);

    EXPECT_EQ(report.runs.size(), 24u) << "6 simulation/mission pairs x 2 drones x 2 lidars";
    EXPECT_EQ(report.metric, "occupied_voxel_iou");
    EXPECT_EQ(report.error_score, -1);
    EXPECT_FALSE(report.generated_at_utc.empty());
    EXPECT_EQ(report.composition_file, composition_.composition.composition_file);
}

/**
 * @brief A factory that throws for every cell still yields a complete report, one error per cell.
 */
TEST_F(SimulationPipelineTest, AFailingFactoryScoresTheCellAndKeepsGoing) {
    /**
     * @note A factory that always throws stands in for a bad map file, which fails identically for
     *       every combination of the affected simulation. Every cell must still appear in the
     *       report, or the run count would silently disagree with what the composition asked for.
     */
    class ThrowingFactory final : public simulator::ISimulationRunFactory {
    public:
        [[nodiscard]] std::unique_ptr<simulator::ISimulationRun> create(
            const simulator::types::SimulationConfigData&,
            const common::types::MissionConfigData&, const common::types::DroneConfigData&,
            const common::types::LidarConfigData&, const std::filesystem::path&) override {
            throw std::runtime_error("deliberate failure");
        }
    };

    simulator::SimulationManager manager{std::make_unique<ThrowingFactory>(), "FakePlugin", logger_};
    const simulator::types::SimulationManagerReport report =
        manager.run(composition_.composition, dir_);

    ASSERT_EQ(report.runs.size(), 24u);
    for (const simulator::types::SimulationResult& result : report.runs) {
        EXPECT_DOUBLE_EQ(result.mission_score, -1.0);
    }
    EXPECT_EQ(logger_.errorCount(), 24u) << "every failed cell is logged as it happens";
}

/**
 * @brief A manager constructed with no factory is rejected immediately.
 * @note Failing in the constructor rather than at first use keeps the null out of the run loop,
 *       where it would surface once per cell as an unexplained crash inside a worker thread.
 */
TEST_F(SimulationPipelineTest, ANullFactoryIsRejected) {
    EXPECT_THROW(simulator::SimulationManager(nullptr, "FakePlugin", logger_),
                 std::invalid_argument);
}

} // namespace
