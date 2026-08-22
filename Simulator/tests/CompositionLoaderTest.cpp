/**
 * @file CompositionLoaderTest.cpp
 * @brief Coverage of composition parsing, path resolution, and the two tiers of recovery.
 * @note The happy-path cases run against the shipped `inputs/` dataset rather than a synthetic
 *       fixture, because path rebasing is only meaningful across a real directory tree. The recovery
 *       cases build small YAML trees in a temp directory, since the shipped dataset is deliberately
 *       well-formed.
 */

#include <Simulator/CompositionLoader.h>

#include <gtest/gtest.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>
#include <tuple>

namespace {

namespace fs = std::filesystem;

/**
 * @brief The repository root, injected by CMake.
 * @return Path to the directory holding `inputs/`.
 * @note Absolute, so the tests behave identically whatever working directory ctest launches them
 *       from - the same reason the loader itself rebases against the composition file.
 */
[[nodiscard]] fs::path sourceRoot() {
    return fs::path{DRONE_SOURCE_DIR};
}

/**
 * @brief The shipped composition file.
 * @return Path to `inputs/sim_compose.yaml`.
 */
[[nodiscard]] fs::path shippedComposition() {
    return sourceRoot() / "inputs" / "sim_compose.yaml";
}

/**
 * @brief Strip units from a length for comparison.
 * @param length Quantity to convert.
 * @return The value in centimetres as a plain double.
 */
[[nodiscard]] double asCm(common::PhysicalLength length) {
    return length.force_numerical_value_in(common::cm);
}

/**
 * @brief Strip units from an X coordinate.
 * @param length Quantity to convert.
 * @return The value in centimetres as a plain double.
 */
[[nodiscard]] double asCm(common::XLength length) {
    return length.force_numerical_value_in(common::cm);
}

/**
 * @brief Strip units from a Y coordinate.
 * @param length Quantity to convert.
 * @return The value in centimetres as a plain double.
 */
[[nodiscard]] double asCm(common::YLength length) {
    return length.force_numerical_value_in(common::cm);
}

/**
 * @brief Strip units from a Z coordinate.
 * @param length Quantity to convert.
 * @return The value in centimetres as a plain double.
 */
[[nodiscard]] double asCm(common::ZLength length) {
    return length.force_numerical_value_in(common::cm);
}

TEST(CompositionLoader, LoadsTheShippedDataset) {
    simulator::ErrorLogger logger;
    const simulator::CompositionLoadResult result =
        simulator::loadComposition(shippedComposition(), logger);

    ASSERT_TRUE(result.ok()) << result.error;

    const simulator::types::SimulationCompositionData& composition = result.composition;
    EXPECT_EQ(composition.simulation_mission_groups.size(), 5u);
    EXPECT_EQ(composition.drone_configs.size(), 2u);
    EXPECT_EQ(composition.lidar_configs.size(), 2u);

    std::size_t pairs = 0;
    for (const auto& group : composition.simulation_mission_groups) {
        pairs += std::get<1>(group).size();
    }
    EXPECT_EQ(pairs, 6u) << "the house simulation carries two missions, the rest one each";

    EXPECT_EQ(logger.errorCount(), 0u) << "the shipped dataset must parse without any recovery";
}

TEST(CompositionLoader, ResolvesEveryMapFileToSomethingThatExists) {
    simulator::ErrorLogger logger;
    const simulator::CompositionLoadResult result =
        simulator::loadComposition(shippedComposition(), logger);
    ASSERT_TRUE(result.ok()) << result.error;

    for (const auto& group : result.composition.simulation_mission_groups) {
        const fs::path& map = std::get<0>(group).map_filename;
        EXPECT_TRUE(map.is_absolute()) << map.string();
        EXPECT_TRUE(fs::exists(map)) << map.string();
        EXPECT_EQ(map.parent_path(), sourceRoot() / "inputs" / "map")
            << "map_filename rebases against the composition, not the file that named it";
    }
}

TEST(CompositionLoader, ParsesSimulationValues) {
    simulator::ErrorLogger logger;
    const simulator::CompositionLoadResult result =
        simulator::loadComposition(shippedComposition(), logger);
    ASSERT_TRUE(result.ok()) << result.error;

    const simulator::types::SimulationConfigData& house =
        std::get<0>(result.composition.simulation_mission_groups.front());

    EXPECT_EQ(house.map_filename.filename(), "scenario_house.npy");
    EXPECT_DOUBLE_EQ(asCm(house.map_resolution), 10.0);
    EXPECT_DOUBLE_EQ(asCm(house.initial_drone_position.x), 150.0);
    EXPECT_DOUBLE_EQ(asCm(house.initial_drone_position.y), 200.0);
    EXPECT_DOUBLE_EQ(asCm(house.initial_drone_position.z), 10.0);
    EXPECT_DOUBLE_EQ(asCm(house.map_offset.z), 150.0);
    EXPECT_DOUBLE_EQ(house.initial_angle.force_numerical_value_in(common::deg), 0.0);
}

TEST(CompositionLoader, HalvesTheDroneDiameterIntoARadius) {
    simulator::ErrorLogger logger;
    const simulator::CompositionLoadResult result =
        simulator::loadComposition(shippedComposition(), logger);
    ASSERT_TRUE(result.ok()) << result.error;

    const common::types::DroneConfigData& small = result.composition.drone_configs.front();
    EXPECT_DOUBLE_EQ(asCm(small.radius), 4.0) << "drone_small.yaml gives dimensions_cm: 8";
    EXPECT_DOUBLE_EQ(small.max_rotate.force_numerical_value_in(common::deg), 90.0);
    EXPECT_DOUBLE_EQ(asCm(small.max_advance), 30.0);
    EXPECT_DOUBLE_EQ(asCm(small.max_elevate), 20.0);
}

TEST(CompositionLoader, ParsesLidarAndMissionValues) {
    simulator::ErrorLogger logger;
    const simulator::CompositionLoadResult result =
        simulator::loadComposition(shippedComposition(), logger);
    ASSERT_TRUE(result.ok()) << result.error;

    const common::types::LidarConfigData& lidar = result.composition.lidar_configs.front();
    EXPECT_DOUBLE_EQ(asCm(lidar.z_min), 20.0);
    EXPECT_DOUBLE_EQ(asCm(lidar.z_max), 150.0);
    EXPECT_DOUBLE_EQ(asCm(lidar.d), 2.5);
    EXPECT_EQ(lidar.fov_circles, 3u);

    const common::types::MissionConfigData& mission =
        std::get<1>(result.composition.simulation_mission_groups.front()).front();
    EXPECT_EQ(mission.max_steps, 2000u);
    EXPECT_DOUBLE_EQ(asCm(mission.gps_resolution), 5.0);
    EXPECT_DOUBLE_EQ(mission.output_mapping_resolution_factor, 1.0) << "absent means 1";
    EXPECT_DOUBLE_EQ(asCm(mission.mission_bounds.max_x), 290.0);
    EXPECT_DOUBLE_EQ(asCm(mission.mission_bounds.max_height), 60.0);
}

TEST(CompositionLoader, RecordsSourcePathsInParallel) {
    simulator::ErrorLogger logger;
    simulator::CompositionPaths paths;
    const simulator::CompositionLoadResult result =
        simulator::loadComposition(shippedComposition(), logger, &paths);
    ASSERT_TRUE(result.ok()) << result.error;

    ASSERT_EQ(paths.simulation_paths.size(), result.composition.simulation_mission_groups.size());
    ASSERT_EQ(paths.mission_paths.size(), result.composition.simulation_mission_groups.size());
    EXPECT_EQ(paths.drone_paths.size(), result.composition.drone_configs.size());
    EXPECT_EQ(paths.lidar_paths.size(), result.composition.lidar_configs.size());

    EXPECT_EQ(paths.simulation_paths.front(), "simulation/house_simulation.yaml")
        << "paths are recorded as written, not as resolved - the report labels runs by them";
    EXPECT_EQ(paths.mission_paths.front().size(), 2u);
    EXPECT_EQ(paths.drone_paths.front(), "drone/drone_small.yaml");
}

/**
 * @brief Gives each recovery test its own scratch tree to write malformed YAML into.
 */
class CompositionLoaderRecoveryTest : public ::testing::Test {
protected:
    /**
     * @brief Create a uniquely named scratch directory under the system temp folder.
     */
    void SetUp() override {
        const ::testing::TestInfo* const info =
            ::testing::UnitTest::GetInstance()->current_test_info();
        dir_ = fs::temp_directory_path() /
               ("ex3_composition_" + std::string{info->name()} + "_" + std::to_string(::getpid()));
        std::error_code ec;
        fs::remove_all(dir_, ec);
        fs::create_directories(dir_, ec);
    }

    /**
     * @brief Remove the scratch directory and everything in it.
     */
    void TearDown() override {
        std::error_code ec;
        fs::remove_all(dir_, ec);
    }

    /**
     * @brief Write a file into the scratch tree, creating parent directories.
     * @param relative Path relative to the scratch directory.
     * @param contents File body.
     * @return The absolute path written.
     */
    fs::path write(const fs::path& relative, const std::string& contents) const {
        const fs::path target = dir_ / relative;
        std::error_code ec;
        fs::create_directories(target.parent_path(), ec);
        std::ofstream stream(target);
        stream << contents;
        return target;
    }

    /**
     * @brief Write a minimal composition naming one of each config kind.
     * @param mission_entry The `mission_configs` sequence body, so a test can empty it.
     * @return The composition file's path.
     */
    fs::path writeComposition(const std::string& mission_entry = "        - \"mission/m.yaml\"\n") const {
        return write("compose.yaml",
                     "simulation_compositions:\n"
                     "  simulations:\n"
                     "    - simulation_config: \"simulation/s.yaml\"\n"
                     "      mission_configs:\n" +
                         mission_entry +
                         "  drone_configs:\n"
                         "    - \"drone/d.yaml\"\n"
                         "  lidar_configs:\n"
                         "    - \"lidar/l.yaml\"\n");
    }

    fs::path dir_{};
};

TEST_F(CompositionLoaderRecoveryTest, MissingKeyDefaultsAndIsLogged) {
    const fs::path composition = writeComposition();
    write("simulation/s.yaml", "simulation_config:\n  map_filename: \"map/m.npy\"\n");
    write("mission/m.yaml", "mission_config:\n  max_steps: 5\n");
    write("drone/d.yaml", "drone_config:\n  dimensions_cm: 8\n");
    write("lidar/l.yaml", "lidar_config:\n  z_min_cm: 1\n");

    simulator::ErrorLogger logger;
    const simulator::CompositionLoadResult result = simulator::loadComposition(composition, logger);

    ASSERT_TRUE(result.ok()) << result.error;
    EXPECT_EQ(result.composition.simulation_mission_groups.size(), 1u)
        << "missing fields must not cost us the entry";
    EXPECT_GT(logger.inputErrorCount(), 0u) << "every recovery is reported as an input error";
    EXPECT_DOUBLE_EQ(asCm(result.composition.drone_configs.front().radius), 4.0);
}

TEST_F(CompositionLoaderRecoveryTest, BadValueDefaultsAndIsLogged) {
    const fs::path composition = writeComposition();
    write("simulation/s.yaml", "simulation_config:\n  map_resolution_cm: \"not a number\"\n");
    write("mission/m.yaml", "mission_config:\n  max_steps: 5\n");
    write("drone/d.yaml", "drone_config:\n  dimensions_cm: 8\n");
    write("lidar/l.yaml", "lidar_config:\n  z_min_cm: 1\n");

    simulator::ErrorLogger logger;
    const simulator::CompositionLoadResult result = simulator::loadComposition(composition, logger);

    ASSERT_TRUE(result.ok()) << result.error;
    EXPECT_DOUBLE_EQ(asCm(std::get<0>(result.composition.simulation_mission_groups.front())
                              .map_resolution),
                     0.0);
    EXPECT_GT(logger.inputErrorCount(), 0u);
}

TEST_F(CompositionLoaderRecoveryTest, UnreadableReferenceDegradesToDefaults) {
    const fs::path composition = writeComposition();
    write("simulation/s.yaml", "simulation_config:\n  map_filename: \"map/m.npy\"\n");
    write("mission/m.yaml", "mission_config:\n  max_steps: 5\n");
    write("lidar/l.yaml", "lidar_config:\n  z_min_cm: 1\n");
    /** @note `drone/d.yaml` is deliberately never written. */

    simulator::ErrorLogger logger;
    const simulator::CompositionLoadResult result = simulator::loadComposition(composition, logger);

    ASSERT_TRUE(result.ok()) << result.error;
    ASSERT_EQ(result.composition.drone_configs.size(), 1u)
        << "the entry still exists, filled with defaults";
    EXPECT_DOUBLE_EQ(asCm(result.composition.drone_configs.front().radius), 0.0);
    EXPECT_GT(logger.inputErrorCount(), 0u);
}

TEST_F(CompositionLoaderRecoveryTest, SimulationWithoutMissionsIsSkippedAndPathsStayAligned) {
    const fs::path composition = writeComposition("        []\n");
    write("simulation/s.yaml", "simulation_config:\n  map_filename: \"map/m.npy\"\n");
    write("drone/d.yaml", "drone_config:\n  dimensions_cm: 8\n");
    write("lidar/l.yaml", "lidar_config:\n  z_min_cm: 1\n");

    simulator::ErrorLogger logger;
    simulator::CompositionPaths paths;
    const simulator::CompositionLoadResult result =
        simulator::loadComposition(composition, logger, &paths);

    ASSERT_TRUE(result.ok()) << result.error;
    EXPECT_TRUE(result.composition.simulation_mission_groups.empty());
    EXPECT_TRUE(paths.simulation_paths.empty())
        << "a skipped group must not record paths, or every later index misaligns";
    EXPECT_TRUE(paths.mission_paths.empty());
    EXPECT_GT(logger.inputErrorCount(), 0u);
}

TEST_F(CompositionLoaderRecoveryTest, MalformedDocumentFailsWithoutThrowing) {
    const fs::path composition = write("broken.yaml", "simulation_compositions: [unclosed\n");

    simulator::ErrorLogger logger;
    simulator::CompositionLoadResult result;
    EXPECT_NO_THROW({ result = simulator::loadComposition(composition, logger); });

    EXPECT_FALSE(result.ok());
    EXPECT_FALSE(result.error.empty());
}

TEST_F(CompositionLoaderRecoveryTest, MissingFileFailsWithoutThrowing) {
    simulator::ErrorLogger logger;
    simulator::CompositionLoadResult result;
    EXPECT_NO_THROW({ result = simulator::loadComposition(dir_ / "absent.yaml", logger); });

    EXPECT_FALSE(result.ok());
    EXPECT_FALSE(result.error.empty());
}

} // namespace
