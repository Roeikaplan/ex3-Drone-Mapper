#include <drone_mapper/CompositionLoader.h>
#include <drone_mapper/ErrorLogger.h>

#include <gtest/gtest.h>

#include <cstddef>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <tuple>

namespace {

using namespace drone_mapper;

// A composition YAML exercising the tricky mappings: diameter->radius, boundaries->mission_bounds,
// omitted output_mapping_resolution_factor (should default to 1), and nested simulation groups.
constexpr const char* kCompositionYaml = R"(
simulation_compositions:
  simulations:
    - simulation_config:
        map_filename: data_maps/single_voxel_x2_y4_z2.npy
        map_resolution_cm: 10.0
        initial_drone_position: {x_cm: 5, y_cm: 6, height_cm: 7}
        initial_angle_deg: 90
        map_axes_offset: {x_offset: 1, y_offset: 2, height_offset: 3}
      mission_configs:
        - max_steps: 25
          gps_resolution_cm: 4
          boundaries:
            x_boundary: {min_cm: 0, max_cm: 100}
            y_boundary: {min_cm: 0, max_cm: 200}
            height_boundary: {min_cm: 0, max_cm: 50}
  drone_configs:
    - dimensions_cm: 30
      max_rotate_deg: 45
      max_advance_cm: 50
      max_elevate_cm: 40
  lidar_configs:
    - z_min_cm: 20
      z_max_cm: 120
      d_cm: 2.5
      fov_circles: 5
)";

/**
 * @brief Write YAML text to a unique temp file and return its path.
 * @param name File stem.
 * @param content YAML body.
 * @return Path to the written file.
 */
[[nodiscard]] std::filesystem::path writeTempYaml(const std::string& name, const char* content) {
    const std::filesystem::path path = std::filesystem::path(::testing::TempDir()) / (name + ".yaml");
    std::ofstream out(path, std::ios::trunc);
    out << content;
    return path;
}

/**
 * @brief A fresh temp path (removed if it already exists) for a per-test output file.
 * @param name File name under the temp dir.
 * @return The (now non-existent) path.
 */
[[nodiscard]] std::filesystem::path freshTempPath(const std::string& name) {
    const std::filesystem::path path = std::filesystem::path(::testing::TempDir()) / name;
    std::filesystem::remove(path);
    return path;
}

/**
 * @brief Read a whole text file into a string.
 * @param path File to read.
 * @return Its contents (empty if it does not exist).
 */
[[nodiscard]] std::string readAll(const std::filesystem::path& path) {
    std::ifstream in(path);
    std::ostringstream os;
    os << in.rdbuf();
    return os.str();
}

// A composition that parses but omits a recoverable scalar (gps_resolution_cm) so the loader records
// a recovered input-file error.
constexpr const char* kMissingKeyYaml = R"(
simulation_compositions:
  simulations:
    - simulation_config:
        map_filename: data_maps/single_voxel_x2_y4_z2.npy
        map_resolution_cm: 10.0
        initial_drone_position: {x_cm: 5, y_cm: 6, height_cm: 7}
        initial_angle_deg: 0
        map_axes_offset: {x_offset: 0, y_offset: 0, height_offset: 0}
      mission_configs:
        - max_steps: 25
          boundaries:
            x_boundary: {min_cm: 0, max_cm: 100}
            y_boundary: {min_cm: 0, max_cm: 200}
            height_boundary: {min_cm: 0, max_cm: 50}
  drone_configs:
    - dimensions_cm: 30
      max_rotate_deg: 45
      max_advance_cm: 50
      max_elevate_cm: 40
  lidar_configs:
    - z_min_cm: 20
      z_max_cm: 120
      d_cm: 2.5
      fov_circles: 5
)";

/**
 * @brief A fresh base directory for a file-reference composition fixture.
 * @param name Directory name distinguishing the test.
 * @return The (re)created empty directory under the gtest temp dir.
 */
[[nodiscard]] std::filesystem::path freshRefBase(const std::string& name) {
    const std::filesystem::path base = std::filesystem::path(::testing::TempDir()) / name;
    std::filesystem::remove_all(base);
    std::filesystem::create_directories(base);
    return base;
}

/**
 * @brief Write YAML text to `base/rel`, creating intermediate directories.
 * @param base Composition base directory.
 * @param rel Relative path of the file inside the base.
 * @param content YAML body.
 * @return The full path written.
 */
[[nodiscard]] std::filesystem::path writeRefYaml(const std::filesystem::path& base,
                                                 const std::string& rel, const std::string& content) {
    const std::filesystem::path path = base / rel;
    std::filesystem::create_directories(path.parent_path());
    std::ofstream out(path, std::ios::trunc);
    out << content;
    return path;
}

/**
 * @brief Populate a complete file-reference composition (the mandated layout) under @p base.
 * @param base Directory to populate with sim/mission/drone/lidar files + compose.yaml.
 * @return Path to the written compose.yaml.
 * @note Mirrors the `inputs/` dataset structure: every entry is a relative path string and every
 *       referenced file wraps its body under its own `*_config:` key. `map_filename` is relative
 *       ("map/tiny.npy") so rebasing against the composition dir is observable.
 */
[[nodiscard]] std::filesystem::path writeReferenceComposition(const std::filesystem::path& base) {
    (void)writeRefYaml(base, "simulation/sim.yaml",
                       "simulation_config:\n"
                       "  map_filename: \"map/tiny.npy\"\n"
                       "  map_resolution_cm: 10\n"
                       "  initial_drone_position: {x_cm: 15, y_cm: 25, height_cm: 35}\n"
                       "  initial_angle_deg: 180\n"
                       "  map_axes_offset: {x_offset: 1, y_offset: 2, height_offset: 3}\n");
    (void)writeRefYaml(base, "mission/mis.yaml",
                       "mission_config:\n"
                       "  max_steps: 77\n"
                       "  gps_resolution_cm: 5\n"
                       "  boundaries:\n"
                       "    x_boundary: {min_cm: 0, max_cm: 110}\n"
                       "    y_boundary: {min_cm: 0, max_cm: 120}\n"
                       "    height_boundary: {min_cm: 0, max_cm: 130}\n");
    (void)writeRefYaml(base, "drone/drn.yaml",
                       "drone_config:\n"
                       "  dimensions_cm: 8\n"
                       "  max_rotate_deg: 90\n"
                       "  max_advance_cm: 30\n"
                       "  max_elevate_cm: 20\n");
    (void)writeRefYaml(base, "lidar/lid.yaml",
                       "lidar_config:\n"
                       "  z_min_cm: 20\n"
                       "  z_max_cm: 150\n"
                       "  d_cm: 2.5\n"
                       "  fov_circles: 3\n");
    return writeRefYaml(base, "compose.yaml",
                        "simulation_compositions:\n"
                        "  simulations:\n"
                        "    - simulation_config: \"simulation/sim.yaml\"\n"
                        "      mission_configs: [\"mission/mis.yaml\"]\n"
                        "  drone_configs: [\"drone/drn.yaml\"]\n"
                        "  lidar_configs: [\"lidar/lid.yaml\"]\n");
}

} // namespace

/**
 * @brief The loader parses every config section into the composition with correct unit conversions.
 *
 * Verifies the subtle mappings: sphere diameter -> radius (30 -> 15 cm), boundaries -> mission_bounds,
 * and an omitted output_mapping_resolution_factor defaulting to 1.
 */
TEST(CompositionLoader, ParsesAllSectionsWithConversions) {
    const std::filesystem::path path = writeTempYaml("composition_ok", kCompositionYaml);
    ErrorLogger logger; // stderr only

    const types::SimulationCompositionData composition = loadComposition(path, logger);

    EXPECT_EQ(composition.composition_file, path);

    ASSERT_EQ(composition.simulation_mission_groups.size(), 1u);
    const auto& [sim, missions] = composition.simulation_mission_groups.front();
    EXPECT_EQ(sim.map_filename, std::filesystem::path{"data_maps/single_voxel_x2_y4_z2.npy"});
    EXPECT_DOUBLE_EQ(sim.map_resolution.force_numerical_value_in(cm), 10.0);
    EXPECT_DOUBLE_EQ(sim.initial_drone_position.x.force_numerical_value_in(cm), 5.0);
    EXPECT_DOUBLE_EQ(sim.initial_drone_position.z.force_numerical_value_in(cm), 7.0);
    EXPECT_DOUBLE_EQ(sim.initial_angle.force_numerical_value_in(deg), 90.0);
    EXPECT_DOUBLE_EQ(sim.map_offset.x.force_numerical_value_in(cm), 1.0);
    EXPECT_DOUBLE_EQ(sim.map_offset.z.force_numerical_value_in(cm), 3.0);

    ASSERT_EQ(missions.size(), 1u);
    EXPECT_EQ(missions[0].max_steps, 25u);
    EXPECT_DOUBLE_EQ(missions[0].gps_resolution.force_numerical_value_in(cm), 4.0);
    EXPECT_DOUBLE_EQ(missions[0].output_mapping_resolution_factor, 1.0); // omitted -> default 1
    EXPECT_DOUBLE_EQ(missions[0].mission_bounds.max_x.force_numerical_value_in(cm), 100.0);
    EXPECT_DOUBLE_EQ(missions[0].mission_bounds.max_y.force_numerical_value_in(cm), 200.0);
    EXPECT_DOUBLE_EQ(missions[0].mission_bounds.max_height.force_numerical_value_in(cm), 50.0);

    ASSERT_EQ(composition.drones.size(), 1u);
    EXPECT_DOUBLE_EQ(composition.drones[0].radius.force_numerical_value_in(cm), 15.0); // 30/2
    EXPECT_DOUBLE_EQ(composition.drones[0].max_rotate.force_numerical_value_in(deg), 45.0);

    ASSERT_EQ(composition.lidars.size(), 1u);
    EXPECT_DOUBLE_EQ(composition.lidars[0].z_min.force_numerical_value_in(cm), 20.0);
    EXPECT_EQ(composition.lidars[0].fov_circles, 5u);
}

/**
 * @brief A recovered input-file error is written to input_errors.txt (created lazily) and errors.log.
 *
 * Omitting `gps_resolution_cm` is recovered with a default; the loader must record it in the
 * lazily-created `input_errors.txt` AND in the general error log.
 */
TEST(CompositionLoader, RecoveredInputErrorWritesInputErrorsFile) {
    const std::filesystem::path yaml = writeTempYaml("composition_missing_key", kMissingKeyYaml);
    const std::filesystem::path errors_log = freshTempPath("cl_errors.log");
    const std::filesystem::path input_errors = freshTempPath("cl_input_errors.txt");
    ErrorLogger logger{errors_log, input_errors};

    (void)loadComposition(yaml, logger);

    ASSERT_TRUE(std::filesystem::exists(input_errors)); // created because there was an input error
    const std::string input_text = readAll(input_errors);
    EXPECT_NE(input_text.find("CONFIG_MISSING_KEY"), std::string::npos);
    EXPECT_NE(input_text.find("gps_resolution_cm"), std::string::npos);
    // Every input error is also a general error: it appears in the error log too.
    EXPECT_NE(readAll(errors_log).find("CONFIG_MISSING_KEY"), std::string::npos);
}

/**
 * @brief A clean composition produces no input_errors.txt at all (created only if there are errors).
 */
TEST(CompositionLoader, CleanCompositionCreatesNoInputErrorsFile) {
    const std::filesystem::path yaml = writeTempYaml("composition_clean", kCompositionYaml);
    const std::filesystem::path errors_log = freshTempPath("cl_errors_clean.log");
    const std::filesystem::path input_errors = freshTempPath("cl_input_errors_clean.txt");
    ErrorLogger logger{errors_log, input_errors};

    (void)loadComposition(yaml, logger);

    EXPECT_FALSE(std::filesystem::exists(input_errors)); // no input errors -> file never created
}

/**
 * @brief The mandated file-reference layout parses every referenced config with unit conversions.
 *
 * Each composition entry is a path string; the loader must open the referenced file, unwrap its
 * `*_config:` key, and parse the body exactly like an inline config (diameter -> radius included).
 */
TEST(CompositionLoader, ParsesFileReferenceLayout) {
    const std::filesystem::path base = freshRefBase("cl_ref_layout");
    const std::filesystem::path compose = writeReferenceComposition(base);
    ErrorLogger logger; // stderr only

    const types::SimulationCompositionData composition = loadComposition(compose, logger);

    ASSERT_EQ(composition.simulation_mission_groups.size(), 1u);
    const auto& [sim, missions] = composition.simulation_mission_groups.front();
    EXPECT_DOUBLE_EQ(sim.map_resolution.force_numerical_value_in(cm), 10.0);
    EXPECT_DOUBLE_EQ(sim.initial_drone_position.y.force_numerical_value_in(cm), 25.0);
    EXPECT_DOUBLE_EQ(sim.initial_angle.force_numerical_value_in(deg), 180.0);
    EXPECT_DOUBLE_EQ(sim.map_offset.z.force_numerical_value_in(cm), 3.0);

    ASSERT_EQ(missions.size(), 1u);
    EXPECT_EQ(missions[0].max_steps, 77u);
    EXPECT_DOUBLE_EQ(missions[0].gps_resolution.force_numerical_value_in(cm), 5.0);
    EXPECT_DOUBLE_EQ(missions[0].mission_bounds.max_height.force_numerical_value_in(cm), 130.0);

    ASSERT_EQ(composition.drones.size(), 1u);
    EXPECT_DOUBLE_EQ(composition.drones[0].radius.force_numerical_value_in(cm), 4.0); // 8/2
    ASSERT_EQ(composition.lidars.size(), 1u);
    EXPECT_DOUBLE_EQ(composition.lidars[0].z_max.force_numerical_value_in(cm), 150.0);
    EXPECT_EQ(composition.lidars[0].fov_circles, 3u);
}

/**
 * @brief A relative map_filename in a referenced simulation is rebased onto the composition's dir.
 *
 * "map/tiny.npy" must come back as "<base>/map/tiny.npy" so map loading works from any CWD (inline
 * configs, by contrast, keep map_filename verbatim — asserted by ParsesAllSectionsWithConversions).
 */
TEST(CompositionLoader, ReferenceMapFilenameRebasedToCompositionDir) {
    const std::filesystem::path base = freshRefBase("cl_ref_rebase");
    const std::filesystem::path compose = writeReferenceComposition(base);
    ErrorLogger logger;

    const types::SimulationCompositionData composition = loadComposition(compose, logger);

    ASSERT_EQ(composition.simulation_mission_groups.size(), 1u);
    const auto& [sim, missions] = composition.simulation_mission_groups.front();
    (void)missions;
    EXPECT_EQ(sim.map_filename, base / "map" / "tiny.npy");
}

/**
 * @brief Inline and referenced entries can be mixed in one list; both parse, order preserved.
 */
TEST(CompositionLoader, MixedInlineAndReferenceEntries) {
    const std::filesystem::path base = freshRefBase("cl_ref_mixed");
    (void)writeRefYaml(base, "drone/drn.yaml",
                       "drone_config:\n"
                       "  dimensions_cm: 8\n"
                       "  max_rotate_deg: 90\n"
                       "  max_advance_cm: 30\n"
                       "  max_elevate_cm: 20\n");
    const std::filesystem::path compose =
        writeRefYaml(base, "compose.yaml",
                     "simulation_compositions:\n"
                     "  simulations: []\n"
                     "  drone_configs:\n"
                     "    - dimensions_cm: 30\n"          // inline map entry
                     "      max_rotate_deg: 45\n"
                     "      max_advance_cm: 50\n"
                     "      max_elevate_cm: 40\n"
                     "    - \"drone/drn.yaml\"\n"         // referenced path entry
                     "  lidar_configs: []\n");
    ErrorLogger logger;

    const types::SimulationCompositionData composition = loadComposition(compose, logger);

    ASSERT_EQ(composition.drones.size(), 2u);
    EXPECT_DOUBLE_EQ(composition.drones[0].radius.force_numerical_value_in(cm), 15.0); // inline 30/2
    EXPECT_DOUBLE_EQ(composition.drones[1].radius.force_numerical_value_in(cm), 4.0);  // referenced 8/2
}

/**
 * @brief A reference to a missing file is recovered: logged, defaults used, no throw.
 */
TEST(CompositionLoader, MissingReferencedFileRecoversWithDefaults) {
    const std::filesystem::path base = freshRefBase("cl_ref_missing");
    const std::filesystem::path compose =
        writeRefYaml(base, "compose.yaml",
                     "simulation_compositions:\n"
                     "  simulations: []\n"
                     "  drone_configs: [\"drone/does_not_exist.yaml\"]\n"
                     "  lidar_configs: []\n");
    const std::filesystem::path errors_log = freshTempPath("cl_ref_missing_errors.log");
    ErrorLogger logger{errors_log};

    const types::SimulationCompositionData composition = loadComposition(compose, logger);

    // The entry is kept with all-default values (recover, don't drop or crash) and logged.
    ASSERT_EQ(composition.drones.size(), 1u);
    EXPECT_DOUBLE_EQ(composition.drones[0].radius.force_numerical_value_in(cm), 0.0);
    EXPECT_NE(readAll(errors_log).find("CONFIG_REF_LOAD_FAILED"), std::string::npos);
}

/**
 * @brief A referenced file missing its `*_config:` wrapper key is logged and parsed as defaults.
 */
TEST(CompositionLoader, ReferenceMissingWrapperKeyLogs) {
    const std::filesystem::path base = freshRefBase("cl_ref_nowrap");
    // Valid YAML, but the body is not wrapped under `drone_config:` as the layout mandates.
    (void)writeRefYaml(base, "drone/unwrapped.yaml",
                       "dimensions_cm: 8\n"
                       "max_rotate_deg: 90\n");
    const std::filesystem::path compose =
        writeRefYaml(base, "compose.yaml",
                     "simulation_compositions:\n"
                     "  simulations: []\n"
                     "  drone_configs: [\"drone/unwrapped.yaml\"]\n"
                     "  lidar_configs: []\n");
    const std::filesystem::path errors_log = freshTempPath("cl_ref_nowrap_errors.log");
    ErrorLogger logger{errors_log};

    const types::SimulationCompositionData composition = loadComposition(compose, logger);

    ASSERT_EQ(composition.drones.size(), 1u);
    EXPECT_DOUBLE_EQ(composition.drones[0].radius.force_numerical_value_in(cm), 0.0);
    EXPECT_NE(readAll(errors_log).find("CONFIG_REF_MISSING_KEY"), std::string::npos);
}

/**
 * @brief A scalar that fails to convert is recovered with the default and logged as a bad value.
 */
TEST(CompositionLoader, BadScalarValueRecoversWithDefault) {
    constexpr const char* kBadValueYaml = R"(
simulation_compositions:
  simulations:
    - simulation_config:
        map_filename: data_maps/single_voxel_x2_y4_z2.npy
        map_resolution_cm: 10.0
        initial_drone_position: {x_cm: 5, y_cm: 6, height_cm: 7}
        initial_angle_deg: 0
        map_axes_offset: {x_offset: 0, y_offset: 0, height_offset: 0}
      mission_configs:
        - max_steps: notanumber
          gps_resolution_cm: 4
          boundaries:
            x_boundary: {min_cm: 0, max_cm: 100}
            y_boundary: {min_cm: 0, max_cm: 100}
            height_boundary: {min_cm: 0, max_cm: 100}
  drone_configs:
    - {dimensions_cm: 30, max_rotate_deg: 45, max_advance_cm: 50, max_elevate_cm: 40}
  lidar_configs:
    - {z_min_cm: 20, z_max_cm: 120, d_cm: 2.5, fov_circles: 5}
)";
    const std::filesystem::path yaml = writeTempYaml("composition_bad_value", kBadValueYaml);
    const std::filesystem::path errors_log = freshTempPath("cl_bad_value_errors.log");
    ErrorLogger logger{errors_log};

    const types::SimulationCompositionData composition = loadComposition(yaml, logger);

    ASSERT_EQ(composition.simulation_mission_groups.size(), 1u);
    const auto& [sim, missions] = composition.simulation_mission_groups.front();
    (void)sim;
    ASSERT_EQ(missions.size(), 1u);
    EXPECT_EQ(missions[0].max_steps, 0u); // unparsable -> the scalar's fallback default
    // Neighbouring valid keys are unaffected by the one bad value.
    EXPECT_DOUBLE_EQ(missions[0].gps_resolution.force_numerical_value_in(cm), 4.0);
    EXPECT_NE(readAll(errors_log).find("CONFIG_BAD_VALUE"), std::string::npos);
}