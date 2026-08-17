#include "IntegrationTestFixture.h"

#include <drone_mapper/CompositionLoader.h>
#include <drone_mapper/ErrorLogger.h>
#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/SimulationManager.h>
#include <drone_mapper/SimulationOutputWriter.h>
#include <drone_mapper/SimulationRunFactoryImpl.h>

#include <gtest/gtest.h>

#include <filesystem>
#include <memory>
#include <string>
#include <vector>

// Integration test #1: the ENTIRE production flow with the REAL mapping algorithm, driven exactly
// as `main` drives it — loadComposition() -> SimulationManager{SimulationRunFactoryImpl} -> run() ->
// writeSimulationOutput(). It uses a small file-reference composition (the mandated layout) that spans
// BOTH datasets: a tiny synthetic map from `data_maps/` (asserted to map perfectly) and a real
// `inputs/` scenario (asserted to map a rich portion within a step budget). This exercises the
// reference-format loader, MockGPS/MockMovement/MockLidar, DroneControl, MissionControl,
// MappingAlgorithm, ScanResultToVoxels, Map3DImpl, MapsComparison, SimulationManager, and the writer.

namespace {

using namespace drone_mapper;
namespace fs = std::filesystem;

// Path/file helpers come from the shared fixture (public statics on `Integration`).
using drone_mapper_test::IntegrationFixture;

/// @copydoc drone_mapper_test::IntegrationFixture::sourceDir
[[nodiscard]] fs::path sourceDir() {
    return IntegrationFixture::sourceDir();
}

/// @copydoc drone_mapper_test::IntegrationFixture::writeFile
void writeFile(const fs::path& path, const std::string& content) {
    IntegrationFixture::writeFile(path, content);
}

/**
 * @brief Find the run whose hidden-map filename matches @p stem_with_ext.
 * @param report Aggregate report to search.
 * @param stem_with_ext Map filename to match (e.g. "scenario_small.npy").
 * @return Pointer to the matching run, or nullptr.
 */
[[nodiscard]] const types::SimulationResult* findRun(const types::SimulationManagerReport& report,
                                                     const std::string& stem_with_ext) {
    for (const types::SimulationResult& run : report.runs) {
        if (run.simulation_config.map_filename.filename().string() == stem_with_ext) {
            return &run;
        }
    }
    return nullptr;
}

/**
 * @brief Build a minimal file-reference composition spanning both datasets in a temp directory.
 * @param root Base directory to populate.
 * @return Path to the written `compose.yaml`.
 * @note Map paths are absolute (under the source dir) so the run never depends on the CWD; the
 *       referenced config files are relative, exercising the reference-format loader end-to-end.
 */
[[nodiscard]] fs::path buildComposition(const fs::path& root) {
    const std::string voxel_map = (sourceDir() / "data_maps" / "single_voxel_x2_y4_z2.npy").string();
    const std::string small_map = (sourceDir() / "inputs" / "map" / "scenario_small.npy").string();

    writeFile(root / "compose.yaml",
              "simulation_compositions:\n"
              "  simulations:\n"
              "    - simulation_config: \"sim/voxel.yaml\"\n"
              "      mission_configs: [\"mis/voxel.yaml\"]\n"
              "    - simulation_config: \"sim/small.yaml\"\n"
              "      mission_configs: [\"mis/small.yaml\"]\n"
              "  drone_configs: [\"drn/d.yaml\"]\n"
              "  lidar_configs: [\"lid/l.yaml\"]\n");

    writeFile(root / "sim" / "voxel.yaml",
              "simulation_config:\n"
              "  map_filename: \"" + voxel_map + "\"\n"
              "  map_resolution_cm: 10\n"
              "  initial_drone_position: {x_cm: 25, y_cm: 25, height_cm: 25}\n"
              "  initial_angle_deg: 0\n"
              "  map_axes_offset: {x_offset: 0, y_offset: 0, height_offset: 0}\n");
    writeFile(root / "sim" / "small.yaml",
              "simulation_config:\n"
              "  map_filename: \"" + small_map + "\"\n"
              "  map_resolution_cm: 10\n"
              "  initial_drone_position: {x_cm: 100, y_cm: 100, height_cm: 100}\n"
              "  initial_angle_deg: 0\n"
              "  map_axes_offset: {x_offset: 0, y_offset: 0, height_offset: 0}\n");

    // gps_resolution == map_resolution keeps the output grid aligned with the hidden map, so scoring
    // is apples-to-apples (not the cross-resolution bonus case).
    writeFile(root / "mis" / "voxel.yaml",
              "mission_config:\n"
              "  max_steps: 5000\n"
              "  boundaries: {x_boundary: {min_cm: 0, max_cm: 50}, y_boundary: {min_cm: 0, max_cm: 50},"
              " height_boundary: {min_cm: 0, max_cm: 50}}\n"
              "  gps_resolution_cm: 10\n");
    writeFile(root / "mis" / "small.yaml",
              "mission_config:\n"
              "  max_steps: 60\n"
              "  boundaries: {x_boundary: {min_cm: 0, max_cm: 200}, y_boundary: {min_cm: 0, max_cm: 200},"
              " height_boundary: {min_cm: 0, max_cm: 200}}\n"
              "  gps_resolution_cm: 10\n");

    writeFile(root / "drn" / "d.yaml",
              "drone_config: {dimensions_cm: 10, max_rotate_deg: 90, max_advance_cm: 10, max_elevate_cm: 10}\n");
    writeFile(root / "lid" / "l.yaml",
              "lidar_config: {z_min_cm: 5, z_max_cm: 120, d_cm: 2.5, fov_circles: 3}\n");

    return root / "compose.yaml";
}

} // namespace

/**
 * @brief The full simulator flow maps both a synthetic and a real scenario map end-to-end.
 *
 * The single-voxel map must be mapped perfectly (score 100, the occupied voxel present in the saved
 * output map). The 20^3 `scenario_small` map, given a small step budget, must map a rich portion
 * (non-error, positive score, occupied voxels written) — proving the whole sensor/voxel/map/compare
 * pipeline works with the real algorithm. Finally the score report is written and structurally valid.
 */
TEST_F(Integration, RealAlgorithmMapsBothDatasetsEndToEnd) {
    const fs::path root = fs::path{::testing::TempDir()} / "intg_real";
    fs::remove_all(root);
    const fs::path compose = buildComposition(root);

    ErrorLogger logger; // stderr only — keeps the test off the filesystem for logs.

    // 1) Reference-format loader: both simulations, the drone, and the lidar parse (not all-empty).
    //    Capture the source config paths so the report can label runs by them.
    CompositionPaths paths;
    const types::SimulationCompositionData composition = loadComposition(compose, logger, &paths);
    ASSERT_EQ(composition.simulation_mission_groups.size(), 2u);
    ASSERT_EQ(composition.drones.size(), 1u);
    ASSERT_EQ(composition.lidars.size(), 1u);
    ASSERT_EQ(paths.simulation_paths.size(), 2u);
    ASSERT_EQ(paths.drone_paths.size(), 1u);

    // 2) Run every combination through the real factory + manager.
    const fs::path out_dir = root / "out";
    SimulationManager manager{std::make_unique<SimulationRunFactoryImpl>(), logger};
    const types::SimulationManagerReport report = manager.run(composition, out_dir);
    ASSERT_EQ(report.runs.size(), 2u);

    // 3a) Synthetic single-voxel map: perfect mapping.
    const types::SimulationResult* voxel = findRun(report, "single_voxel_x2_y4_z2.npy");
    ASSERT_NE(voxel, nullptr);
    ASSERT_FALSE(voxel->mission_results.empty());
    EXPECT_EQ(voxel->mission_results.front().status, types::MissionRunStatus::Completed);
    EXPECT_GT(voxel->mission_results.front().steps, 0u);
    EXPECT_DOUBLE_EQ(voxel->mission_score, 100.0);

    // The saved output map must actually contain the known occupied voxel (index {2,4,2} at 10 cm ->
    // world centre {25,45,25}). This checks the scan -> voxel -> map -> save path, not just the score.
    const std::shared_ptr<NpyArray> voxel_arr = Map3DImpl::loadArray(voxel->output_map_file);
    const Map3DImpl voxel_map{voxel_arr, voxel->output_map_config};
    EXPECT_EQ(voxel_map.atVoxel(Position3D{25.0 * x_extent[cm], 45.0 * y_extent[cm], 25.0 * z_extent[cm]}),
              types::VoxelOccupancy::Occupied);

    // 3b) Real scenario map: a rich, non-trivial partial map within the step budget.
    const types::SimulationResult* small = findRun(report, "scenario_small.npy");
    ASSERT_NE(small, nullptr);
    ASSERT_FALSE(small->mission_results.empty());
    EXPECT_NE(small->mission_results.front().status, types::MissionRunStatus::Error);
    EXPECT_GT(small->mission_score, 0.0);
    EXPECT_LE(small->mission_score, 100.0);
    EXPECT_TRUE(fs::exists(small->output_map_file));

    // 4) Writer (path-based): with the file-reference composition, the report labels each level by
    //    its source config file path, matching the PDF's simulation_output.yaml sample.
    writeSimulationOutput(report, out_dir, composition.composition_file, paths);
    const fs::path report_yaml = out_dir / "simulation_output.yaml";
    ASSERT_TRUE(fs::exists(report_yaml));
    const std::string text = readAll(report_yaml);
    EXPECT_NE(text.find("score_report:"), std::string::npos);
    EXPECT_NE(text.find("resolution_request_status"), std::string::npos);
    EXPECT_NE(text.find("simulation_config:"), std::string::npos);
    EXPECT_NE(text.find("mission_config:"), std::string::npos);
    EXPECT_NE(text.find("drone_config:"), std::string::npos);
    EXPECT_NE(text.find("lidar_config:"), std::string::npos);
    // The referenced config paths are echoed verbatim (as written in the composition).
    EXPECT_NE(text.find("sim/voxel.yaml"), std::string::npos);
    EXPECT_NE(text.find("drn/d.yaml"), std::string::npos);
    EXPECT_NE(text.find("lid/l.yaml"), std::string::npos);
}

/**
 * @brief With an inline composition (no config paths), the report falls back to value-based labels.
 *
 * Passing an empty `CompositionPaths` must produce the value-based shape: simulations labelled by
 * `map_filename` and runs by their `output_map` filename (not `drone_config`/`lidar_config`).
 */
TEST_F(Integration, OutputWriterValueBasedFallbackWhenNoPaths) {
    const fs::path root = fs::path{::testing::TempDir()} / "intg_valuewriter";
    fs::remove_all(root);
    const fs::path compose = buildComposition(root);

    ErrorLogger logger;
    const types::SimulationCompositionData composition = loadComposition(compose, logger);
    const fs::path out_dir = root / "out";
    SimulationManager manager{std::make_unique<SimulationRunFactoryImpl>(), logger};
    const types::SimulationManagerReport report = manager.run(composition, out_dir);

    // No paths supplied -> value-based labelling.
    writeSimulationOutput(report, out_dir, composition.composition_file, CompositionPaths{});
    const std::string text = readAll(out_dir / "simulation_output.yaml");

    EXPECT_NE(text.find("score_report:"), std::string::npos);
    EXPECT_NE(text.find("map_filename:"), std::string::npos);          // value-based simulation label
    EXPECT_NE(text.find("single_voxel_x2_y4_z2.npy"), std::string::npos);
    EXPECT_NE(text.find("output_map:"), std::string::npos);            // value-based run label
    EXPECT_EQ(text.find("drone_config:"), std::string::npos);          // no path labels
}

/**
 * @brief A nonzero map_axes_offset places the drone inside the (shifted) map, not below it.
 *
 * The single-voxel map is shifted up 100 cm via the offset. The configured drone position and mission
 * bounds are relative to the map origin, so the factory must translate them into world coordinates;
 * otherwise the drone spawns at world z=25 — below the map at world z[100,150] — and the algorithm
 * finishes on step 1 with score 0 (the house-scenario bug). With the fix the run maps the map exactly,
 * just like the un-offset case.
 */
TEST_F(Integration, NonzeroMapOffsetPlacesDroneInsideMap) {
    types::SimulationCompositionData composition{};

    types::SimulationConfigData sim{};
    sim.map_filename = sourceDir() / "data_maps" / "single_voxel_x2_y4_z2.npy"; // 5^3 @ 10 cm
    sim.map_resolution = 10.0 * cm;
    sim.map_offset = Position3D{0.0 * x_extent[cm], 0.0 * y_extent[cm], 100.0 * z_extent[cm]};
    // Map-relative pose/bounds: the drone at local z=25 must become world z=125 (inside the map).
    sim.initial_drone_position =
        Position3D{25.0 * x_extent[cm], 25.0 * y_extent[cm], 25.0 * z_extent[cm]};
    sim.initial_angle = 0.0 * horizontal_angle[deg];

    types::MissionConfigData mission{};
    mission.max_steps = 5000;
    mission.gps_resolution = 10.0 * cm;
    mission.output_mapping_resolution_factor = 1.0;
    mission.mission_bounds = types::MappingBounds{
        0.0 * x_extent[cm], 50.0 * x_extent[cm], 0.0 * y_extent[cm],
        50.0 * y_extent[cm], 0.0 * z_extent[cm], 50.0 * z_extent[cm],
    };

    composition.simulation_mission_groups.emplace_back(
        sim, std::vector<types::MissionConfigData>{mission});
    composition.drones.push_back(
        types::DroneConfigData{5.0 * cm, 90.0 * horizontal_angle[deg], 10.0 * cm, 10.0 * cm});
    composition.lidars.push_back(types::LidarConfigData{5.0 * cm, 120.0 * cm, 2.5 * cm, 3});

    const fs::path out_dir = fs::path{::testing::TempDir()} / "intg_offset";
    fs::remove_all(out_dir);
    ErrorLogger logger;
    SimulationManager manager{std::make_unique<SimulationRunFactoryImpl>(), logger};
    const types::SimulationManagerReport report = manager.run(composition, out_dir);

    ASSERT_EQ(report.runs.size(), 1u);
    const types::SimulationResult& run = report.runs.front();
    ASSERT_FALSE(run.mission_results.empty());
    // Placed inside the map, the drone actually explores (not the 1-step "outside grid" finish) and
    // maps the shifted map perfectly.
    EXPECT_EQ(run.mission_results.front().status, types::MissionRunStatus::Completed);
    EXPECT_GT(run.mission_results.front().steps, 1u);
    EXPECT_DOUBLE_EQ(run.mission_score, 100.0);

    // The occupied voxel sits at world {25,45,125} (local {25,45,25} + offset {0,0,100}).
    const std::shared_ptr<NpyArray> out_arr = Map3DImpl::loadArray(run.output_map_file);
    const Map3DImpl out_map{out_arr, run.output_map_config};
    EXPECT_EQ(out_map.atVoxel(Position3D{25.0 * x_extent[cm], 45.0 * y_extent[cm], 125.0 * z_extent[cm]}),
              types::VoxelOccupancy::Occupied);
}

/**
 * @brief A mission whose boundaries have a non-zero minimum maps that sub-region, not the map origin.
 *
 * The output grid is anchored at the mission region's world origin (map_offset + boundary minimum),
 * so a drone positioned inside that region maps it. Regression for the bug where the grid ignored a
 * non-zero boundary minimum and anchored at the map origin — leaving the drone outside its own grid
 * and finishing on step 1 with score 0 (as the inputs' "_room" missions did).
 */
TEST_F(Integration, NonzeroBoundaryMinAnchorsGridToMissionRegion) {
    types::SimulationCompositionData composition{};

    types::SimulationConfigData sim{};
    sim.map_filename = sourceDir() / "data_maps" / "single_voxel_x2_y4_z2.npy"; // occupied at {25,45,25}
    sim.map_resolution = 10.0 * cm;
    // Drone starts in empty space at y=45 (the same y-slab as the occupied voxel).
    sim.initial_drone_position =
        Position3D{15.0 * x_extent[cm], 45.0 * y_extent[cm], 25.0 * z_extent[cm]};
    sim.initial_angle = 0.0 * horizontal_angle[deg];

    types::MissionConfigData mission{};
    mission.max_steps = 500;
    mission.gps_resolution = 10.0 * cm;
    mission.output_mapping_resolution_factor = 1.0;
    // y_boundary starts at 40 (a non-zero minimum): the mission region is the slab y in [40,50].
    mission.mission_bounds = types::MappingBounds{
        0.0 * x_extent[cm], 50.0 * x_extent[cm], 40.0 * y_extent[cm],
        50.0 * y_extent[cm], 0.0 * z_extent[cm], 50.0 * z_extent[cm],
    };

    composition.simulation_mission_groups.emplace_back(
        sim, std::vector<types::MissionConfigData>{mission});
    composition.drones.push_back(
        types::DroneConfigData{5.0 * cm, 90.0 * horizontal_angle[deg], 10.0 * cm, 10.0 * cm});
    composition.lidars.push_back(types::LidarConfigData{5.0 * cm, 120.0 * cm, 2.5 * cm, 3});

    const fs::path out_dir = fs::path{::testing::TempDir()} / "intg_boundmin";
    fs::remove_all(out_dir);
    ErrorLogger logger;
    SimulationManager manager{std::make_unique<SimulationRunFactoryImpl>(), logger};
    const types::SimulationManagerReport report = manager.run(composition, out_dir);

    ASSERT_EQ(report.runs.size(), 1u);
    const types::SimulationResult& run = report.runs.front();
    ASSERT_FALSE(run.mission_results.empty());
    // Placed inside the correctly-anchored grid, the drone explores (not the 1-step finish) and maps
    // the region's occupied voxel perfectly.
    EXPECT_EQ(run.mission_results.front().status, types::MissionRunStatus::Completed);
    EXPECT_GT(run.mission_results.front().steps, 1u);
    EXPECT_DOUBLE_EQ(run.mission_score, 100.0);
}
