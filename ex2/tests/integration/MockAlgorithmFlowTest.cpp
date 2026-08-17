#include "IntegrationTestFixture.h"

#include <drone_mapper/DroneControlImpl.h>
#include <drone_mapper/ErrorLogger.h>
#include <drone_mapper/IMap3D.h>
#include <drone_mapper/IMappingAlgorithm.h>
#include <drone_mapper/ISimulationRun.h>
#include <drone_mapper/ISimulationRunFactory.h>
#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/MissionControlImpl.h>
#include <drone_mapper/MockGPS.h>
#include <drone_mapper/MockLidar.h>
#include <drone_mapper/MockMovement.h>
#include <drone_mapper/SimulationManager.h>
#include <drone_mapper/SimulationRunImpl.h>

#include <gtest/gtest.h>

#include <cstddef>
#include <deque>
#include <filesystem>
#include <memory>
#include <string>
#include <utility>
#include <vector>

// Integration test #2: the whole flow with a MOCK (scripted) mapping algorithm. Everything else is
// real — MockGPS/MockMovement/MockLidar, DroneControl, MissionControl, ScanResultToVoxels, Map3DImpl,
// MapsComparison, SimulationManager, SimulationRunImpl. Only the planner is replaced, by a fixed
// script of scans, so the test drives the sensor->voxel->map->compare pipeline deterministically and
// isolates it from the real algorithm's behaviour. A test-local ISimulationRunFactory injects the
// scripted algorithm; the real SimulationManager expands and runs the composition.

#ifndef DRONE_MAPPER_SOURCE_DIR
#define DRONE_MAPPER_SOURCE_DIR "."
#endif

namespace {

using namespace drone_mapper;
namespace fs = std::filesystem;

/// Repository source root, injected by CMake so data maps are found regardless of the CWD.
[[nodiscard]] fs::path sourceDir() {
    return fs::path{DRONE_MAPPER_SOURCE_DIR};
}

/**
 * @brief Full-extent map geometry for a freshly loaded array (mirrors the real factory's helper).
 * @param array Loaded voxel grid; its Shape() gives per-axis voxel counts.
 * @param offset World position of voxel index {0,0,0}.
 * @param res Voxel edge length (cm).
 * @return A MapConfig spanning `[offset, offset + Shape()*res]` per axis.
 */
[[nodiscard]] types::MapConfig fullExtentConfig(const NpyArray& array, const Position3D& offset,
                                                PhysicalLength res) {
    const NpyArray::shape_t& shape = array.Shape();
    const double r = res.force_numerical_value_in(cm);
    const double sx = shape.size() > 0 ? static_cast<double>(shape[0]) : 0.0;
    const double sy = shape.size() > 1 ? static_cast<double>(shape[1]) : 0.0;
    const double sz = shape.size() > 2 ? static_cast<double>(shape[2]) : 0.0;
    const types::MappingBounds bounds{
        offset.x, offset.x + (sx * r) * x_extent[cm],
        offset.y, offset.y + (sy * r) * y_extent[cm],
        offset.z, offset.z + (sz * r) * z_extent[cm],
    };
    return types::MapConfig{bounds, offset, res};
}

/**
 * @brief A mapping algorithm that replays a fixed command script, then latches Finished.
 * @note Purely deterministic: it ignores the drone state and scan results, so the pipeline outcome
 *       depends only on the (real) sensors and map math — exactly what an integration test wants to
 *       isolate from the real planner.
 */
class ScriptedAlgorithm : public IMappingAlgorithm {
public:
    ScriptedAlgorithm(const types::MissionConfigData& mission, const types::LidarConfigData& lidar,
                      const types::DroneConfigData& drone, const IMap3D& output_map,
                      std::deque<types::MappingStepCommand> script)
        : IMappingAlgorithm(mission, lidar, drone, output_map), script_(std::move(script)) {}

    [[nodiscard]] types::MappingStepCommand nextStep(const types::DroneState&,
                                                     const types::LidarScanResult*) override {
        if (script_.empty()) {
            types::MappingStepCommand done{};
            done.status = types::AlgorithmStatus::Finished;
            return done;
        }
        const types::MappingStepCommand command = script_.front();
        script_.pop_front();
        return command;
    }

private:
    std::deque<types::MappingStepCommand> script_;
};

/**
 * @brief A six-direction scan sweep (four compass headings + up + down), then Finished.
 * @return The scripted command list.
 * @note Covers all axis directions so the sweep hits the single occupied voxel regardless of whether
 *       the scan orientation is interpreted as absolute or heading-relative.
 */
[[nodiscard]] std::deque<types::MappingStepCommand> buildScript() {
    std::deque<types::MappingStepCommand> script;
    const auto scan = [](double horizontal_deg, double altitude_deg) {
        types::MappingStepCommand command{};
        command.scan_orientation =
            Orientation{horizontal_deg * horizontal_angle[deg], altitude_deg * altitude_angle[deg]};
        command.status = types::AlgorithmStatus::Working;
        return command;
    };
    for (double heading : {0.0, 90.0, 180.0, 270.0}) {
        script.push_back(scan(heading, 0.0));
    }
    script.push_back(scan(0.0, 90.0));   // up
    script.push_back(scan(0.0, -90.0));  // down
    types::MappingStepCommand finished{};
    finished.status = types::AlgorithmStatus::Finished;
    script.push_back(finished);
    return script;
}

/**
 * @brief Test-local factory: the real wiring with the scripted algorithm swapped in for the planner.
 * @note Deliberately mirrors `SimulationRunFactoryImpl::create` so the object graph and lifetimes
 *       match production; the only substitution is `ScriptedAlgorithm` in place of the real one. The
 *       output map is built at the hidden-map resolution so scoring stays apples-to-apples.
 */
class ScriptedAlgoFactory final : public ISimulationRunFactory {
public:
    [[nodiscard]] std::unique_ptr<ISimulationRun>
    create(const types::SimulationConfigData& simulation, const types::MissionConfigData& mission,
           const types::DroneConfigData& drone, const types::LidarConfigData& lidar,
           const fs::path& output_path) override {
        auto hidden_array = Map3DImpl::loadArray(simulation.map_filename);
        const types::MapConfig hidden_config =
            fullExtentConfig(*hidden_array, simulation.map_offset, simulation.map_resolution);
        auto hidden_map = std::make_unique<Map3DImpl>(hidden_array, hidden_config);

        const types::MapConfig output_config{mission.mission_bounds, simulation.map_offset,
                                             simulation.map_resolution};
        auto output_map =
            std::make_unique<Map3DImpl>(Map3DImpl::makeEmptyArray(output_config), output_config);

        auto gps = std::make_unique<MockGPS>(
            simulation.initial_drone_position,
            Orientation{simulation.initial_angle, 0.0 * altitude_angle[deg]}, mission.gps_resolution);
        auto movement = std::make_unique<MockMovement>(*gps);
        auto lidar_impl = std::make_unique<MockLidar>(lidar, *hidden_map, *gps);
        auto algorithm =
            std::make_unique<ScriptedAlgorithm>(mission, lidar, drone, *output_map, buildScript());
        auto drone_control = std::make_unique<DroneControlImpl>(
            drone, mission, *lidar_impl, *gps, *movement, *output_map, *algorithm);

        const fs::path results_dir = output_path / "output_results";
        fs::create_directories(results_dir);
        const fs::path output_map_file =
            results_dir / (simulation.map_filename.stem().string() + "_mock.npy");

        auto mission_control = std::make_unique<MissionControlImpl>(
            mission, drone, *hidden_map, *output_map, *drone_control, output_map_file);

        return std::make_unique<SimulationRunImpl>(
            std::move(hidden_map), std::move(output_map), std::move(gps), std::move(movement),
            std::move(lidar_impl), std::move(algorithm), std::move(drone_control),
            std::move(mission_control), simulation, mission,
            types::ResolutionRequestStatus::Accepted, output_map_file);
    }
};

/// A single-scenario composition over the synthetic single-voxel map (offset 0, 10 cm voxels).
[[nodiscard]] types::SimulationCompositionData singleVoxelComposition() {
    types::SimulationCompositionData composition{};

    types::SimulationConfigData sim{};
    sim.map_filename = sourceDir() / "data_maps" / "single_voxel_x2_y4_z2.npy";
    sim.map_resolution = 10.0 * cm;
    sim.initial_drone_position =
        Position3D{25.0 * x_extent[cm], 25.0 * y_extent[cm], 25.0 * z_extent[cm]};
    sim.initial_angle = 0.0 * horizontal_angle[deg];

    types::MissionConfigData mission{};
    mission.max_steps = 50;
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
    return composition;
}

} // namespace

/**
 * @brief A scripted scan sweep, run through the whole flow, maps the occupied voxel and is scored.
 *
 * With a mock algorithm the run must complete (the script latches Finished), the saved output map
 * must contain the known occupied voxel — proving the real MockLidar -> ScanResultToVoxels ->
 * Map3DImpl path executed — and MapsComparison must return a positive, in-range score assembled into
 * the SimulationResult.
 */
TEST_F(Integration, MockAlgorithmSweepMapsAndScoresEndToEnd) {
    const fs::path out_dir = fs::path{::testing::TempDir()} / "intg_mock";
    fs::remove_all(out_dir);

    ErrorLogger logger; // stderr only
    SimulationManager manager{std::make_unique<ScriptedAlgoFactory>(), logger};
    const types::SimulationManagerReport report = manager.run(singleVoxelComposition(), out_dir);

    ASSERT_EQ(report.runs.size(), 1u);
    const types::SimulationResult& run = report.runs.front();

    // The scripted algorithm latches Finished, so the mission completes (not max_steps/error).
    ASSERT_FALSE(run.mission_results.empty());
    EXPECT_EQ(run.mission_results.front().status, types::MissionRunStatus::Completed);
    EXPECT_GT(run.mission_results.front().steps, 0u);

    // Comparison ran and produced a real, in-range score (not the -1 error sentinel).
    EXPECT_GT(run.mission_score, 0.0);
    EXPECT_LE(run.mission_score, 100.0);

    // The known occupied voxel (index {2,4,2} at 10 cm -> world {25,45,25}) was written to the saved
    // output map: proof the scan -> voxel -> map path ran under the mock planner.
    ASSERT_TRUE(fs::exists(run.output_map_file));
    const std::shared_ptr<NpyArray> out_arr = Map3DImpl::loadArray(run.output_map_file);
    const Map3DImpl out_map{out_arr, run.output_map_config};
    EXPECT_EQ(out_map.atVoxel(Position3D{25.0 * x_extent[cm], 45.0 * y_extent[cm], 25.0 * z_extent[cm]}),
              types::VoxelOccupancy::Occupied);
}
