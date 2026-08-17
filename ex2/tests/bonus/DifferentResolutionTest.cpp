#include <drone_mapper/ErrorLogger.h>
#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/MapsComparison.h>
#include <drone_mapper/SimulationManager.h>
#include <drone_mapper/SimulationRunFactoryImpl.h>
#include <drone_mapper/Units.h>

#include <gtest/gtest.h>

#include <cstddef>
#include <filesystem>
#include <memory>
#include <vector>

// BONUS: "Supporting different resolution requests."
//
// The simulator can write the output map at a resolution DIFFERENT from the input map (driven by the
// mission's GPS precision x output_mapping_resolution_factor), and MapsComparison scores that output
// against the different-resolution ground-truth map. `compare` evaluates on the ORIGIN (ground-truth)
// grid and samples every map by world position, so a target of a different resolution/offset is
// handled positionally.
//
// This suite (filter `BonusResolution.*`, see bonus.txt) proves the capability two ways:
//   - MapsComparison over two maps of different resolutions (both directions + a mismatch), and
//   - the whole simulator flow honouring a coarser-than-map output resolution request end-to-end.

#ifndef DRONE_MAPPER_SOURCE_DIR
#define DRONE_MAPPER_SOURCE_DIR "."
#endif

namespace {

using namespace drone_mapper;
namespace fs = std::filesystem;

/// Repository source root, injected by CMake so the data map is found regardless of the CWD.
[[nodiscard]] fs::path sourceDir() {
    return fs::path{DRONE_MAPPER_SOURCE_DIR};
}

/**
 * @brief Cubic, offset-free map geometry `[0, extent]³` at a given voxel size.
 * @param res_cm Voxel edge length (cm).
 * @param extent_cm Cube side length (cm).
 * @return The `MapConfig`.
 */
[[nodiscard]] types::MapConfig cubicConfig(double res_cm, double extent_cm) {
    const types::MappingBounds bounds{
        XLength{}, extent_cm * x_extent[cm],
        YLength{}, extent_cm * y_extent[cm],
        ZLength{}, extent_cm * z_extent[cm],
    };
    return types::MapConfig{bounds, Position3D{}, res_cm * cm};
}

/**
 * @brief Build a map at @p res_cm over `[0, extent]³` with every voxel whose CENTRE lies inside the
 *        world box `[lo, hi]³` marked Occupied.
 * @param res_cm Voxel edge length (cm) — this is what differs between the compared maps.
 * @param extent_cm Cube side length (cm).
 * @param lo_cm,hi_cm World box marked occupied (half-open `[lo, hi)` per axis).
 * @return A ready-to-compare `Map3DImpl`.
 * @note Marking by world box (not by index) makes the same solid region occupied at any resolution,
 *       which is exactly what a cross-resolution comparison must recognise as identical.
 */
[[nodiscard]] Map3DImpl boxMap(double res_cm, double extent_cm, double lo_cm, double hi_cm) {
    const types::MapConfig cfg = cubicConfig(res_cm, extent_cm);
    Map3DImpl map{Map3DImpl::makeEmptyArray(cfg), cfg};
    const int n = static_cast<int>(extent_cm / res_cm + 0.5);
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            for (int k = 0; k < n; ++k) {
                const double cx = (static_cast<double>(i) + 0.5) * res_cm;
                const double cy = (static_cast<double>(j) + 0.5) * res_cm;
                const double cz = (static_cast<double>(k) + 0.5) * res_cm;
                if (cx >= lo_cm && cx < hi_cm && cy >= lo_cm && cy < hi_cm && cz >= lo_cm &&
                    cz < hi_cm) {
                    map.set(Position3D{cx * x_extent[cm], cy * y_extent[cm], cz * z_extent[cm]},
                            types::VoxelOccupancy::Occupied);
                }
            }
        }
    }
    return map;
}

/// A single-scenario composition over the synthetic single-voxel map, requesting a coarser output.
[[nodiscard]] types::SimulationCompositionData coarserOutputComposition() {
    types::SimulationCompositionData composition{};

    types::SimulationConfigData sim{};
    sim.map_filename = sourceDir() / "data_maps" / "single_voxel_x2_y4_z2.npy"; // 10 cm voxels
    sim.map_resolution = 10.0 * cm;
    sim.initial_drone_position =
        Position3D{25.0 * x_extent[cm], 25.0 * y_extent[cm], 25.0 * z_extent[cm]};
    sim.initial_angle = 0.0 * horizontal_angle[deg];

    types::MissionConfigData mission{};
    mission.max_steps = 200;
    mission.gps_resolution = 10.0 * cm;
    mission.output_mapping_resolution_factor = 2.0; // request output at 2x the GPS precision -> 20 cm
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
 * @brief A coarse ground truth and a fine output of the SAME solid region score a perfect match.
 *
 * Origin at 20 cm and target at 10 cm both mark the world box [0,20)³ occupied; comparing across the
 * differing grids must recognise them as identical (IoU 100).
 */
TEST(BonusResolution, CoarseGroundTruthVsFineOutputSameStructure) {
    const Map3DImpl origin = boxMap(20.0, 40.0, 0.0, 20.0);
    Map3DImpl target = boxMap(10.0, 40.0, 0.0, 20.0);

    const std::vector<double> scores = MapsComparison::compare(origin, {&target});
    ASSERT_EQ(scores.size(), 1u);
    EXPECT_DOUBLE_EQ(scores[0], 100.0);
}

/**
 * @brief The reverse direction (fine ground truth, coarse output) is also a perfect match.
 */
TEST(BonusResolution, FineGroundTruthVsCoarseOutputSameStructure) {
    const Map3DImpl origin = boxMap(10.0, 40.0, 0.0, 20.0);
    Map3DImpl target = boxMap(20.0, 40.0, 0.0, 20.0);

    const std::vector<double> scores = MapsComparison::compare(origin, {&target});
    ASSERT_EQ(scores.size(), 1u);
    EXPECT_DOUBLE_EQ(scores[0], 100.0);
}

/**
 * @brief Different-resolution maps of DIFFERENT regions still score near zero.
 *
 * Proves the cross-resolution comparison discriminates structure, not just that it returns a number:
 * origin (20 cm) occupies [0,20)³ while target (10 cm) occupies the disjoint [20,40)³.
 */
TEST(BonusResolution, DifferentResolutionDifferentStructureScoresLow) {
    const Map3DImpl origin = boxMap(20.0, 40.0, 0.0, 20.0);
    const Map3DImpl target = boxMap(10.0, 40.0, 20.0, 40.0);

    const std::vector<double> scores = MapsComparison::compare(origin, {const_cast<Map3DImpl*>(&target)});
    ASSERT_EQ(scores.size(), 1u);
    EXPECT_DOUBLE_EQ(scores[0], 0.0);
}

/**
 * @brief The whole simulator honours a request for an output resolution different from the map.
 *
 * With gps_resolution 10 cm and output_mapping_resolution_factor 2, the run must ACCEPT the request,
 * write the output map at 20 cm (the input map is 10 cm), and still produce an in-range score.
 */
TEST(BonusResolution, EndToEndCoarserOutputRequestHonored) {
    const fs::path out_dir = fs::path{::testing::TempDir()} / "bonus_res";
    fs::remove_all(out_dir);

    ErrorLogger logger; // stderr only
    SimulationManager manager{std::make_unique<SimulationRunFactoryImpl>(), logger};
    const types::SimulationManagerReport report = manager.run(coarserOutputComposition(), out_dir);

    ASSERT_EQ(report.runs.size(), 1u);
    const types::SimulationResult& run = report.runs.front();

    // The different-resolution request is accepted, and the actual output resolution differs from the
    // 10 cm input map.
    EXPECT_EQ(run.resolution_request_status, types::ResolutionRequestStatus::Accepted);
    EXPECT_DOUBLE_EQ(run.output_map_config.resolution.force_numerical_value_in(cm), 20.0);

    // The run still scores against the 10 cm ground truth (cross-resolution), without erroring.
    ASSERT_FALSE(run.mission_results.empty());
    EXPECT_NE(run.mission_results.front().status, types::MissionRunStatus::Error);
    EXPECT_GE(run.mission_score, 0.0);
    EXPECT_LE(run.mission_score, 100.0);
}
