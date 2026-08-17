#include "IntegrationTestFixture.h"

#include <drone_mapper/CompositionLoader.h>
#include <drone_mapper/ErrorLogger.h>
#include <drone_mapper/SimulationManager.h>
#include <drone_mapper/SimulationRunFactoryImpl.h>

#include <gtest/gtest.h>

#include <cstddef>
#include <filesystem>
#include <memory>
#include <string>
#include <tuple>
#include <vector>

// Whole-flow coverage of EVERY scenario in the provided `inputs/` dataset: the real
// `inputs/sim_compose.yaml` is parsed by the real reference-format loader, and each
// simulation×mission combination is executed through the real factory/manager with the dataset's
// full 2-drone × 2-lidar product. The only deviation from the shipped configs is a small step cap
// (the configured 1000–2000-step budgets make the full dataset a ~6-minute manual check, not a unit
// suite); the caps stay far below the step counts where any known drone-step errors occur, so every
// run must end Completed or MaxSteps — never Error.

namespace {

using namespace drone_mapper;
namespace fs = std::filesystem;

/// Step budget used for every scenario run (see the file comment for why the real budgets are capped).
constexpr std::size_t kStepCap = 80;

} // namespace

/**
 * @brief The provided dataset parses into the full expected composition shape.
 *
 * 5 simulations (house with 2 missions, the rest with 1 → 6 scenario combos), 2 drones, 2 lidars —
 * a 24-run cartesian product — and every referenced map path is rebased to a real file on disk.
 */
TEST_F(Integration, InputsCompositionParsesFullShape) {
    ErrorLogger logger; // stderr only
    const types::SimulationCompositionData composition =
        loadComposition(sourceDir() / "inputs" / "sim_compose.yaml", logger);

    ASSERT_EQ(composition.simulation_mission_groups.size(), 5u);
    std::size_t scenario_combos = 0;
    for (const auto& [sim, missions] : composition.simulation_mission_groups) {
        EXPECT_FALSE(missions.empty());
        scenario_combos += missions.size();
        // The loader rebases relative map paths against the composition directory: they must exist.
        EXPECT_TRUE(fs::exists(sim.map_filename))
            << "missing map: " << sim.map_filename;
    }
    EXPECT_EQ(scenario_combos, 6u); // house has 2 missions, the other 4 sims have 1 each
    EXPECT_EQ(composition.drones.size(), 2u);
    EXPECT_EQ(composition.lidars.size(), 2u);
}

namespace {

/**
 * @brief Run one `inputs/` scenario (a simulation group + one of its missions) end-to-end.
 * @param group_index Simulation group position in `sim_compose.yaml` (0 house, 1 large_out,
 *        2 large_room, 3 small_out, 4 small_room). Selected by index because the out/room variants
 *        share a map and differ only in simulation config (initial pose).
 * @param mission_index Which of the group's missions to run.
 * @param map_stem Expected hidden-map stem — a sanity check that the index picked the right group.
 * @param out_name Distinct output directory name for this test.
 * @note Uses the loaded dataset verbatim — real bounds, GPS resolution, and the full 2×2
 *       drone×lidar product — with only `max_steps` capped to `kStepCap`. Asserts every run in the
 *       product ends without an error status, scores in [0, 100], and saved its output map.
 */
void runScenario(std::size_t group_index, std::size_t mission_index, const std::string& map_stem,
                 const std::string& out_name) {
    ErrorLogger logger; // stderr only
    const types::SimulationCompositionData full =
        loadComposition(Integration::sourceDir() / "inputs" / "sim_compose.yaml", logger);
    ASSERT_LT(group_index, full.simulation_mission_groups.size());

    // Filter the dataset down to the requested simulation + single mission, keeping all drones/lidars.
    const auto& [sim, missions] = full.simulation_mission_groups[group_index];
    ASSERT_EQ(sim.map_filename.stem().string(), map_stem) << "group order changed in sim_compose.yaml";
    ASSERT_LT(mission_index, missions.size());
    types::MissionConfigData mission = missions[mission_index];
    mission.max_steps = kStepCap;

    types::SimulationCompositionData scenario{};
    scenario.composition_file = full.composition_file;
    scenario.drones = full.drones;
    scenario.lidars = full.lidars;
    scenario.simulation_mission_groups.emplace_back(
        sim, std::vector<types::MissionConfigData>{mission});

    const fs::path out_dir = Integration::freshOutDir(out_name);
    SimulationManager manager{std::make_unique<SimulationRunFactoryImpl>(), logger};
    const types::SimulationManagerReport report = manager.run(scenario, out_dir);

    // Full drone×lidar product of the dataset: 2 × 2 runs.
    ASSERT_EQ(report.runs.size(), 4u);
    for (const types::SimulationResult& run : report.runs) {
        ASSERT_FALSE(run.mission_results.empty());
        // On failure, surface the run's own error so the log explains itself (e.g. a transient
        // filesystem problem shows up as a RUN_FACTORY_ERROR/RUN_FAILED message here).
        const types::MissionRunResult& mission_result = run.mission_results.front();
        std::string error_detail;
        for (const types::ErrorRef& err : mission_result.errors) {
            error_detail += " [" + err.code + "] " + err.message;
        }
        EXPECT_NE(mission_result.status, types::MissionRunStatus::Error)
            << map_stem << ": " << run.output_map_file << error_detail;
        EXPECT_GE(mission_result.steps, 1u);
        EXPECT_GE(run.mission_score, 0.0);
        EXPECT_LE(run.mission_score, 100.0);
        EXPECT_TRUE(fs::exists(run.output_map_file)) << map_stem << error_detail;
    }
}

} // namespace

/**
 * @brief scenario_house × house_mission_lower (the offset-150 scenario; runs but maps a solid slab).
 */
TEST_F(Integration, InputsScenarioHouseLower) {
    runScenario(0, 0, "scenario_house", "intg_house_lower");
}

/**
 * @brief scenario_house × house_mission_full (reaches the actual rooms above the pedestal).
 */
TEST_F(Integration, InputsScenarioHouseFull) {
    runScenario(0, 1, "scenario_house", "intg_house_full");
}

/**
 * @brief large_simulation_out × large_mission_out (scenario_big, outdoor start).
 */
TEST_F(Integration, InputsScenarioLargeOut) {
    runScenario(1, 0, "scenario_big", "intg_large_out");
}

/**
 * @brief large_simulation_room × large_mission_room (scenario_big again, but the in-room start —
 *        a distinct simulation group sharing the map).
 */
TEST_F(Integration, InputsScenarioLargeRoom) {
    runScenario(2, 0, "scenario_big", "intg_large_room");
}

/**
 * @brief small_simulation_out × small_mission_out (scenario_small, outdoor start).
 */
TEST_F(Integration, InputsScenarioSmallOut) {
    runScenario(3, 0, "scenario_small", "intg_small_out");
}

/**
 * @brief small_simulation_room × small_mission_room (scenario_small, in-room start).
 */
TEST_F(Integration, InputsScenarioSmallRoom) {
    runScenario(4, 0, "scenario_small", "intg_small_room");
}
