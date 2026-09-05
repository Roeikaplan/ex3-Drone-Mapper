/**
 * @file SimulationManager.cpp
 * @brief The four-deep expansion, per-cell failure containment, and report assembly.
 */

#include <Simulator/SimulationManager.h>

#include <Simulator/PluginRegistry.h>
#include <Simulator/UtcTime.h>

#include <exception>
#include <stdexcept>
#include <tuple>
#include <utility>
#include <vector>

namespace simulator {
namespace {

/**
 * @brief The score given to a combination that could not be run or whose mission failed.
 * @note Negative on purpose, so it can never be confused with a genuine score - the metric's range
 *       is 0 to 100, and a report distinguishes failures from poor results by this sentinel alone.
 */
constexpr double kErrorScore = -1.0;

/**
 * @brief Build a result for a combination whose run could not be created or executed.
 * @param simulation_config Simulation of the failed combination.
 * @param mission_config Mission of the failed combination.
 * @param message Detail from the caught exception.
 * @return A result marked errored and scored with the sentinel.
 * @note A failed combination still produces a result rather than being skipped. The report must
 *       account for every combination the composition described, and a silent gap would look like
 *       the combination was never requested.
 */
[[nodiscard]] types::SimulationResult makeErrorResult(
    const types::SimulationConfigData& simulation_config,
    const common::types::MissionConfigData& mission_config, std::string message) {
    types::SimulationResult result{};
    result.simulation_config = simulation_config;
    result.mission_config = mission_config;
    result.mission_results = {common::types::MissionRunResult{
        common::types::MissionRunStatus::Error, 0,
        {common::types::ErrorRef{"RUN_FACTORY_ERROR", std::move(message)}}}};
    result.mission_score = kErrorScore;
    return result;
}

/**
 * @brief Report everything a completed run failed at, immediately.
 * @param logger Shared error sink.
 * @param plugin_label Name of the plugin the run used.
 * @param result The run's assembled result.
 * @note Covers what a run reports by *status* rather than by throwing: mission-level errors carried
 *       in the result, and a resolution request the factory could not honour.
 */
void logRunOutcome(ErrorLogger& logger, const std::string& plugin_label,
                   const types::SimulationResult& result) {
    for (const common::types::MissionRunResult& mission : result.mission_results) {
        for (const common::types::ErrorRef& error : mission.errors) {
            logger.log(error.code, plugin_label + ": " + error.message);
        }
    }

    if (result.resolution_request_status != types::ResolutionRequestStatus::Accepted) {
        logger.log("RESOLUTION_IGNORED",
                   plugin_label + ": output_mapping_resolution_factor " +
                       std::to_string(result.mission_config.output_mapping_resolution_factor) +
                       " not honoured; used the default output resolution.");
    }
}

} // namespace

/**
 * @brief Construct over a bound factory.
 * @param run_factory Factory already bound to one plugin pair.
 * @param plugin_label Name of the varied plugin.
 * @param logger Sink for run failures.
 * @param plugins The plugin pair this manager's runs borrow a use of; empty when unmanaged.
 * @throws std::invalid_argument when @p run_factory is null.
 */
SimulationManager::SimulationManager(std::unique_ptr<ISimulationRunFactory> run_factory,
                                     std::string plugin_label, ErrorLogger& logger,
                                     PluginUse plugins)
    : run_factory_(std::move(run_factory)),
      plugins_(plugins),
      plugin_label_(std::move(plugin_label)),
      logger_(logger) {
    if (!run_factory_) {
        throw std::invalid_argument("SimulationManager requires a run factory.");
    }
}

/**
 * @brief Append this plugin's runs to a table.
 * @param composition Simulations with their missions, crossed with drones and lidars.
 * @param output_path Directory each run writes its output map into.
 * @param table Table to append to.
 * @note Pointers rather than copies: the cells refer into the composition, which the caller keeps
 *       alive. Copying four configs per cell would also break `ConfigIdentityIndex`, which resolves
 *       a config's source filename by its address.
 */
void SimulationManager::enumerate(const types::SimulationCompositionData& composition,
                                  const std::filesystem::path& output_path,
                                  SimulationTaskTable& table) {
    const std::size_t begin = table.size();

    for (const auto& group : composition.simulation_mission_groups) {
        const types::SimulationConfigData& simulation = std::get<0>(group);
        for (const common::types::MissionConfigData& mission : std::get<1>(group)) {
            for (const common::types::DroneConfigData& drone : composition.drone_configs) {
                for (const common::types::LidarConfigData& lidar : composition.lidar_configs) {
                    table.append(RunCell{run_factory_.get(), plugins_, &simulation, &mission,
                                         &drone, &lidar, output_path});
                }
            }
        }
    }

    table.closePluginRange(begin);
}

/**
 * @brief Execute one cell and record its outcome.
 * @param table Table holding the cell and its result slot.
 * @param index Which cell to run.
 * @note A bad map file throws for every combination of the affected simulation, which fills the
 *       sentinel across that whole group without the caller needing to special-case it.
 * @note `catch (...)` is present because plugin code runs inside `run()`. An exception escaping a
 *       third-party plugin must not end the batch - and once execution is concurrent, must not reach
 *       a worker's boundary at all.
 * @note **The guard is declared before the `try` on purpose.** Members and locals are destroyed in
 *       reverse order of declaration, so this ordering makes the run - and with it both plugin
 *       instances - die first, and the use is given back only afterwards. If this cell happens to
 *       hold the last outstanding use of a library, the guard's destructor is what unmaps it, and
 *       unmapping code that a live object still points into is the one way this design can crash.
 */
void SimulationManager::runCell(SimulationTaskTable& table, std::size_t index) {
    const RunCell& cell = table.cell(index);
    const PluginUseGuard use{cell.plugins};

    try {
        std::unique_ptr<ISimulationRun> run = cell.factory->create(
            *cell.simulation, *cell.mission, *cell.drone, *cell.lidar, cell.output_path);
        types::SimulationResult result = run->run();
        logRunOutcome(logger_, plugin_label_, result);
        table.result(index) = std::move(result);
    } catch (const PluginUnavailable& error) {
        /**
         * @note Deliberately not logged. The registry already reported this library's load failure
         *       once, when it happened; every run of that plugin fails for that one reason, and
         *       repeating it per run would bury the real diagnostic under its own consequences. The
         *       result still carries the reason, and the plugin is still named under `errors:`.
         */
        table.result(index) = makeErrorResult(*cell.simulation, *cell.mission, error.what());
    } catch (const std::exception& error) {
        logger_.log("RUN_FAILED",
                    plugin_label_ + ": run could not be executed: " + std::string{error.what()});
        table.result(index) = makeErrorResult(*cell.simulation, *cell.mission, error.what());
    } catch (...) {
        logger_.log("RUN_FAILED", plugin_label_ + ": run failed with a non-standard exception");
        table.result(index) =
            makeErrorResult(*cell.simulation, *cell.mission, "non-standard exception");
    }
}

/**
 * @brief Turn this plugin's results into its report.
 * @param composition The composition the runs came from, for the report's metadata.
 * @param results One plugin's results, in expansion order.
 * @return The assembled report.
 */
types::SimulationManagerReport SimulationManager::assemble(
    const types::SimulationCompositionData& composition,
    std::vector<types::SimulationResult> results) const {
    types::SimulationManagerReport report{};
    report.composition_file = composition.composition_file;
    report.generated_at_utc = utcIso8601();
    report.metric = "occupied_voxel_iou";
    report.score_range = {0.0, 100.0};
    report.error_score = static_cast<int>(kErrorScore);
    report.runs = std::move(results);
    return report;
}

/**
 * @brief Run every combination the composition describes.
 * @param composition Simulations with their missions, crossed with drones and lidars.
 * @param output_path Directory each run writes its output map into.
 * @return One result per combination, in expansion order, plus report metadata.
 * @note The three steps composed at single-plugin scope, on the calling thread. This is what
 *       `ISimulation` promises; a caller wanting a different schedule uses the steps directly.
 */
types::SimulationManagerReport SimulationManager::run(
    const types::SimulationCompositionData& composition,
    const std::filesystem::path& output_path) {
    SimulationTaskTable table;
    enumerate(composition, output_path, table);
    table.seal();

    /**
     * @note This path reserves its own plugin uses, because nothing else has. The orchestrator does
     *       it for the runs it schedules; a caller using `ISimulation::run` directly gets the same
     *       accounting here, so a cell can never give back a use that was never taken.
     */
    if (plugins_.registry != nullptr) {
        if (plugins_.mission_control != nullptr) {
            plugins_.registry->reserve(*plugins_.mission_control, table.size());
        }
        if (plugins_.algorithm != nullptr) {
            plugins_.registry->reserve(*plugins_.algorithm, table.size());
        }
    }

    InlineExecutor executor;
    executor.forEach(table.size(), [this, &table](std::size_t index) { runCell(table, index); });

    return assemble(composition, table.resultsForPlugin(0));
}

} // namespace simulator
