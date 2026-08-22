/**
 * @file SimulationManager.cpp
 * @brief The four-deep expansion, its failure containment, and the report it assembles.
 */

#include <Simulator/SimulationManager.h>

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
 *       in the result, and a resolution request the factory could not honour. Called the moment
 *       `run()` returns so the log is never deferred to the end of the batch.
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
 * @throws std::invalid_argument when @p run_factory is null.
 */
SimulationManager::SimulationManager(std::unique_ptr<ISimulationRunFactory> run_factory,
                                     std::string plugin_label, ErrorLogger& logger)
    : run_factory_(std::move(run_factory)),
      plugin_label_(std::move(plugin_label)),
      logger_(logger) {
    if (!run_factory_) {
        throw std::invalid_argument("SimulationManager requires a run factory.");
    }
}

/**
 * @brief Run every combination the composition describes.
 * @param composition Simulations with their missions, crossed with drones and lidars.
 * @param output_path Directory each run writes its output map into.
 * @return One result per combination, in expansion order, plus report metadata.
 * @note Each combination is wrapped individually. A bad map file throws for every combination of the
 *       affected simulation, which fills the sentinel across that whole group without the loop
 *       needing to special-case it.
 * @note `catch (...)` is present because plugin code runs inside `run()`. An exception escaping a
 *       third-party plugin must not end the batch, and it must not skip the remaining combinations.
 */
types::SimulationManagerReport SimulationManager::run(
    const types::SimulationCompositionData& composition,
    const std::filesystem::path& output_path) {
    std::vector<types::SimulationResult> runs;

    for (const auto& group : composition.simulation_mission_groups) {
        const types::SimulationConfigData& simulation = std::get<0>(group);
        for (const common::types::MissionConfigData& mission : std::get<1>(group)) {
            for (const common::types::DroneConfigData& drone : composition.drone_configs) {
                for (const common::types::LidarConfigData& lidar : composition.lidar_configs) {
                    try {
                        std::unique_ptr<ISimulationRun> run =
                            run_factory_->create(simulation, mission, drone, lidar, output_path);
                        types::SimulationResult result = run->run();
                        logRunOutcome(logger_, plugin_label_, result);
                        runs.push_back(std::move(result));
                    } catch (const std::exception& error) {
                        logger_.log("RUN_FAILED", plugin_label_ + ": run could not be executed: " +
                                                      std::string{error.what()});
                        runs.push_back(makeErrorResult(simulation, mission, error.what()));
                    } catch (...) {
                        logger_.log("RUN_FAILED",
                                    plugin_label_ + ": run failed with a non-standard exception");
                        runs.push_back(
                            makeErrorResult(simulation, mission, "non-standard exception"));
                    }
                }
            }
        }
    }

    types::SimulationManagerReport report{};
    report.composition_file = composition.composition_file;
    report.generated_at_utc = utcIso8601();
    report.metric = "occupied_voxel_iou";
    report.score_range = {0.0, 100.0};
    report.error_score = static_cast<int>(kErrorScore);
    report.runs = std::move(runs);
    return report;
}

} // namespace simulator
