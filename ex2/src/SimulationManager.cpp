#include <drone_mapper/SimulationManager.h>

#include <ctime>
#include <exception>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace drone_mapper {
namespace {

/**
 * @brief Current wall-clock time as an ISO-8601 UTC string (e.g. "2026-07-03T13:07:00Z").
 * @return The formatted timestamp for the report's `generated_at_utc` field.
 */
[[nodiscard]] std::string nowUtcIso8601() {
    const std::time_t now = std::time(nullptr);
    std::tm tm_utc{};
    ::gmtime_r(&now, &tm_utc);
    char buffer[32] = {};
    std::strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%SZ", &tm_utc);
    return std::string(buffer);
}

/**
 * @brief Build a score -1 result for a combination whose run could not be created or executed.
 * @param simulation Simulation config of the failed combination.
 * @param mission Mission config of the failed combination.
 * @param message Failure detail (from the caught exception).
 * @return A `SimulationResult` marked errored so the report/writer can score it -1 and continue.
 */
[[nodiscard]] types::SimulationResult makeErrorResult(const types::SimulationConfigData& simulation,
                                                      const types::MissionConfigData& mission,
                                                      std::string message) {
    types::SimulationResult result{};
    result.simulation_config = simulation;
    result.mission_config = mission;
    result.mission_results = {types::MissionRunResult{
        types::MissionRunStatus::Error, 0,
        {types::ErrorRef{"RUN_FACTORY_ERROR", std::move(message)}}}};
    result.mission_score = -1.0;
    return result;
}

/**
 * @brief Immediately log a completed run's errors and any ignored resolution request.
 * @param logger Shared error sink.
 * @param result The run's assembled result.
 * @note The run loop already logs *thrown* failures (RUN_FAILED); this covers errors a run reports by
 *       *status* — mission-level errors (e.g. DRONE_STEP_ERROR) captured in the result and a
 *       resolution request the factory could not honour. Called the moment `run()` returns so the
 *       error log is never deferred to the end of the batch.
 */
void logRunOutcome(ErrorLogger& logger, const types::SimulationResult& result) {
    for (const types::MissionRunResult& mission : result.mission_results) {
        for (const types::ErrorRef& err : mission.errors) {
            logger.log(err.code, err.message);
        }
    }
    if (result.resolution_request_status != types::ResolutionRequestStatus::Accepted) {
        logger.log("RESOLUTION_IGNORED",
                   "output_mapping_resolution_factor " +
                       std::to_string(result.mission_config.output_mapping_resolution_factor) +
                       " not honoured; used the default output resolution.");
    }
}

} // namespace

SimulationManager::SimulationManager(std::unique_ptr<ISimulationRunFactory> run_factory,
                                     ErrorLogger& logger)
    : run_factory_(std::move(run_factory)), logger_(logger) {
    if (!run_factory_) {
        throw std::invalid_argument("SimulationManager requires a run factory.");
    }
}

types::SimulationManagerReport SimulationManager::run(const types::SimulationCompositionData& composition,
                                                      const std::filesystem::path& output_path) {
    std::vector<types::SimulationResult> runs;

    for (const auto& [simulation, missions] : composition.simulation_mission_groups) {
        for (const types::MissionConfigData& mission : missions) {
            for (const types::DroneConfigData& drone : composition.drones) {
                for (const types::LidarConfigData& lidar : composition.lidars) {
                    // Group-level safety: a bad map file (or any wiring/run failure) must not crash
                    // the batch. We log immediately and score this combination -1, then continue.
                    // Because a bad map throws for every combination of that simulation, this fills
                    // -1 for the whole group without special-casing.
                    try {
                        std::unique_ptr<ISimulationRun> run =
                            run_factory_->create(simulation, mission, drone, lidar, output_path);
                        types::SimulationResult result = run->run();
                        // Surface status-reported errors (mission errors, ignored resolution) to the
                        // error log immediately, before moving on to the next combination.
                        logRunOutcome(logger_, result);
                        runs.push_back(std::move(result));
                    } catch (const std::exception& e) {
                        logger_.log("RUN_FAILED", std::string("run could not be executed: ") + e.what());
                        runs.push_back(makeErrorResult(simulation, mission, e.what()));
                    }
                }
            }
        }
    }

    types::SimulationManagerReport report{};
    report.generated_at_utc = nowUtcIso8601();
    report.metric = "occupied_voxel_iou";
    report.score_range = {0.0, 100.0};
    report.error_score = -1;
    report.runs = std::move(runs);
    return report;
}

} // namespace drone_mapper