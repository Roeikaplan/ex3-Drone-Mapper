/**
 * @file SimulationOutputWriter.cpp
 * @brief Building the `score_report:` document and writing it out.
 * @note Ported from Assignment 2's writer with its value-based labelling half removed. Ex2 supported
 *       two composition layouts and therefore two ways of naming things; Ex3 ships only the
 *       file-reference layout, which retires the config-grouping machinery entirely. A small flat
 *       fallback survives for the case where the paths do not describe the runs - see
 *       `buildFlatRuns`.
 */

#include <Simulator/SimulationOutputWriter.h>

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cstddef>
#include <fstream>
#include <limits>
#include <string>
#include <system_error>
#include <tuple>
#include <vector>

namespace simulator {
namespace {

using common::cm;

/**
 * @brief The mission outcome of a run.
 * @param result One run's result.
 * @return Pointer to its single `MissionRunResult`, or nullptr when the list is empty.
 * @note Each run carries exactly one mission, but the type is a vector, so the empty case has to be
 *       handled rather than indexed into.
 */
[[nodiscard]] const common::types::MissionRunResult* firstMission(
    const types::SimulationResult& result) {
    return result.mission_results.empty() ? nullptr : &result.mission_results.front();
}

/**
 * @brief Whether a run failed rather than scoring badly.
 * @param result One run's result.
 * @return True when the score is the negative error sentinel.
 * @note The distinction matters everywhere in this file: errored runs are reported but excluded from
 *       every summary statistic.
 */
[[nodiscard]] bool isErrored(const types::SimulationResult& result) {
    return result.mission_score < 0.0;
}

/**
 * @brief YAML text for a mission status.
 * @param status The status to name.
 * @return A lowercase token.
 */
[[nodiscard]] std::string missionStatusString(common::types::MissionRunStatus status) {
    switch (status) {
    case common::types::MissionRunStatus::Completed:
        return "completed";
    case common::types::MissionRunStatus::MaxSteps:
        return "max_steps";
    case common::types::MissionRunStatus::Error:
        return "error";
    }
    return "unknown";
}

/**
 * @brief YAML text for a resolution request outcome.
 * @param status The status to name.
 * @return An uppercase token.
 */
[[nodiscard]] std::string resolutionStatusString(types::ResolutionRequestStatus status) {
    switch (status) {
    case types::ResolutionRequestStatus::Accepted:
        return "ACCEPTED";
    case types::ResolutionRequestStatus::Ignored:
        return "IGNORED";
    case types::ResolutionRequestStatus::IgnoredTooSmall:
        return "IGNORED TOO SMALL";
    }
    return "IGNORED";
}

/**
 * @brief Label one config level, falling back to a positional name.
 * @param paths Recorded source paths for this level.
 * @param index Position within that level.
 * @param prefix Word used to build a positional name when no path was recorded.
 * @return The path as written in the composition, or `<prefix><index>`.
 * @note The path is emitted **as written**, not as resolved, because that is what the reader typed
 *       into the composition and what the assignment's sample shows.
 */
[[nodiscard]] std::string label(const std::vector<std::filesystem::path>& paths, std::size_t index,
                                const char* prefix) {
    if (index < paths.size() && !paths[index].empty()) {
        return paths[index].string();
    }
    return prefix + std::to_string(index);
}

/**
 * @brief Derived statistics over the run list.
 * @param report The aggregate report.
 * @return A map with the run counts plus average, minimum, and maximum score.
 * @note Averages, minima, and maxima cover **scored runs only**. Folding the -1 sentinel in would
 *       produce an average below the `score_range` the same document declares, which is worse than
 *       useless - it looks like a valid number.
 */
[[nodiscard]] YAML::Node buildSummary(const types::SimulationManagerReport& report) {
    std::size_t error_runs = 0;
    std::size_t scored_runs = 0;
    double sum = 0.0;
    double min_score = std::numeric_limits<double>::max();
    double max_score = std::numeric_limits<double>::lowest();

    for (const types::SimulationResult& run : report.runs) {
        if (isErrored(run)) {
            ++error_runs;
            continue;
        }
        ++scored_runs;
        sum += run.mission_score;
        min_score = std::min(min_score, run.mission_score);
        max_score = std::max(max_score, run.mission_score);
    }

    YAML::Node summary;
    summary["total_runs"] = report.runs.size();
    summary["scored_runs"] = scored_runs;
    summary["error_runs"] = error_runs;
    summary["average_score"] = scored_runs > 0 ? sum / static_cast<double>(scored_runs) : 0.0;
    summary["min_score"] = scored_runs > 0 ? min_score : 0.0;
    summary["max_score"] = scored_runs > 0 ? max_score : 0.0;
    return summary;
}

/**
 * @brief Pick the run whose geometry represents its mission.
 * @param runs The runs belonging to one mission; all requested the same resolution.
 * @return The first scored run, else the first run, else nullptr.
 * @note A scored run is preferred deliberately. An errored result carries a default
 *       `output_map_config`, so taking the first run blindly would report `resolution_cm: 0` for any
 *       mission whose first combination happened to fail.
 */
[[nodiscard]] const types::SimulationResult* representativeRun(
    const std::vector<const types::SimulationResult*>& runs) {
    for (const types::SimulationResult* run : runs) {
        if (!isErrored(*run)) {
            return run;
        }
    }
    return runs.empty() ? nullptr : runs.front();
}

/**
 * @brief Append a run's outcome fields to its node.
 * @param node The run node; its identity fields are added by the caller first.
 * @param run The run's result.
 * @note An errored run is reported with its code rather than omitted. A report that silently drops
 *       failures would disagree with the composition about how many runs were requested.
 */
void appendRunOutcome(YAML::Node& node, const types::SimulationResult& run) {
    const common::types::MissionRunResult* mission = firstMission(run);
    node["status"] =
        mission != nullptr ? missionStatusString(mission->status) : std::string{"unknown"};
    node["steps"] = mission != nullptr ? mission->steps : std::size_t{0};
    node["score"] = run.mission_score;
    node["output_map"] = run.output_map_file.filename().string();

    if (isErrored(run) && mission != nullptr && !mission->errors.empty()) {
        YAML::Node error_ref;
        error_ref["code"] = mission->errors.front().code;
        node["error_ref"] = error_ref;
    }
}

/**
 * @brief Add the mission-level resolution fields.
 * @param mission_node The mission node to populate.
 * @param representative The run those fields are taken from; nullptr omits them.
 */
void appendMissionResolution(YAML::Node& mission_node,
                             const types::SimulationResult* representative) {
    if (representative != nullptr) {
        mission_node["resolution_cm"] =
            representative->output_map_config.resolution.force_numerical_value_in(cm);
        mission_node["resolution_request_status"] =
            resolutionStatusString(representative->resolution_request_status);
    }
}

/**
 * @brief Whether the recorded paths describe exactly the runs the report holds.
 * @param paths Source config paths.
 * @param report The aggregate report.
 * @return True when Σ(missions × drones × lidars) equals the run count.
 * @note This is the guard on the positional labelling below. It catches a gross mismatch - paths
 *       that were never recorded, or a composition that changed underneath - but it cannot catch a
 *       *reordering*, because a permutation has the same count. The expansion order in
 *       `SimulationManager::run` is a contract, not something this can verify.
 */
[[nodiscard]] bool describesRuns(const CompositionPaths& paths,
                                 const types::SimulationManagerReport& report) {
    if (paths.simulation_paths.empty() || paths.drone_paths.empty() || paths.lidar_paths.empty() ||
        paths.mission_paths.size() != paths.simulation_paths.size()) {
        return false;
    }

    std::size_t expected = 0;
    for (const std::vector<std::filesystem::path>& missions : paths.mission_paths) {
        expected += missions.size() * paths.drone_paths.size() * paths.lidar_paths.size();
    }
    return expected == report.runs.size();
}

/**
 * @brief Build the nested `simulations` node, labelling every level by its source file.
 * @param report The aggregate report, in the manager's expansion order.
 * @param paths Source config paths, already checked by `describesRuns`.
 * @return A sequence of simulation nodes, each holding missions, each holding runs.
 * @note Walks simulation, then mission, then drone, then lidar - the identical nesting
 *       `SimulationManager::run` uses - consuming `report.runs` by index. Change one and the other
 *       must change with it.
 */
[[nodiscard]] YAML::Node buildNestedSimulations(const types::SimulationManagerReport& report,
                                                const CompositionPaths& paths) {
    YAML::Node simulations;
    std::size_t run_index = 0;

    for (std::size_t g = 0; g < paths.simulation_paths.size(); ++g) {
        YAML::Node simulation_node;
        simulation_node["simulation_config"] = label(paths.simulation_paths, g, "simulation");

        YAML::Node missions;
        for (std::size_t m = 0; m < paths.mission_paths[g].size(); ++m) {
            YAML::Node mission_node;
            mission_node["mission_config"] = label(paths.mission_paths[g], m, "mission");

            YAML::Node runs;
            std::vector<const types::SimulationResult*> mission_runs;
            for (std::size_t d = 0; d < paths.drone_paths.size(); ++d) {
                for (std::size_t l = 0; l < paths.lidar_paths.size(); ++l) {
                    const types::SimulationResult& run = report.runs[run_index++];
                    mission_runs.push_back(&run);

                    YAML::Node run_node;
                    run_node["drone_config"] = label(paths.drone_paths, d, "drone");
                    run_node["lidar_config"] = label(paths.lidar_paths, l, "lidar");
                    appendRunOutcome(run_node, run);
                    runs.push_back(run_node);
                }
            }

            appendMissionResolution(mission_node, representativeRun(mission_runs));
            mission_node["runs"] = runs;
            missions.push_back(mission_node);
        }

        simulation_node["missions"] = missions;
        simulations.push_back(simulation_node);
    }

    return simulations;
}

/**
 * @brief Build a flat run list, used when the paths do not describe the runs.
 * @param report The aggregate report.
 * @return A sequence of run nodes identified by their output-map filename.
 * @note A degraded but complete report. Every run still appears with its outcome, identified by the
 *       output map it produced - which already names every dimension of the run. Emitting confidently
 *       wrong `simulation_config` labels would be far worse than emitting none.
 */
[[nodiscard]] YAML::Node buildFlatRuns(const types::SimulationManagerReport& report) {
    YAML::Node runs;
    for (const types::SimulationResult& run : report.runs) {
        YAML::Node run_node;
        appendRunOutcome(run_node, run);
        runs.push_back(run_node);
    }
    return runs;
}

} // namespace

/**
 * @brief Write one plugin's report to `<plugin_label>__simulation_output.yaml`.
 * @param report The aggregate produced by `SimulationManager::run`.
 * @param output_path Directory to write into; created if missing.
 * @param plugin_label Name of the plugin these results belong to.
 * @param paths Source config paths, used to label each run.
 * @note Nothing here throws. The runs already happened and their maps are already on disk; failing
 *       to record them is worth reporting but not worth ending the program over, and `main` has no
 *       better recovery available than continuing to the next plugin.
 */
void writeSimulationOutput(const types::SimulationManagerReport& report,
                           const std::filesystem::path& output_path,
                           const std::string& plugin_label, const CompositionPaths& paths) {
    YAML::Node report_node;
    report_node["plugin"] = plugin_label;
    report_node["composition_file"] = report.composition_file.string();
    report_node["generated_at_utc"] = report.generated_at_utc;
    report_node["metric"] = report.metric;

    YAML::Node score_range;
    score_range["min"] = std::get<0>(report.score_range);
    score_range["max"] = std::get<1>(report.score_range);
    report_node["score_range"] = score_range;
    report_node["error_score"] = report.error_score;
    report_node["summary"] = buildSummary(report);

    if (describesRuns(paths, report)) {
        report_node["simulations"] = buildNestedSimulations(report, paths);
    } else {
        report_node["runs"] = buildFlatRuns(report);
    }

    YAML::Node root;
    root["score_report"] = report_node;

    std::error_code ec;
    std::filesystem::create_directories(output_path, ec);

    std::ofstream out(output_path / (plugin_label + "__simulation_output.yaml"), std::ios::trunc);
    if (out) {
        out << root << '\n';
    }
}

} // namespace simulator
