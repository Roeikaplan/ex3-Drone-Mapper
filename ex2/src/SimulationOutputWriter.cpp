#include <drone_mapper/SimulationOutputWriter.h>

#include <drone_mapper/CompositionPaths.h>
#include <drone_mapper/Units.h>

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cstddef>
#include <fstream>
#include <limits>
#include <sstream>
#include <string>
#include <system_error>
#include <tuple>
#include <vector>

namespace drone_mapper {
namespace {

/**
 * @brief The mission outcome of a run (each run carries exactly one), or nullptr if absent.
 * @param result One simulation run's result.
 * @return Pointer to the first `MissionRunResult`, or nullptr when the list is empty.
 */
[[nodiscard]] const types::MissionRunResult* firstMission(const types::SimulationResult& result) {
    return result.mission_results.empty() ? nullptr : &result.mission_results.front();
}

/**
 * @brief Whether a run is an error run (excluded from the scored summary statistics).
 * @param result One simulation run's result.
 * @return True iff its score is the negative error sentinel.
 */
[[nodiscard]] bool isErrored(const types::SimulationResult& result) {
    return result.mission_score < 0.0;
}

/**
 * @brief YAML text for a mission run status.
 * @param status Mission status enum.
 * @return "completed" / "max_steps" / "error" (lowercase, matching the PDF sample).
 */
[[nodiscard]] std::string missionStatusString(types::MissionRunStatus status) {
    switch (status) {
    case types::MissionRunStatus::Completed:
        return "completed";
    case types::MissionRunStatus::MaxSteps:
        return "max_steps";
    case types::MissionRunStatus::Error:
        return "error";
    }
    return "unknown";
}

/**
 * @brief YAML text for a resolution request status.
 * @param status Resolution request status enum.
 * @return "ACCEPTED" / "IGNORED" / "IGNORED TOO SMALL".
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

/// One mission's runs within a simulation group (preserves first-seen order).
struct MissionGroup {
    std::string key;
    const types::MissionConfigData* config = nullptr;
    std::vector<const types::SimulationResult*> runs;
};

/// One simulation's missions (preserves first-seen order).
struct SimGroup {
    std::string key;
    const types::SimulationConfigData* config = nullptr;
    std::vector<MissionGroup> missions;
};

/**
 * @brief Stable grouping key for a simulation config (its distinguishing geometry).
 * @param sim Simulation config.
 * @return A string uniquely identifying the simulation among the composition's simulations.
 */
[[nodiscard]] std::string simKey(const types::SimulationConfigData& sim) {
    std::ostringstream os;
    os << sim.map_filename.string() << '|' << sim.map_resolution.force_numerical_value_in(cm) << '|'
       << sim.map_offset.x.force_numerical_value_in(cm) << ','
       << sim.map_offset.y.force_numerical_value_in(cm) << ','
       << sim.map_offset.z.force_numerical_value_in(cm);
    return os.str();
}

/**
 * @brief Stable grouping key for a mission config (its distinguishing parameters).
 * @param mission Mission config.
 * @return A string uniquely identifying the mission within a simulation.
 */
[[nodiscard]] std::string missionKey(const types::MissionConfigData& mission) {
    std::ostringstream os;
    const types::MappingBounds& b = mission.mission_bounds;
    os << mission.max_steps << '|' << mission.output_mapping_resolution_factor << '|'
       << mission.gps_resolution.force_numerical_value_in(cm) << '|'
       << b.min_x.force_numerical_value_in(cm) << ',' << b.max_x.force_numerical_value_in(cm) << ','
       << b.min_y.force_numerical_value_in(cm) << ',' << b.max_y.force_numerical_value_in(cm) << ','
       << b.min_height.force_numerical_value_in(cm) << ','
       << b.max_height.force_numerical_value_in(cm);
    return os.str();
}

/**
 * @brief Re-group the flat run list into nested simulation → mission groups, keeping input order.
 * @param report The aggregate report.
 * @return One `SimGroup` per distinct simulation, each holding its distinct missions and their runs.
 */
[[nodiscard]] std::vector<SimGroup> groupRuns(const types::SimulationManagerReport& report) {
    std::vector<SimGroup> groups;
    for (const types::SimulationResult& run : report.runs) {
        const std::string s_key = simKey(run.simulation_config);
        SimGroup* sim_group = nullptr;
        for (SimGroup& g : groups) {
            if (g.key == s_key) {
                sim_group = &g;
                break;
            }
        }
        if (sim_group == nullptr) {
            groups.push_back(SimGroup{s_key, &run.simulation_config, {}});
            sim_group = &groups.back();
        }

        const std::string m_key = missionKey(run.mission_config);
        MissionGroup* mission_group = nullptr;
        for (MissionGroup& mg : sim_group->missions) {
            if (mg.key == m_key) {
                mission_group = &mg;
                break;
            }
        }
        if (mission_group == nullptr) {
            sim_group->missions.push_back(MissionGroup{m_key, &run.mission_config, {}});
            mission_group = &sim_group->missions.back();
        }
        mission_group->runs.push_back(&run);
    }
    return groups;
}

/**
 * @brief Build the derived `summary` node from the run list.
 * @param report The aggregate report.
 * @return A YAML map with total/scored/error counts and average/min/max over scored runs only.
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
 * @brief Pick the run whose output geometry/resolution status represents its mission.
 * @param runs The mission's runs (all share the same requested resolution).
 * @return The first successfully-scored run, else the first run, else nullptr when empty.
 * @note Error results carry a default `output_map_config` (resolution 0) and a default Ignored
 *       status, so a scored run is preferred as the source of the mission-level resolution fields.
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
 * @brief Append a run's outcome fields (status, steps, score, and error_ref on error) to a node.
 * @param node The run node to populate (its identity fields are added by the caller first).
 * @param run One simulation run's result.
 */
void appendRunOutcome(YAML::Node& node, const types::SimulationResult& run) {
    const types::MissionRunResult* mission = firstMission(run);
    node["status"] = mission != nullptr ? missionStatusString(mission->status) : std::string("unknown");
    node["steps"] = mission != nullptr ? mission->steps : std::size_t{0};
    node["score"] = run.mission_score;
    if (isErrored(run) && mission != nullptr && !mission->errors.empty()) {
        YAML::Node error_ref;
        error_ref["code"] = mission->errors.front().code;
        node["error_ref"] = error_ref;
    }
}

/**
 * @brief Build a value-based run node: identified by its unique output-map filename.
 * @param run One simulation run's result.
 * @return A YAML map with `output_map` plus the outcome fields.
 * @note Used only for the inline composition layout, where no config file paths exist.
 */
[[nodiscard]] YAML::Node buildValueRunNode(const types::SimulationResult& run) {
    YAML::Node node;
    node["output_map"] = run.output_map_file.filename().string();
    appendRunOutcome(node, run);
    return node;
}

/**
 * @brief Add the mission-level resolution fields from a representative run.
 * @param mission_node The mission node to populate.
 * @param rep The representative run (nullptr → fields omitted).
 */
void appendMissionResolution(YAML::Node& mission_node, const types::SimulationResult* rep) {
    if (rep != nullptr) {
        mission_node["resolution_cm"] = rep->output_map_config.resolution.force_numerical_value_in(cm);
        mission_node["resolution_request_status"] = resolutionStatusString(rep->resolution_request_status);
    }
}

/**
 * @brief The value-based `simulations` node: labels each level by config VALUES (inline layout).
 * @param report The aggregate report.
 * @return A YAML sequence of simulation nodes (`map_filename` / mission params / `output_map`).
 */
[[nodiscard]] YAML::Node buildValueSimulations(const types::SimulationManagerReport& report) {
    YAML::Node simulations;
    for (const SimGroup& sim_group : groupRuns(report)) {
        YAML::Node sim_node;
        sim_node["map_filename"] = sim_group.config->map_filename.string();
        sim_node["map_resolution_cm"] = sim_group.config->map_resolution.force_numerical_value_in(cm);
        YAML::Node offset;
        offset["x_cm"] = sim_group.config->map_offset.x.force_numerical_value_in(cm);
        offset["y_cm"] = sim_group.config->map_offset.y.force_numerical_value_in(cm);
        offset["height_cm"] = sim_group.config->map_offset.z.force_numerical_value_in(cm);
        sim_node["map_offset"] = offset;

        YAML::Node missions;
        for (const MissionGroup& mission_group : sim_group.missions) {
            YAML::Node mission_node;
            mission_node["max_steps"] = mission_group.config->max_steps;
            mission_node["output_mapping_resolution_factor"] =
                mission_group.config->output_mapping_resolution_factor;
            appendMissionResolution(mission_node, representativeRun(mission_group.runs));
            YAML::Node runs;
            for (const types::SimulationResult* run : mission_group.runs) {
                runs.push_back(buildValueRunNode(*run));
            }
            mission_node["runs"] = runs;
            missions.push_back(mission_node);
        }
        sim_node["missions"] = missions;
        simulations.push_back(sim_node);
    }
    return simulations;
}

/**
 * @brief Whether the composition paths fully describe every run, enabling path-based labelling.
 * @param paths Source config paths (parallel to the composition).
 * @param report The aggregate report.
 * @return True iff every simulation/mission/drone/lidar path is present and the implied run count
 *         (Σ missions × drones × lidars) equals the number of runs — i.e. the mandated file-reference
 *         layout. Any inline entry (empty path) or mismatch falls back to value-based labelling.
 */
[[nodiscard]] bool canUsePaths(const CompositionPaths& paths,
                               const types::SimulationManagerReport& report) {
    if (paths.simulation_paths.empty() || paths.drone_paths.empty() || paths.lidar_paths.empty() ||
        paths.mission_paths.size() != paths.simulation_paths.size()) {
        return false;
    }
    const auto nonEmpty = [](const std::filesystem::path& p) { return !p.empty(); };
    if (!std::all_of(paths.simulation_paths.begin(), paths.simulation_paths.end(), nonEmpty) ||
        !std::all_of(paths.drone_paths.begin(), paths.drone_paths.end(), nonEmpty) ||
        !std::all_of(paths.lidar_paths.begin(), paths.lidar_paths.end(), nonEmpty)) {
        return false;
    }
    std::size_t expected = 0;
    for (const std::vector<std::filesystem::path>& mission_list : paths.mission_paths) {
        if (!std::all_of(mission_list.begin(), mission_list.end(), nonEmpty)) {
            return false;
        }
        expected += mission_list.size() * paths.drone_paths.size() * paths.lidar_paths.size();
    }
    return expected == report.runs.size();
}

/**
 * @brief The path-based `simulations` node: labels each level by its source config file path.
 * @param report The aggregate report (runs in `SimulationManager` expansion order).
 * @param paths Source config paths (validated by `canUsePaths`).
 * @return A YAML sequence matching the PDF sample (`simulation_config` / `mission_config` /
 *         per-run `drone_config` + `lidar_config`).
 * @note Walks the runs in the same nested order the manager produced them
 *       (simulation → mission → drone → lidar), so each run aligns with its config paths by position.
 */
[[nodiscard]] YAML::Node buildPathSimulations(const types::SimulationManagerReport& report,
                                              const CompositionPaths& paths) {
    YAML::Node simulations;
    std::size_t run_index = 0;
    for (std::size_t g = 0; g < paths.simulation_paths.size(); ++g) {
        YAML::Node sim_node;
        sim_node["simulation_config"] = paths.simulation_paths[g].string();

        YAML::Node missions;
        for (const std::filesystem::path& mission_path : paths.mission_paths[g]) {
            YAML::Node mission_node;
            mission_node["mission_config"] = mission_path.string();

            const std::size_t block_start = run_index;
            YAML::Node runs;
            for (const std::filesystem::path& drone_path : paths.drone_paths) {
                for (const std::filesystem::path& lidar_path : paths.lidar_paths) {
                    const types::SimulationResult& run = report.runs[run_index++];
                    YAML::Node run_node;
                    run_node["drone_config"] = drone_path.string();
                    run_node["lidar_config"] = lidar_path.string();
                    appendRunOutcome(run_node, run);
                    runs.push_back(run_node);
                }
            }
            // Mission-level resolution from a representative run of this mission's block.
            const types::SimulationResult* rep = nullptr;
            for (std::size_t i = block_start; i < run_index; ++i) {
                if (!isErrored(report.runs[i])) {
                    rep = &report.runs[i];
                    break;
                }
            }
            if (rep == nullptr && run_index > block_start) {
                rep = &report.runs[block_start];
            }
            appendMissionResolution(mission_node, rep);
            mission_node["runs"] = runs;
            missions.push_back(mission_node);
        }
        sim_node["missions"] = missions;
        simulations.push_back(sim_node);
    }
    return simulations;
}

} // namespace

void writeSimulationOutput(const types::SimulationManagerReport& report,
                           const std::filesystem::path& output_path,
                           const std::filesystem::path& composition_file,
                           const CompositionPaths& paths) {
    // Everything is nested under a single `score_report:` key (the mandated top-level shape).
    YAML::Node report_node;
    report_node["composition_file"] = composition_file.string();
    report_node["generated_at_utc"] = report.generated_at_utc;
    report_node["metric"] = report.metric;

    YAML::Node score_range;
    score_range["min"] = std::get<0>(report.score_range);
    score_range["max"] = std::get<1>(report.score_range);
    report_node["score_range"] = score_range;
    report_node["error_score"] = report.error_score;
    report_node["summary"] = buildSummary(report);

    // Prefer the mandated path-based identity (simulation_config/mission_config/drone_config/
    // lidar_config) when the composition supplied every source path (the file-reference layout);
    // otherwise fall back to value-based labels (the inline layout has no config paths).
    report_node["simulations"] =
        canUsePaths(paths, report) ? buildPathSimulations(report, paths) : buildValueSimulations(report);

    YAML::Node root;
    root["score_report"] = report_node;

    std::error_code ec;
    std::filesystem::create_directories(output_path, ec);
    std::ofstream out(output_path / "simulation_output.yaml", std::ios::trunc);
    out << root << '\n';
}

} // namespace drone_mapper