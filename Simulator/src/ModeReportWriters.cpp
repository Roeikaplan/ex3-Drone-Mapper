/**
 * @file ModeReportWriters.cpp
 * @brief Ranking, grouping, and the shared shape of the two aggregate reports.
 */

#include <Simulator/ModeReportWriters.h>

#include <Simulator/UtcTime.h>

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cstddef>
#include <fstream>
#include <system_error>
#include <utility>

namespace simulator {
namespace {

/**
 * @brief A plugin's summed results.
 */
struct Totals {
    double score = 0.0;
    std::size_t steps = 0;
};

/**
 * @brief The per-run signature two plugins must share to count as behaving identically.
 * @note An ordered sequence, not a set: run *i* of one plugin is compared against run *i* of the
 *       other, so two plugins that scored the same values in a different order are correctly
 *       reported as different. The order is the manager's expansion order, which is fixed.
 */
using BehaviourKey = std::vector<std::pair<double, std::size_t>>;

/**
 * @brief Steps a run took.
 * @param result One run's result.
 * @return The mission's step count, or 0 when it recorded no mission at all.
 */
[[nodiscard]] std::size_t stepsOf(const types::SimulationResult& result) {
    return result.mission_results.empty() ? std::size_t{0} : result.mission_results.front().steps;
}

/**
 * @brief Whether a run failed rather than scoring badly.
 * @param result One run's result.
 * @return True when it carries the negative error sentinel.
 */
[[nodiscard]] bool isErrored(const types::SimulationResult& result) {
    return result.mission_score < 0.0;
}

/**
 * @brief Sum a plugin's runs.
 * @param report The plugin's results.
 * @return Total score and total steps across every run.
 * @note Every run counts, sentinels included. A plugin that failed a third of its runs should rank
 *       below one that completed them all, and dropping the `-1` values would hide exactly that.
 */
[[nodiscard]] Totals totalsOf(const types::SimulationManagerReport& report) {
    Totals totals{};
    for (const types::SimulationResult& run : report.runs) {
        totals.score += run.mission_score;
        totals.steps += stepsOf(run);
    }
    return totals;
}

/**
 * @brief Whether a plugin produced nothing usable.
 * @param report The plugin's results.
 * @return True when it has no runs at all, or when every run errored.
 * @note Such a plugin belongs under `errors:` rather than at the bottom of the ranking. It did not
 *       score poorly; it did not function.
 */
[[nodiscard]] bool producedNothingUsable(const types::SimulationManagerReport& report) {
    if (report.runs.empty()) {
        return true;
    }
    return std::all_of(report.runs.begin(), report.runs.end(), isErrored);
}

/**
 * @brief Build a plugin's behavioural signature.
 * @param report The plugin's results.
 * @return One `(score, steps)` pair per run, in expansion order.
 */
[[nodiscard]] BehaviourKey behaviourKeyOf(const types::SimulationManagerReport& report) {
    BehaviourKey key;
    key.reserve(report.runs.size());
    for (const types::SimulationResult& run : report.runs) {
        key.emplace_back(run.mission_score, stepsOf(run));
    }
    return key;
}

/**
 * @brief Split outcomes into those worth ranking and the names of those that are not.
 * @param input The mode's results.
 * @param usable Receives the outcomes that produced something.
 * @param errors Receives the names that did not, load failures first.
 * @note Load failures and total run failures land in the same list because the report makes no
 *       distinction between them - from a reader's point of view the plugin did not work.
 */
void partitionOutcomes(const ModeReportInput& input, std::vector<const PluginOutcome*>& usable,
                       std::vector<std::string>& errors) {
    errors = input.failed_to_load;
    for (const PluginOutcome& outcome : input.outcomes) {
        if (producedNothingUsable(outcome.report)) {
            errors.push_back(outcome.name);
        } else {
            usable.push_back(&outcome);
        }
    }
    std::sort(errors.begin(), errors.end());
}

/**
 * @brief Fill the metadata every mode report carries.
 * @param node Report node to populate.
 * @param input The mode's results.
 * @note `generated_at_utc` comes from the shared clock helper, so a report and the directory holding
 *       it never appear to disagree about when the run happened.
 */
void appendCommonMetadata(YAML::Node& node, const ModeReportInput& input) {
    node["composition_file"] = input.composition_file.string();
    node["generated_at_utc"] = utcIso8601();
}

/**
 * @brief Build the `errors:` sequence.
 * @param errors Names that failed to load or failed to run.
 * @return A YAML sequence, empty when nothing failed.
 * @note Constructed explicitly as a sequence. A default-constructed node that never gets an element
 *       emits as `~` - YAML null - rather than `[]`, and a reader expecting a list would have to
 *       special-case the successful run. The empty case is the common one, so it has to be the
 *       well-formed one.
 */
[[nodiscard]] YAML::Node buildErrors(const std::vector<std::string>& errors) {
    YAML::Node node(YAML::NodeType::Sequence);
    for (const std::string& name : errors) {
        node.push_back(name);
    }
    return node;
}

/**
 * @brief Write a report document, creating the directory if needed.
 * @param root The document to write.
 * @param output_path Directory to write into.
 * @param filename Name of the file within it.
 * @note Silent on failure by design. The runs already happened and their artefacts are already on
 *       disk; failing to summarise them is not worth ending the program over.
 */
void writeDocument(const YAML::Node& root, const std::filesystem::path& output_path,
                   const char* filename) {
    std::error_code ec;
    std::filesystem::create_directories(output_path, ec);

    std::ofstream out(output_path / filename, std::ios::trunc);
    if (out) {
        out << root << '\n';
    }
}

} // namespace

/**
 * @brief Write `comparative_report.yaml`: which mission controls behaved identically.
 * @param input Results and the run's identifying paths.
 * @param output_path Directory to write into.
 * @note Grouping is a linear scan rather than a sort-by-key: the number of plugins in a folder is
 *       small, and preserving first-seen order within a group keeps the output stable without
 *       needing the key itself to be orderable.
 * @note Both the members of a group and the groups themselves are ordered deterministically, so two
 *       runs over the same data produce byte-identical documents. Comparative mode exists to answer
 *       whether two plugins agree; a report that disagreed with itself between runs would be worse
 *       than none.
 */
void writeComparativeReport(const ModeReportInput& input, const std::filesystem::path& output_path) {
    std::vector<const PluginOutcome*> usable;
    std::vector<std::string> errors;
    partitionOutcomes(input, usable, errors);

    struct Group {
        BehaviourKey key;
        std::vector<std::string> names;
        Totals totals;
    };

    std::vector<Group> groups;
    for (const PluginOutcome* outcome : usable) {
        BehaviourKey key = behaviourKeyOf(outcome->report);
        const auto found = std::find_if(groups.begin(), groups.end(),
                                        [&key](const Group& g) { return g.key == key; });
        if (found == groups.end()) {
            groups.push_back(Group{std::move(key), {outcome->name}, totalsOf(outcome->report)});
        } else {
            found->names.push_back(outcome->name);
        }
    }

    for (Group& group : groups) {
        std::sort(group.names.begin(), group.names.end());
    }

    /**
     * @note Larger groups first, as the assignment shows. Ties fall back to the first member's name
     *       purely so the ordering is total - without it, equal-sized groups could appear in either
     *       order between runs.
     */
    std::sort(groups.begin(), groups.end(), [](const Group& lhs, const Group& rhs) {
        if (lhs.names.size() != rhs.names.size()) {
            return lhs.names.size() > rhs.names.size();
        }
        return lhs.names.front() < rhs.names.front();
    });

    YAML::Node report;
    appendCommonMetadata(report, input);
    report["mission_control_folder"] = input.varied_plugin_folder.string();
    report["algorithm"] = input.fixed_plugin_file.filename().string();

    YAML::Node summary(YAML::NodeType::Sequence);
    for (const Group& group : groups) {
        YAML::Node entry;
        YAML::Node names(YAML::NodeType::Sequence);
        for (const std::string& name : group.names) {
            names.push_back(name);
        }
        entry["same_results"] = names;
        entry["total_score"] = group.totals.score;
        entry["total_steps"] = group.totals.steps;
        summary.push_back(entry);
    }
    report["results_summary"] = summary;
    report["errors"] = buildErrors(errors);

    YAML::Node root;
    root["comparative_report"] = report;
    writeDocument(root, output_path, "comparative_report.yaml");
}

/**
 * @brief Write `competitive_report.yaml`: which algorithm scored best.
 * @param input Results and the run's identifying paths.
 * @param output_path Directory to write into.
 * @note The sort is score descending, then steps ascending, then name. The third key does no ranking
 *       work - it exists so two plugins that tied on both real criteria still appear in a fixed
 *       order rather than whichever the sort happened to leave them in.
 */
void writeCompetitiveReport(const ModeReportInput& input, const std::filesystem::path& output_path) {
    std::vector<const PluginOutcome*> usable;
    std::vector<std::string> errors;
    partitionOutcomes(input, usable, errors);

    struct Ranked {
        std::string name;
        Totals totals;
    };

    std::vector<Ranked> ranked;
    ranked.reserve(usable.size());
    for (const PluginOutcome* outcome : usable) {
        ranked.push_back(Ranked{outcome->name, totalsOf(outcome->report)});
    }

    std::sort(ranked.begin(), ranked.end(), [](const Ranked& lhs, const Ranked& rhs) {
        if (lhs.totals.score != rhs.totals.score) {
            return lhs.totals.score > rhs.totals.score;
        }
        if (lhs.totals.steps != rhs.totals.steps) {
            return lhs.totals.steps < rhs.totals.steps;
        }
        return lhs.name < rhs.name;
    });

    YAML::Node report;
    appendCommonMetadata(report, input);
    report["mission_control"] = input.fixed_plugin_file.filename().string();
    report["algorithms_folder"] = input.varied_plugin_folder.string();

    YAML::Node summary(YAML::NodeType::Sequence);
    for (const Ranked& entry : ranked) {
        YAML::Node node;
        node["algorithm"] = entry.name;
        node["total_score"] = entry.totals.score;
        node["total_steps"] = entry.totals.steps;
        summary.push_back(node);
    }
    report["results_summary"] = summary;
    report["errors"] = buildErrors(errors);

    YAML::Node root;
    root["competitive_report"] = report;
    writeDocument(root, output_path, "competitive_report.yaml");
}

} // namespace simulator
