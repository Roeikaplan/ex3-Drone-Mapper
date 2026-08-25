/**
 * @file ModeReportWriters.h
 * @brief The two aggregate reports that answer what each run mode was asked.
 * @note yaml-cpp is confined to the implementation, so this header stays dependency-free.
 */

#pragma once

#include <Simulator/SimulationTypes.h>

#include <filesystem>
#include <string>
#include <vector>

namespace simulator {

/**
 * @brief One plugin's complete results.
 * @note The name is the plugin's **filename**, not its path: that is how the assignment's samples
 *       identify plugins, and the `errors:` list has to use the same form to be readable alongside
 *       the ranking.
 */
struct PluginOutcome {
    std::string name{};
    types::SimulationManagerReport report{};
};

/**
 * @brief Everything a mode-level report needs to be written.
 *
 * @note Architectural boundary: the writers take results rather than reaching for them. They never
 *       touch a plugin, a map, or the composition - which is what lets the whole grouping and
 *       ranking logic be tested against values built by hand.
 */
struct ModeReportInput {
    /// The composition every plugin was run against.
    std::filesystem::path composition_file{};

    /// The plugin held fixed across the whole mode: the algorithm, or the mission control.
    std::filesystem::path fixed_plugin_file{};

    /// The folder whose plugins were varied.
    std::filesystem::path varied_plugin_folder{};

    /// One entry per plugin that loaded and ran.
    std::vector<PluginOutcome> outcomes{};

    /**
     * @brief Plugins that could not be loaded at all.
     * @note Filenames only. These never reach the run loop, so they have no report to summarise and
     *       can only be named here.
     */
    std::vector<std::string> failed_to_load{};
};

/**
 * @brief Write `comparative_report.yaml`: which mission controls behaved identically.
 * @param input Results and the run's identifying paths.
 * @param output_path Directory to write into; created if missing.
 *
 * @note Plugins are grouped by the **ordered sequence of per-run `(score, steps)`** - two mission
 *       controls agree only if they match run by run. The assignment's own sample rules out grouping
 *       by total score: it shows the same total in two different groups.
 * @note Groups are ordered by member count descending, with ties broken by name so the document is
 *       reproducible. A comparative report that reordered itself between identical runs would defeat
 *       its own purpose.
 * @note Never throws; a directory that cannot be created leaves no report rather than ending the run.
 */
void writeComparativeReport(const ModeReportInput& input, const std::filesystem::path& output_path);

/**
 * @brief Write `competitive_report.yaml`: which algorithm scored best.
 * @param input Results and the run's identifying paths.
 * @param output_path Directory to write into; created if missing.
 *
 * @note Ranked by total score descending, then by total steps **ascending**. Steps break ties only,
 *       which is the single place a slower plugin loses to an equally accurate faster one.
 * @note Totals sum every run, including those that scored the `-1` sentinel. Skipping failures would
 *       flatter a plugin that crashed on a third of its runs.
 * @note Never throws.
 */
void writeCompetitiveReport(const ModeReportInput& input, const std::filesystem::path& output_path);

} // namespace simulator
