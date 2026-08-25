/**
 * @file SimulationTaskTable.h
 * @brief The complete set of runs, known before any of them starts.
 */

#pragma once

#include <Simulator/ISimulationRunFactory.h>
#include <Simulator/SimulationTypes.h>

#include <cstddef>
#include <filesystem>
#include <utility>
#include <vector>

namespace simulator {

/**
 * @brief One run: which plugin pair builds it, and from which four configs.
 *
 * @note Holds a factory pointer rather than a plugin index because that is all execution needs -
 *       `factory->create(...)->run()`. Non-owning: the orchestrator owns the managers that own the
 *       factories, and they outlive the table.
 * @note Config pointers refer into the composition, which must not be copied or resized after
 *       enumeration. The same constraint `ConfigIdentityIndex` already carries.
 */
struct RunCell {
    ISimulationRunFactory* factory = nullptr;
    const types::SimulationConfigData* simulation = nullptr;
    const common::types::MissionConfigData* mission = nullptr;
    const common::types::DroneConfigData* drone = nullptr;
    const common::types::LidarConfigData* lidar = nullptr;
    std::filesystem::path output_path{};

    /**
     * @brief Which plugin, in enumeration order, this run belongs to.
     * @note Set by `append`, not by the caller.
     * @note An index rather than a pointer to the owning manager: the manager needs the table, so a
     *       pointer here would make the two headers mutually dependent. Execution itself does not
     *       need it - only failure reporting, which wants the plugin's name.
     */
    std::size_t plugin_index = 0;
};

/**
 * @brief Every run to be performed, with a result slot reserved for each.
 *
 * @note Architectural boundary: the work set is **fully known before execution begins**, which is
 *       what makes the whole design lock-free. Three properties follow, and phase 08's concurrency
 *       rests on all three:
 *       - Cell *i* is written by exactly one task and read by nobody until every task has finished,
 *         so the results need no mutex. Distinct vector elements are distinct objects.
 *       - The results vector is sized once, before dispatch, and never resized - so no reference
 *         into it can be invalidated.
 *       - Results land in **index** order regardless of **completion** order, so the reports are
 *         identical however the work is scheduled. Comparative grouping would be meaningless
 *         otherwise.
 * @note One flat table across all plugin pairs rather than one per plugin. Running each plugin's set
 *       separately would put a barrier between plugins, leaving threads idle through the tail of
 *       every one.
 */
class SimulationTaskTable {
public:
    /**
     * @brief Add one run to the table.
     * @param cell The run to add; its `plugin_index` is filled in here.
     * @note Only valid before `seal()`. Appending afterwards would resize the results vector out
     *       from under whoever is holding a reference into it.
     */
    void append(RunCell cell);

    /**
     * @brief Record that a plugin's cells have all been appended.
     * @param begin Index of that plugin's first cell.
     * @note Ranges are recorded as enumeration happens rather than derived afterwards, which is what
     *       makes the assumption explicit: a plugin's cells must be **contiguous**. Interleaving two
     *       plugins would silently mis-slice every report.
     */
    void closePluginRange(std::size_t begin);

    /**
     * @brief Reserve a result slot for every cell.
     * @note Must be called after the last `append` and before any execution.
     */
    void seal();

    /**
     * @brief How many runs the table holds.
     * @return The cell count.
     */
    [[nodiscard]] std::size_t size() const noexcept { return cells_.size(); }

    /**
     * @brief The run at an index.
     * @param index Cell index.
     * @return That cell.
     */
    [[nodiscard]] const RunCell& cell(std::size_t index) const { return cells_[index]; }

    /**
     * @brief The result slot for an index.
     * @param index Cell index.
     * @return A reference to that cell's slot, for the task to write.
     * @note Concurrent calls with **different** indices are safe and are how execution is meant to
     *       work. Two tasks sharing an index would not be.
     */
    [[nodiscard]] types::SimulationResult& result(std::size_t index) { return results_[index]; }

    /**
     * @brief How many plugins contributed cells.
     * @return The number of recorded ranges.
     */
    [[nodiscard]] std::size_t pluginCount() const noexcept { return plugin_ranges_.size(); }

    /**
     * @brief One plugin's results.
     * @param plugin_index Which plugin, in enumeration order.
     * @return Its contiguous slice of the results, ready to assemble into a report.
     */
    [[nodiscard]] std::vector<types::SimulationResult> resultsForPlugin(
        std::size_t plugin_index) const;

private:
    std::vector<RunCell> cells_{};
    std::vector<types::SimulationResult> results_{};
    std::vector<std::pair<std::size_t, std::size_t>> plugin_ranges_{};
};

} // namespace simulator
