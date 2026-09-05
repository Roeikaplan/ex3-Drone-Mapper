/**
 * @file SimulationTaskTable.cpp
 * @brief Accumulating the run set and slicing its results back out per plugin.
 */

#include <Simulator/SimulationTaskTable.h>

#include <utility>

namespace simulator {

/**
 * @brief Add one run to the table.
 * @param cell The run to add.
 * @note The owning plugin is stamped here rather than supplied by the caller. Because a plugin's
 *       cells are contiguous and its range is closed before the next one begins, the number of ranges
 *       closed so far *is* the current plugin's index - so the one place that already enforces
 *       contiguity is also the one that derives from it.
 */
void SimulationTaskTable::append(RunCell cell) {
    cell.plugin_index = plugin_ranges_.size();
    cells_.push_back(std::move(cell));
}

/**
 * @brief Record that a plugin's cells have all been appended.
 * @param begin Index of that plugin's first cell.
 * @note The range closes at whatever the current cell count is, which is why a plugin must finish
 *       enumerating before the next one starts. Recording it here rather than reconstructing it
 *       later keeps that requirement visible at the point it applies.
 */
void SimulationTaskTable::closePluginRange(std::size_t begin) {
    plugin_ranges_.emplace_back(begin, cells_.size());
}

/**
 * @brief Reserve a result slot for every cell.
 * @note Sizing once, up front, is what lets tasks write concurrently without synchronisation: the
 *       vector never reallocates, so a reference to one slot stays valid while another is written.
 * @note Slots start default-constructed, which scores 0 rather than the -1 sentinel. Every slot is
 *       overwritten by its task, including the failure path, so a default value never survives into
 *       a report - but if one ever did, it would read as a legitimate zero rather than as an error.
 */
void SimulationTaskTable::seal() {
    results_.assign(cells_.size(), types::SimulationResult{});
}

/**
 * @brief How many cells one plugin contributed.
 * @param plugin_index Which plugin, in enumeration order.
 * @return The length of that plugin's range, or 0 if there is no such plugin.
 * @note An unknown index yields 0 rather than throwing, matching `resultsForPlugin`. A caller that
 *       reserves zero uses of a library simply never loads it, which is the correct outcome for a
 *       plugin with nothing to run.
 */
std::size_t SimulationTaskTable::pluginCellCount(std::size_t plugin_index) const {
    if (plugin_index >= plugin_ranges_.size()) {
        return 0;
    }

    const auto [begin, end] = plugin_ranges_[plugin_index];
    return end - begin;
}

/**
 * @brief One plugin's results.
 * @param plugin_index Which plugin, in enumeration order.
 * @return Its contiguous slice of the results.
 * @note Returns a copy rather than a view. The slice is handed to report assembly, which outlives
 *       nothing in particular, and a few dozen results is not worth the lifetime question a view
 *       would introduce.
 */
std::vector<types::SimulationResult> SimulationTaskTable::resultsForPlugin(
    std::size_t plugin_index) const {
    if (plugin_index >= plugin_ranges_.size()) {
        return {};
    }

    const auto [begin, end] = plugin_ranges_[plugin_index];
    return std::vector<types::SimulationResult>{results_.begin() + static_cast<std::ptrdiff_t>(begin),
                                                results_.begin() + static_cast<std::ptrdiff_t>(end)};
}

} // namespace simulator
