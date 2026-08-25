/**
 * @file SimulationManager.h
 * @brief Expansion of a composition into runs, for one plugin pair.
 */

#pragma once

#include <Simulator/ErrorLogger.h>
#include <Simulator/ISimulation.h>
#include <Simulator/ISimulationRunFactory.h>
#include <Simulator/SimulationTaskTable.h>
#include <Simulator/TaskExecutor.h>

#include <memory>
#include <string>
#include <vector>

namespace simulator {

/**
 * @brief Turns a composition into runs for one plugin pair, and their results into a report.
 *
 * @note Architectural boundary: the plugin dimension is *not* here. The factory it is given is
 *       already bound to a pair, so one `run()` covers the whole composition for exactly that pair.
 *       Running several pairs means several managers.
 * @note The work is split into three so that a wider-scoped caller can reuse the halves: `enumerate`
 *       says what runs exist, an executor runs them, and `assemble` turns results into a report.
 *       `run()` is those three at single-plugin scope, which keeps `ISimulation` independently
 *       usable rather than becoming a shell. The orchestrator does the same three steps across every
 *       plugin at once, so no barrier falls between them.
 * @note **It owns the factory, and the factory holds plugin `std::function`s.** This object must be
 *       destroyed before any plugin library is unloaded.
 */
class SimulationManager final : public ISimulation {
public:
    /**
     * @brief Construct over a bound factory.
     * @param run_factory Factory already bound to one plugin pair; must not be null.
     * @param plugin_label Name of the varied plugin, used in log messages.
     * @param logger Sink for run failures; must outlive this object.
     * @throws std::invalid_argument when @p run_factory is null.
     */
    SimulationManager(std::unique_ptr<ISimulationRunFactory> run_factory, std::string plugin_label,
                      ErrorLogger& logger);

    /**
     * @brief Run every combination the composition describes.
     * @param composition Simulations with their missions, crossed with drones and lidars.
     * @param output_path Directory each run writes its output map into.
     * @return One result per combination, in expansion order, plus report metadata.
     * @note Enumerates, runs everything on the calling thread, and assembles - the three steps below
     *       composed. A caller wanting different scheduling uses them directly instead.
     */
    [[nodiscard]] types::SimulationManagerReport run(
        const types::SimulationCompositionData& composition,
        const std::filesystem::path& output_path) override;

    /**
     * @brief Append this plugin's runs to a table.
     * @param composition Simulations with their missions, crossed with drones and lidars.
     * @param output_path Directory each run writes its output map into.
     * @param table Table to append to; this plugin's range is closed before returning.
     * @note The nesting order - simulation, then mission, then drone, then lidar - is a contract.
     *       The report writer walks the source paths in the same order to label runs, so changing it
     *       here silently mislabels every report.
     * @note Cells hold pointers into @p composition, which must therefore outlive the table and must
     *       not be copied or resized afterwards.
     */
    void enumerate(const types::SimulationCompositionData& composition,
                   const std::filesystem::path& output_path, SimulationTaskTable& table);

    /**
     * @brief Execute one cell and record its outcome.
     * @param table Table holding the cell and its result slot.
     * @param index Which cell to run.
     * @note Writes only `table.result(index)`, which is what makes concurrent execution of distinct
     *       indices safe without any synchronisation.
     * @note Catches everything. A failed cell is logged, scored -1, and the batch continues - and
     *       under a concurrent executor an escaping exception would terminate the process outright,
     *       so the containment has to live at this level rather than around the loop.
     */
    void runCell(SimulationTaskTable& table, std::size_t index);

    /**
     * @brief Turn this plugin's results into its report.
     * @param results One plugin's results, in expansion order.
     * @return The assembled report.
     */
    [[nodiscard]] types::SimulationManagerReport assemble(
        const types::SimulationCompositionData& composition,
        std::vector<types::SimulationResult> results) const;

    /**
     * @brief The name of the plugin this manager runs.
     * @return The label given at construction.
     */
    [[nodiscard]] const std::string& pluginLabel() const noexcept { return plugin_label_; }

private:
    std::unique_ptr<ISimulationRunFactory> run_factory_;
    std::string plugin_label_;
    ErrorLogger& logger_;
};

} // namespace simulator
