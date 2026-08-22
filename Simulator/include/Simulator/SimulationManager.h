/**
 * @file SimulationManager.h
 * @brief Expansion of a composition into runs, for one plugin pair.
 */

#pragma once

#include <Simulator/ErrorLogger.h>
#include <Simulator/ISimulation.h>
#include <Simulator/ISimulationRunFactory.h>

#include <memory>
#include <string>

namespace simulator {

/**
 * @brief Runs every configuration combination in a composition against one plugin pair.
 *
 * @note Architectural boundary: the plugin dimension is *not* here. This class knows nothing about
 *       which plugins it is running - the factory it was given is already bound to a pair, so one
 *       `run()` call covers the whole composition for exactly that pair. Running several pairs means
 *       several managers, which is what the mode-level orchestration will do.
 * @note **It owns the factory, and the factory holds plugin `std::function`s.** This object must
 *       therefore be destroyed before any plugin library is unloaded, alongside the loader's own
 *       copies of those factories.
 * @note A failure in one combination never ends the batch: it is logged, scored -1, and the
 *       expansion continues. That is what the assignment prescribes for a group that cannot run.
 */
class SimulationManager final : public ISimulation {
public:
    /**
     * @brief Construct over a bound factory.
     * @param run_factory Factory already bound to one plugin pair; must not be null.
     * @param plugin_label Name of the varied plugin, used in log messages.
     * @param logger Sink for run failures and ignored resolution requests; must outlive this object.
     * @throws std::invalid_argument when @p run_factory is null.
     */
    SimulationManager(std::unique_ptr<ISimulationRunFactory> run_factory, std::string plugin_label,
                      ErrorLogger& logger);

    /**
     * @brief Run every combination the composition describes.
     * @param composition Simulations with their missions, crossed with drones and lidars.
     * @param output_path Directory each run writes its output map into.
     * @return One result per combination, in expansion order, plus report metadata.
     * @note The order is fixed - simulation, then mission, then drone, then lidar - and is relied on
     *       elsewhere: a report labels runs by position, so a change here silently mislabels them.
     */
    [[nodiscard]] types::SimulationManagerReport run(
        const types::SimulationCompositionData& composition,
        const std::filesystem::path& output_path) override;

private:
    std::unique_ptr<ISimulationRunFactory> run_factory_;
    std::string plugin_label_;
    ErrorLogger& logger_;
};

} // namespace simulator
