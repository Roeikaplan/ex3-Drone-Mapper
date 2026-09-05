/**
 * @file SimulationOrchestrator.h
 * @brief The layer above `ISimulation`: many plugin pairs, one run mode, one set of reports.
 */

#pragma once

#include <Simulator/CommandLineArgs.h>
#include <Simulator/CompositionPaths.h>
#include <Simulator/ConfigIdentityIndex.h>
#include <Simulator/ErrorLogger.h>
#include <Simulator/PluginDiscovery.h>
#include <Simulator/PluginRegistry.h>
#include <Simulator/SimulationManager.h>
#include <Simulator/TaskExecutor.h>

#include <filesystem>
#include <memory>
#include <string>
#include <vector>

namespace simulator {

/**
 * @brief The plugins a run mode was pointed at, discovered but not yet loaded.
 *
 * @note Slots, not factories. Nothing here has been `dlopen`ed: the whole task table is built from
 *       filenames, and each library is mapped later by the first run that actually needs it.
 * @note `failures` holds only what discovery itself could detect - an unreadable folder, a path that
 *       is neither file nor directory. A `.so` that exists but cannot be *loaded* is not known yet,
 *       and surfaces after execution instead.
 */
struct PluginSet {
    std::vector<PluginSlot*> algorithms{};
    std::vector<PluginSlot*> mission_controls{};
    std::vector<PluginFailure> failures{};
};

/**
 * @brief Runs a whole composition against every plugin in the varied folder, then reports.
 *
 * @note Architectural boundary: this is the layer the provided interfaces have no room for.
 *       `ISimulation::run` and `ISimulationRunFactory::create` mention no plugin, no mode and no
 *       thread budget, and cannot be changed - so everything the assignment adds beyond Assignment 2
 *       lives here, wrapped around them rather than inside them.
 * @note It enumerates **every** plugin's runs into one table and executes them in a single pass.
 *       Running each plugin's set separately would put a barrier between plugins, leaving threads
 *       idle through the tail of each - which matters as soon as execution is concurrent.
 * @note That single pass is also what bounds how many libraries are mapped at once. Cells are
 *       dispatched from one monotonically advancing cursor, so a plugin is mapped only while one of
 *       its own cells is in flight - which no more than the live threads can be.
 * @note **It owns the managers, but the managers no longer hold plugin callables**, so unloading is
 *       driven by the runs themselves rather than by this object's destruction. `main` still scopes
 *       it explicitly, because the registry's final sweep must come after the managers are gone.
 */
class SimulationOrchestrator {
public:
    /**
     * @brief Prepare a run.
     * @param args The accepted command line.
     * @param composition The parsed composition; must outlive this object and must not be copied.
     * @param composition_paths Source config paths, used to label runs in each report.
     * @param identity Source names for the configs, used to name output maps.
     * @param results_directory Where maps and reports are written.
     * @param logger Sink for failures; must outlive this object.
     * @param executor How the runs are scheduled; must outlive this object.
     * @param registry Owner of the plugin libraries; must outlive this object.
     */
    SimulationOrchestrator(const CommandLineArgs& args,
                           const types::SimulationCompositionData& composition,
                           const CompositionPaths& composition_paths,
                           const ConfigIdentityIndex& identity,
                           std::filesystem::path results_directory, ErrorLogger& logger,
                           ITaskExecutor& executor, PluginRegistry& registry);

    /**
     * @brief Run every plugin pair and write every report.
     * @param plugins The discovered plugins, including anything discovery itself rejected.
     * @note Four phases, in order: enumerate all plugins into one table, reserve one use of each
     *       library per run that will need it, execute the table once, then assemble and write.
     *       The reservation phase is what lets a library be unloaded the instant its last run ends,
     *       and it is only possible because the table is complete before anything starts.
     */
    void execute(const PluginSet& plugins);

private:
    /**
     * @brief One plugin pair, the two names its artefacts are labelled with, and its two libraries.
     * @note Two names because they are used for different things and are not interchangeable: the
     *       stem prefixes every file this plugin writes, while the full filename is what the
     *       mode-level report names the plugin by.
     * @note The slots are kept so that, once execution is over, this pair can be asked whether its
     *       varied library ever loaded - which decides whether it is reported as a result or as an
     *       error.
     */
    struct PluginRun {
        std::unique_ptr<SimulationManager> manager;
        std::string report_name;
        PluginSlot* algorithm_slot = nullptr;
        PluginSlot* mission_control_slot = nullptr;
        PluginSlot* varied_slot = nullptr;
    };

    /**
     * @brief Build one manager per plugin pair.
     * @param plugins The discovered plugins.
     * @note One plugin is held fixed and the other varied, as the mode describes. The plugin prefix
     *       in every output filename is what keeps their artefacts from colliding in one directory.
     * @note Loads nothing, and neither does the enumeration that follows it.
     */
    void buildManagers(const PluginSet& plugins);

    const CommandLineArgs& args_;
    const types::SimulationCompositionData& composition_;
    const CompositionPaths& composition_paths_;
    const ConfigIdentityIndex& identity_;
    std::filesystem::path results_directory_;
    ErrorLogger& logger_;
    ITaskExecutor& executor_;
    PluginRegistry& registry_;

    std::vector<PluginRun> plugins_{};
};

} // namespace simulator
