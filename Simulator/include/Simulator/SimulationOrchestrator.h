/**
 * @file SimulationOrchestrator.h
 * @brief The layer above `ISimulation`: many plugin pairs, one run mode, one set of reports.
 */

#pragma once

#include <Simulator/CommandLineArgs.h>
#include <Simulator/CompositionPaths.h>
#include <Simulator/ConfigIdentityIndex.h>
#include <Simulator/ErrorLogger.h>
#include <Simulator/PluginLoader.h>
#include <Simulator/SimulationManager.h>
#include <Simulator/TaskExecutor.h>

#include <filesystem>
#include <memory>
#include <string>
#include <vector>

namespace simulator {

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
 * @note **Owns the managers, and therefore the factories holding plugin `std::function`s.** It must
 *       be destroyed before the plugin libraries are unloaded; `main` scopes it explicitly rather
 *       than relying on declaration order, because that ordering is not something a reader should
 *       have to reconstruct.
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
     */
    SimulationOrchestrator(const CommandLineArgs& args,
                           const types::SimulationCompositionData& composition,
                           const CompositionPaths& composition_paths,
                           const ConfigIdentityIndex& identity,
                           std::filesystem::path results_directory, ErrorLogger& logger,
                           ITaskExecutor& executor);

    /**
     * @brief Run every plugin pair and write every report.
     * @param plugins The loaded plugins, including those that failed to load.
     * @note Three phases, in order: enumerate all plugins into one table, execute it once, then
     *       assemble and write. Keeping them apart is what lets the executor decide scheduling
     *       without knowing anything about simulations.
     */
    void execute(const PluginLoadReport& plugins);

private:
    /**
     * @brief One plugin pair, and the two names its artefacts are labelled with.
     * @note Two names because they are used for different things and are not interchangeable: the
     *       stem prefixes every file this plugin writes, while the full filename is what the
     *       mode-level report names the plugin by.
     */
    struct PluginRun {
        std::unique_ptr<SimulationManager> manager;
        std::string report_name;
    };

    /**
     * @brief Build one manager per plugin pair.
     * @param plugins The loaded plugins.
     * @note One plugin is held fixed and the other varied, as the mode describes. The plugin prefix
     *       in every output filename is what keeps their artefacts from colliding in one directory.
     */
    void buildManagers(const PluginLoadReport& plugins);

    const CommandLineArgs& args_;
    const types::SimulationCompositionData& composition_;
    const CompositionPaths& composition_paths_;
    const ConfigIdentityIndex& identity_;
    std::filesystem::path results_directory_;
    ErrorLogger& logger_;
    ITaskExecutor& executor_;

    std::vector<PluginRun> plugins_{};
};

} // namespace simulator
