/**
 * @file main.cpp
 * @brief Simulator entry point: parse the command line, prepare the output, load the plugins, and
 *        run the composition against them.
 * @note Returns from `main` on every path and never calls `exit()`, per the assignment's rule that
 *       the program always ends by finishing `main`.
 * @note **This file owns the teardown ordering.** Everything holding a plugin-derived object or a
 *       plugin `std::function` has to be destroyed before the libraries are unloaded, and the
 *       registrar outlives `main`, so no scoping trick can arrange that on its own.
 * @note Under the lazy plugin lifecycle most libraries have already unloaded themselves long before
 *       this file's teardown runs - each one goes the moment its last run finishes, on whichever
 *       thread that was. What remains here is the sweep for libraries no run ever needed, and it is
 *       still written out step by step, because it is also the ordering every other unload obeys.
 * @note Running the composition against every plugin, and aggregating their reports, belongs to
 *       `SimulationOrchestrator`. What stays here is what only an entry point can do: decide the
 *       arguments are usable, choose where output goes, load the libraries, and unload them last.
 */

#include <Simulator/CommandLineArgs.h>
#include <Simulator/CompositionLoader.h>
#include <Simulator/ConfigIdentityIndex.h>
#include <Simulator/ErrorLogger.h>
#include <Simulator/PluginLifecycleLog.h>
#include <Simulator/PluginRegistry.h>
#include <Simulator/Registrar.h>
#include <Simulator/ResultsDirectory.h>
#include <Simulator/SimulationOrchestrator.h>
#include <Simulator/TaskExecutor.h>

#include <cstddef>
#include <iostream>
#include <string>
#include <tuple>
#include <vector>

namespace {

/**
 * @brief Print the usage text followed by every problem found.
 * @param errors Problems collected by parsing and validation; never empty when this is called.
 * @note Goes straight to `std::cerr` rather than through the `ErrorLogger`. At this point no results
 *       directory has been chosen, and the log file lives inside one - which is exactly why argument
 *       errors are handled before anything else.
 */
void reportRejection(const std::vector<std::string>& errors) {
    std::cerr << simulator::commandLineUsage() << "\n";
    for (const std::string& error : errors) {
        std::cerr << "error: " << error << "\n";
    }
}

/**
 * @brief Echo the configuration the command line resolved to.
 * @param args The accepted arguments.
 * @param results_directory Where this run's artefacts will be written.
 * @note Printed before any plugin is touched so a mis-typed but *valid* command line is obvious in
 *       the transcript rather than only in the eventual report.
 */
void reportConfiguration(const simulator::CommandLineArgs& args,
                         const std::filesystem::path& results_directory) {
    const bool comparative = args.mode == simulator::RunMode::Comparative;
    std::cout << "mode=" << (comparative ? "comparative" : "competition")
              << " threads=" << args.num_threads << " verbose=" << (args.verbose ? "yes" : "no")
              << "\n";
    std::cout << "composition=" << args.composition_file.string() << "\n";
    std::cout << (comparative ? "algorithm=" : "mission_control=")
              << args.fixed_plugin_file.string() << "\n";
    std::cout << (comparative ? "mission_control_folder=" : "algorithms_folder=")
              << args.varied_plugin_folder.string() << "\n";
    std::cout << "results=" << results_directory.string() << "\n";
}

/**
 * @brief Echo what the composition file turned out to contain.
 * @param composition The parsed composition.
 * @note One line rather than a dump: enough to notice at a glance that the wrong dataset was named,
 *       without burying the run's real output.
 */
void reportComposition(const simulator::types::SimulationCompositionData& composition) {
    std::size_t pairs = 0;
    for (const auto& group : composition.simulation_mission_groups) {
        pairs += std::get<1>(group).size();
    }
    std::cout << "simulations=" << composition.simulation_mission_groups.size()
              << " pairs=" << pairs << " drones=" << composition.drone_configs.size()
              << " lidars=" << composition.lidar_configs.size() << "\n";
}

/**
 * @brief Print what the plugin lifecycle actually did, and record the same in the audit log.
 * @param registry The registry every load went through.
 * @param lifecycle The audit log to close out.
 * @note This is the evidence for the lazy-loading claim, so it names the three properties directly
 *       rather than leaving them to be inferred: how many libraries were mapped at once at the
 *       worst moment, that no file was ever opened twice, and that none is still mapped.
 * @note Printed after every worker has joined and after the final sweep, so the counters cannot be
 *       caught mid-change.
 */
void reportPluginLifecycle(const simulator::PluginRegistry& registry,
                           simulator::PluginLifecycleLog& lifecycle) {
    const simulator::PluginLibraryStats stats = simulator::pluginLibraryStats();
    lifecycle.recordSummary(registry.discoveredCount(), registry.loadedCount());

    std::cout << "plugin lifecycle: discovered=" << registry.discoveredCount()
              << " loaded=" << registry.loadedCount() << " dlopen=" << stats.opens
              << " dlclose=" << stats.closes << " peak_mapped=" << stats.peak_open
              << " mapped_at_end=" << stats.currently_open << "\n";
}

} // namespace

/**
 * @brief Entry point.
 * @param argc Argument count.
 * @param argv Argument vector; see `commandLineUsage()` for the accepted forms.
 * @return 0 in every case; failures are reported rather than signalled through the exit status.
 * @note The nested scope below is the teardown ordering made explicit. The load report holding the
 *       plugin factories, and the orchestrator holding its own copies of them, both die before
 *       `Registrar::clear` and `releaseAll`. Getting this wrong crashes during static destruction,
 *       after `main` has already returned successfully.
 */
int main(int argc, char** argv) {
    simulator::CommandLineParseResult parsed = simulator::parseCommandLine(argc, argv);
    simulator::validateCommandLinePaths(parsed.args, parsed.errors);

    if (!parsed.ok()) {
        reportRejection(parsed.errors);
        return 0;
    }

    const simulator::CommandLineArgs& args = parsed.args;

    const simulator::ResultsDirectory results = simulator::createResultsDirectory(args);
    if (!results.ok()) {
        std::cerr << "error: " << results.error << "\n";
        return 0;
    }

    simulator::ErrorLogger logger{results.path / "errors.log"};
    reportConfiguration(args, results.path);

    simulator::CompositionPaths composition_paths;
    const simulator::CompositionLoadResult composition =
        simulator::loadComposition(args.composition_file, logger, &composition_paths);
    if (!composition.ok()) {
        logger.log("COMPOSITION_LOAD_FAILED", composition.error);
        return 0;
    }
    reportComposition(composition.composition);

    /**
     * @note Built once over the composition, which is not copied or resized afterwards - the index
     *       records addresses, so a copy would leave every output map named "unknown".
     */
    const simulator::ConfigIdentityIndex identity{composition.composition, composition_paths};

    /**
     * @note Declared before the registry, and therefore destroyed after it: the registry records an
     *       event on every load and unload, including those performed by its own final sweep.
     */
    simulator::PluginLifecycleLog lifecycle{results.path / "plugin_lifecycle.log"};

    /**
     * @note The registry stays in `main`, outside the scope below, for the same reason the loader it
     *       replaces did: it owns the `dlopen` handles, so it must outlive everything that borrows a
     *       factory from them.
     */
    simulator::PluginRegistry registry{logger, lifecycle};
    {
        const bool comparative = args.mode == simulator::RunMode::Comparative;
        const std::filesystem::path& algorithms_source =
            comparative ? args.fixed_plugin_file : args.varied_plugin_folder;
        const std::filesystem::path& mission_controls_source =
            comparative ? args.varied_plugin_folder : args.fixed_plugin_file;

        /**
         * @note Discovery only lists files. Not one `.so` is mapped here, which is the whole point:
         *       a library is loaded by the first run that needs it and unloaded by the last one to
         *       finish with it, so at no moment is a folder's worth of plugins resident at once.
         */
        simulator::PluginRegistry::Discovery algorithms =
            registry.discover(algorithms_source, simulator::PluginKind::Algorithm);
        simulator::PluginRegistry::Discovery mission_controls =
            registry.discover(mission_controls_source, simulator::PluginKind::MissionControl);

        simulator::PluginSet plugins;
        plugins.algorithms = std::move(algorithms.slots);
        plugins.mission_controls = std::move(mission_controls.slots);
        plugins.failures = std::move(algorithms.failures);
        plugins.failures.insert(plugins.failures.end(), mission_controls.failures.begin(),
                                mission_controls.failures.end());

        /**
         * @note These are the failures discovery itself can see - an unreadable path. A `.so` that
         *       exists but will not load is reported later, by the run that first tries to use it.
         */
        for (const simulator::PluginFailure& failure : plugins.failures) {
            logger.log("PLUGIN_DISCOVERY_FAILED", failure.file.string() + ": " + failure.reason);
        }

        std::cout << "plugins discovered: " << plugins.algorithms.size() << " algorithm(s), "
                  << plugins.mission_controls.size() << " mission control(s)\n";

        /**
         * @note No branch on `num_threads` here. The executor owns the whole thread rule, including
         *       the case where the answer is no threads at all, so this stays one construction
         *       whatever was asked for.
         * @note Declared before the orchestrator so it outlives it: the orchestrator borrows it by
         *       reference, and a worker must never outlive the object that scheduled it.
         */
        simulator::ThreadPoolExecutor executor{args.num_threads};
        simulator::SimulationOrchestrator orchestrator{
            args,          composition.composition, composition_paths, identity,
            results.path,  logger,                  executor,          registry};

        /**
         * @note TEARDOWN STEP 1 - join the workers, and STEP 2 - destroy every run.
         *       Both complete inside this call, which is why neither appears as a statement below.
         *       `ThreadPoolExecutor::forEach` joins its pool before returning, and each
         *       `SimulationRunImpl` is a local of `SimulationManager::runCell` destroyed at the end of
         *       its own cell. So when `execute` returns, no thread is inside plugin code and no plugin
         *       *instance* is alive.
         * @note STEP 3 also happens inside, per library rather than once at the end: the guard in
         *       each cell gives back that run's use, and the run that returns a library's last use
         *       destroys its factory and unmaps it there and then.
         */
        orchestrator.execute(plugins);

        /**
         * @note The scope still exists, and still matters. The managers hold slot *references*, and
         *       the registry's final sweep below must not run while anything that could still ask it
         *       for a factory is alive.
         */
    }

    /**
     * @note TEARDOWN STEP 4a - drop anything a library registered that the registry never claimed.
     *       These are `std::function`s too, and the registrar is a singleton that outlives `main`, so
     *       no scope can reach them; they have to be cleared by hand, while the libraries are still
     *       mapped.
     */
    simulator::Registrar::instance().clear();

    /**
     * @note TEARDOWN STEP 4b - and only now. Normally this finds nothing: every library that ran
     *       anything has already been unloaded by its last run. What it catches is the library no run
     *       ever needed - the composition was empty, or the plugin set was incomplete - which would
     *       otherwise stay mapped until process exit.
     * @note Getting this order wrong does not fail here. It crashes during static destruction, after
     *       `main` has already returned 0, with a stack that names nothing in this project - which is
     *       exactly why the sequence is written out rather than left to destructor order.
     */
    registry.releaseAll();

    reportPluginLifecycle(registry, lifecycle);

    const std::size_t errors = logger.errorCount();
    if (errors > 0) {
        std::cout << errors << " error(s) logged to " << logger.file().string() << "\n";
    }
    return 0;
}
