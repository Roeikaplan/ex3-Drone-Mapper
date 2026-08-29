/**
 * @file main.cpp
 * @brief Simulator entry point: parse the command line, prepare the output, load the plugins, and
 *        run the composition against them.
 * @note Returns from `main` on every path and never calls `exit()`, per the assignment's rule that
 *       the program always ends by finishing `main`.
 * @note **This file owns the teardown ordering.** Everything holding a plugin-derived object or a
 *       plugin `std::function` has to be destroyed before the libraries are unloaded, and the
 *       registrar outlives `main`, so no scoping trick can arrange that on its own.
 * @note Running the composition against every plugin, and aggregating their reports, belongs to
 *       `SimulationOrchestrator`. What stays here is what only an entry point can do: decide the
 *       arguments are usable, choose where output goes, load the libraries, and unload them last.
 */

#include <Simulator/CommandLineArgs.h>
#include <Simulator/CompositionLoader.h>
#include <Simulator/ConfigIdentityIndex.h>
#include <Simulator/ErrorLogger.h>
#include <Simulator/PluginLoader.h>
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

    simulator::PluginLoader loader;
    {
        simulator::PluginLoadReport plugins;

        const bool comparative = args.mode == simulator::RunMode::Comparative;
        const std::filesystem::path& algorithms_source =
            comparative ? args.fixed_plugin_file : args.varied_plugin_folder;
        const std::filesystem::path& mission_controls_source =
            comparative ? args.varied_plugin_folder : args.fixed_plugin_file;

        loader.load(algorithms_source, simulator::PluginLoader::Kind::Algorithm, plugins);
        loader.load(mission_controls_source, simulator::PluginLoader::Kind::MissionControl, plugins);

        for (const simulator::PluginFailure& failure : plugins.failures) {
            logger.log("PLUGIN_LOAD_FAILED", failure.file.string() + ": " + failure.reason);
        }

        /**
         * @note No branch on `num_threads` here. The executor owns the whole thread rule, including
         *       the case where the answer is no threads at all, so this stays one construction
         *       whatever was asked for.
         * @note Declared before the orchestrator so it outlives it: the orchestrator borrows it by
         *       reference, and a worker must never outlive the object that scheduled it.
         */
        simulator::ThreadPoolExecutor executor{args.num_threads};
        simulator::SimulationOrchestrator orchestrator{
            args, composition.composition, composition_paths, identity, results.path, logger,
            executor};
        orchestrator.execute(plugins);
    }

    /**
     * @note Anything a library registered but the loader never claimed is dropped here, while the
     *       libraries are still mapped.
     */
    simulator::Registrar::instance().clear();

    /**
     * @note Only now. Every plugin instance, every factory copy, and every claimed factory is gone,
     *       so unmapping the code they pointed into is finally safe.
     */
    loader.releaseAll();

    const std::size_t errors = logger.errorCount();
    if (errors > 0) {
        std::cout << errors << " error(s) logged to " << logger.file().string() << "\n";
    }
    return 0;
}
