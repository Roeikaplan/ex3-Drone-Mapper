/**
 * @file main.cpp
 * @brief Simulator entry point: parse the command line, prepare the output, load the plugins, and
 *        run the composition against them.
 * @note Returns from `main` on every path and never calls `exit()`, per the assignment's rule that
 *       the program always ends by finishing `main`.
 * @note **This file owns the teardown ordering.** Everything holding a plugin-derived object or a
 *       plugin `std::function` has to be destroyed before the libraries are unloaded, and the
 *       registrar outlives `main`, so no scoping trick can arrange that on its own.
 * @note One plugin pair is run here. Iterating every plugin in the varied folder, and aggregating
 *       their reports into the mode-level YAML, is the orchestration layer's job.
 */

#include <Simulator/CommandLineArgs.h>
#include <Simulator/CompositionLoader.h>
#include <Simulator/ConfigIdentityIndex.h>
#include <Simulator/ErrorLogger.h>
#include <Simulator/Map3DImpl.h>
#include <Simulator/PluginLoader.h>
#include <Simulator/Registrar.h>
#include <Simulator/ResultsDirectory.h>
#include <Simulator/SimulationManager.h>
#include <Simulator/SimulationOutputWriter.h>
#include <Simulator/SimulationRunFactoryImpl.h>

#include <cstddef>
#include <exception>
#include <iostream>
#include <memory>
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
 * @brief Summarise a plugin pair's report.
 * @param report The results of running the whole composition.
 * @note Reports the scored average and the error count separately: mixing the -1 sentinel into an
 *       average would drag it below the metric's own range and make a mostly-successful batch look
 *       like a failure.
 */
void reportRunSummary(const simulator::types::SimulationManagerReport& report) {
    std::size_t errored = 0;
    double total = 0.0;
    std::size_t scored = 0;
    for (const simulator::types::SimulationResult& result : report.runs) {
        if (result.mission_score < 0.0) {
            ++errored;
        } else {
            total += result.mission_score;
            ++scored;
        }
    }

    std::cout << "runs=" << report.runs.size() << " scored=" << scored << " errored=" << errored;
    if (scored > 0) {
        std::cout << " average_score=" << total / static_cast<double>(scored);
    }
    std::cout << "\n";
}

/**
 * @brief Run the whole composition against every plugin in the varied folder.
 * @param args The accepted command line.
 * @param composition The parsed composition.
 * @param composition_paths Source config paths, used to label runs in each report.
 * @param identity Source names for the configs, used to name output maps.
 * @param plugins The loaded plugins.
 * @param results_directory Where output maps and reports are written.
 * @param logger Sink for run failures.
 * @note One plugin is held fixed and the other varied, exactly as the mode describes. Each pairing
 *       gets its own manager, its own report, and its own output maps - which the plugin prefix in
 *       every filename keeps from colliding.
 * @note **Each manager is scoped to one iteration on purpose.** It owns a factory holding copies of
 *       that plugin's `std::function`s, so letting them accumulate across the loop would keep every
 *       plugin's callables alive at once and erode the teardown invariant.
 * @note What is still missing is the aggregation *across* these reports - the comparative grouping
 *       and the competitive ranking - which needs the mode-level writers.
 */
void runEveryPluginPair(const simulator::CommandLineArgs& args,
                        const simulator::types::SimulationCompositionData& composition,
                        const simulator::CompositionPaths& composition_paths,
                        const simulator::ConfigIdentityIndex& identity,
                        const simulator::PluginLoadReport& plugins,
                        const std::filesystem::path& results_directory,
                        simulator::ErrorLogger& logger) {
    if (plugins.algorithms.empty() || plugins.mission_controls.empty()) {
        logger.log("PLUGIN_SET_INCOMPLETE",
                   "need at least one algorithm and one mission control to run anything");
        return;
    }

    const bool comparative = args.mode == simulator::RunMode::Comparative;
    const std::size_t varied_count =
        comparative ? plugins.mission_controls.size() : plugins.algorithms.size();

    for (std::size_t i = 0; i < varied_count; ++i) {
        const simulator::LoadedAlgorithm& algorithm =
            comparative ? plugins.algorithms.front() : plugins.algorithms[i];
        const simulator::LoadedMissionControl& mission_control =
            comparative ? plugins.mission_controls[i] : plugins.mission_controls.front();

        const std::filesystem::path& varied_file =
            comparative ? mission_control.file : algorithm.file;
        const std::string plugin_label = varied_file.stem().string();

        std::cout << "running plugin pair: algorithm=" << algorithm.file.filename().string()
                  << " mission_control=" << mission_control.file.filename().string() << "\n";

        auto factory = std::make_unique<simulator::SimulationRunFactoryImpl>(
            mission_control.factory, algorithm.factory, plugin_label, identity, args.verbose);
        simulator::SimulationManager manager{std::move(factory), plugin_label, logger};

        const simulator::types::SimulationManagerReport report =
            manager.run(composition, results_directory);
        reportRunSummary(report);
        simulator::writeSimulationOutput(report, results_directory, plugin_label,
                                         composition_paths);
    }
}

} // namespace

/**
 * @brief Entry point.
 * @param argc Argument count.
 * @param argv Argument vector; see `commandLineUsage()` for the accepted forms.
 * @return 0 in every case; failures are reported rather than signalled through the exit status.
 * @note The nested scopes below are the teardown ordering made explicit. The report holding the
 *       plugin factories, and the manager holding its own copies, both die before `Registrar::clear`
 *       and `releaseAll`. Getting this wrong crashes during static destruction, after `main` has
 *       already returned successfully.
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

        runEveryPluginPair(args, composition.composition, composition_paths, identity, plugins,
                           results.path, logger);
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
