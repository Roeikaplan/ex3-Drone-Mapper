/**
 * @file SimulationOrchestrator.cpp
 * @brief Enumerating every plugin's runs, executing them in one pass, and writing what came out.
 */

#include <Simulator/SimulationOrchestrator.h>

#include <Simulator/ModeReportWriters.h>
#include <Simulator/SimulationOutputWriter.h>
#include <Simulator/SimulationRunFactoryImpl.h>
#include <Simulator/SimulationTaskTable.h>

#include <cstddef>
#include <iostream>
#include <utility>

namespace simulator {
namespace {

/**
 * @brief Print a plugin's headline numbers.
 * @param label The plugin's name.
 * @param report Its results.
 * @note The scored average excludes errored runs while the counts include them. Folding the -1
 *       sentinel into an average would drag it below the metric's own range and make a
 *       mostly-successful batch look like a failure.
 */
void reportRunSummary(const std::string& label, const types::SimulationManagerReport& report) {
    std::size_t errored = 0;
    std::size_t scored = 0;
    double total = 0.0;
    for (const types::SimulationResult& result : report.runs) {
        if (result.mission_score < 0.0) {
            ++errored;
        } else {
            total += result.mission_score;
            ++scored;
        }
    }

    std::cout << label << ": runs=" << report.runs.size() << " scored=" << scored
              << " errored=" << errored;
    if (scored > 0) {
        std::cout << " average_score=" << total / static_cast<double>(scored);
    }
    std::cout << "\n";
}

} // namespace

/**
 * @brief Prepare a run.
 * @param args The accepted command line.
 * @param composition The parsed composition.
 * @param composition_paths Source config paths.
 * @param identity Source names for the configs.
 * @param results_directory Where maps and reports are written.
 * @param logger Sink for failures.
 * @param executor How the runs are scheduled.
 */
SimulationOrchestrator::SimulationOrchestrator(const CommandLineArgs& args,
                                               const types::SimulationCompositionData& composition,
                                               const CompositionPaths& composition_paths,
                                               const ConfigIdentityIndex& identity,
                                               std::filesystem::path results_directory,
                                               ErrorLogger& logger, ITaskExecutor& executor)
    : args_(args),
      composition_(composition),
      composition_paths_(composition_paths),
      identity_(identity),
      results_directory_(std::move(results_directory)),
      logger_(logger),
      executor_(executor) {}

/**
 * @brief Build one manager per plugin pair.
 * @param plugins The loaded plugins.
 * @note The varied side is the mission controls in comparative mode and the algorithms in
 *       competitive mode; the other is held fixed at its first entry. That asymmetry is the whole
 *       difference between the two modes.
 * @note Every manager is built before anything runs, and they all stay alive until the reports are
 *       written - unlike the per-iteration scoping this replaces. That is unavoidable once the runs
 *       are interleaved, and it is why the orchestrator itself has to be destroyed before the
 *       libraries are unloaded.
 */
void SimulationOrchestrator::buildManagers(const PluginLoadReport& plugins) {
    const bool comparative = args_.mode == RunMode::Comparative;
    const std::size_t varied_count =
        comparative ? plugins.mission_controls.size() : plugins.algorithms.size();

    for (std::size_t i = 0; i < varied_count; ++i) {
        const LoadedAlgorithm& algorithm =
            comparative ? plugins.algorithms.front() : plugins.algorithms[i];
        const LoadedMissionControl& mission_control =
            comparative ? plugins.mission_controls[i] : plugins.mission_controls.front();

        const std::filesystem::path& varied_file =
            comparative ? mission_control.file : algorithm.file;
        const std::string plugin_label = varied_file.stem().string();

        std::cout << "plugin pair: algorithm=" << algorithm.file.filename().string()
                  << " mission_control=" << mission_control.file.filename().string() << "\n";

        auto factory = std::make_unique<SimulationRunFactoryImpl>(
            mission_control.factory, algorithm.factory, plugin_label, identity_, args_.verbose);
        plugins_.push_back(
            PluginRun{std::make_unique<SimulationManager>(std::move(factory), plugin_label, logger_),
                      varied_file.filename().string()});
    }
}

/**
 * @brief Run every plugin pair and write every report.
 * @param plugins The loaded plugins, including those that failed to load.
 * @note The single `forEach` over the whole table is the point of this class. Every cell of every
 *       plugin is dispatched together, so no barrier falls between plugins and nothing waits for one
 *       plugin's tail before the next begins.
 * @note Reports are assembled only after execution has finished. Until then no result slot is read,
 *       which is the condition that makes writing them without synchronisation correct.
 */
void SimulationOrchestrator::execute(const PluginLoadReport& plugins) {
    if (plugins.algorithms.empty() || plugins.mission_controls.empty()) {
        logger_.log("PLUGIN_SET_INCOMPLETE",
                    "need at least one algorithm and one mission control to run anything");
        return;
    }

    buildManagers(plugins);

    SimulationTaskTable table;
    for (const PluginRun& plugin : plugins_) {
        plugin.manager->enumerate(composition_, results_directory_, table);
    }
    table.seal();

    std::cout << "running " << plugins_.size() << " plugin pair(s), " << table.size()
              << " run(s) total\n";

    /**
     * @note The cell names its own plugin, so one dispatch covers all of them. Execution needs only
     *       the factory the cell already carries; the manager is looked up because it holds the
     *       label that failures are reported under.
     */
    executor_.forEach(table.size(), [this, &table](std::size_t index) {
        plugins_[table.cell(index).plugin_index].manager->runCell(table, index);
    });

    ModeReportInput mode_input{};
    mode_input.composition_file = args_.composition_file;
    mode_input.fixed_plugin_file = args_.fixed_plugin_file;
    mode_input.varied_plugin_folder = args_.varied_plugin_folder;

    /**
     * @note A plugin that never loaded has no results to summarise, so naming it here is the only way
     *       the report accounts for a folder entry that was asked for and produced nothing.
     */
    for (const PluginFailure& failure : plugins.failures) {
        mode_input.failed_to_load.push_back(failure.file.filename().string());
    }

    for (std::size_t index = 0; index < plugins_.size(); ++index) {
        const PluginRun& plugin = plugins_[index];
        const std::string& label = plugin.manager->pluginLabel();
        types::SimulationManagerReport report =
            plugin.manager->assemble(composition_, table.resultsForPlugin(index));

        reportRunSummary(label, report);
        writeSimulationOutput(report, results_directory_, label, composition_paths_);
        mode_input.outcomes.push_back(PluginOutcome{plugin.report_name, std::move(report)});
    }

    /**
     * @note Written last, because it is the only artefact that compares plugins to one another rather
     *       than describing one in isolation.
     */
    if (args_.mode == RunMode::Comparative) {
        writeComparativeReport(mode_input, results_directory_);
    } else {
        writeCompetitiveReport(mode_input, results_directory_);
    }
}

} // namespace simulator
