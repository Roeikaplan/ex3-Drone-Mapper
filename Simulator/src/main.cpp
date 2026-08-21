/**
 * @file main.cpp
 * @brief Simulator entry point: parse the command line, prepare the run's output, then exercise the
 *        plugins it names.
 * @note Returns from `main` on every path and never calls `exit()`, per the assignment's rule that
 *       the program always ends by finishing `main` - including when the command line is rejected or
 *       the results directory cannot be created.
 * @note The smoke-check call is transitional. It is replaced by the orchestrator once composition
 *       loading and the run pipeline exist; until then it keeps the proven plugin lifecycle running
 *       behind the real command line.
 */

#include <Simulator/CommandLineArgs.h>
#include <Simulator/ErrorLogger.h>
#include <Simulator/PluginSmokeCheck.h>
#include <Simulator/ResultsDirectory.h>

#include <iostream>
#include <string>
#include <vector>

namespace {

/**
 * @brief Print the usage text followed by every problem found.
 * @param errors Problems collected by parsing and validation; never empty when this is called.
 * @note Goes straight to `std::cerr` rather than through the `ErrorLogger`. At this point no results
 *       directory has been chosen, and the log file lives inside one - so there is nowhere to log to
 *       yet, which is exactly why argument errors are handled before anything else.
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

} // namespace

/**
 * @brief Entry point.
 * @param argc Argument count.
 * @param argv Argument vector; see `commandLineUsage()` for the accepted forms.
 * @return 0 when the command line was rejected, the output could not be prepared, or the plugins
 *         ran; 1 when a plugin failed.
 * @note A rejected command line returns 0, not a failure code: the assignment treats it as the
 *       program finishing cleanly after reporting, not as a crash.
 * @note Order matters here. The results directory must exist before the logger can open a file
 *       inside it, and the logger must exist before anything that might need to report.
 * @note The two plugin roles map onto the smoke check's parameters by mode. `PluginLoader::load`
 *       already accepts either a single `.so` or a folder, so no branching beyond this is needed.
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

    const bool comparative = args.mode == simulator::RunMode::Comparative;
    const std::filesystem::path& algorithms =
        comparative ? args.fixed_plugin_file : args.varied_plugin_folder;
    const std::filesystem::path& mission_controls =
        comparative ? args.varied_plugin_folder : args.fixed_plugin_file;

    const int outcome = simulator::runPluginSmokeCheck(algorithms, mission_controls, logger);

    const std::size_t errors = logger.errorCount();
    if (errors > 0) {
        std::cout << errors << " error(s) logged to " << logger.file().string() << "\n";
    }
    return outcome;
}
