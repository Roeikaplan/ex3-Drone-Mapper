#include <drone_mapper/CompositionLoader.h>
#include <drone_mapper/ErrorLogger.h>
#include <drone_mapper/SimulationManager.h>
#include <drone_mapper/SimulationOutputWriter.h>
#include <drone_mapper/SimulationRunFactoryImpl.h>

#include <exception>
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>

namespace {

namespace fs = std::filesystem;

/**
 * @brief Resolve the composition-file argument.
 * @param argc,argv Program arguments.
 * @return `simulation.yaml` (in CWD) when absent; otherwise the given path. A filename or relative
 *         path resolves against the CWD when opened; an absolute path is used as-is.
 * @note Matches the assignment's argument rules; `std::filesystem::path` handles the relative/absolute
 *       distinction without extra logic.
 */
[[nodiscard]] fs::path resolveCompositionPath(int argc, char** argv) {
    return argc >= 2 ? fs::path{argv[1]} : fs::path{"simulation.yaml"};
}

/**
 * @brief Resolve the output directory argument.
 * @param argc,argv Program arguments.
 * @return The given output path, or the current working directory when absent.
 */
[[nodiscard]] fs::path resolveOutputPath(int argc, char** argv) {
    return argc >= 3 ? fs::path{argv[2]} : fs::current_path();
}

} // namespace

/**
 * @brief Entry point: load the composition, run every combination, write the report.
 * @note Kept minimal — argument resolution plus wiring loader → manager → writer. It never throws
 *       out (a bad composition is logged and the program returns cleanly) and never calls exit().
 */
int main(int argc, char** argv) {
    const fs::path composition_file = resolveCompositionPath(argc, argv);
    const fs::path output_path = resolveOutputPath(argc, argv);

    // Two sinks: errors.log (every error, immediate) and input_errors.txt (recovered input-file
    // errors only, created lazily by the logger so it appears only when the input had problems).
    drone_mapper::ErrorLogger logger{output_path / "output_results" / "errors.log",
                                     output_path / "input_errors.txt"};

    drone_mapper::types::SimulationCompositionData composition;
    drone_mapper::CompositionPaths composition_paths; // source config paths for the score report
    try {
        composition = drone_mapper::loadComposition(composition_file, logger, &composition_paths);
    } catch (const std::exception& e) {
        // Unrecoverable input: message to screen, no input_errors.txt (this is not a recovered error),
        // return from main — never crash, never exit().
        logger.log("COMPOSITION_LOAD_FAILED", std::string{"could not load composition: "} + e.what());
        std::cerr << "Failed to load " << composition_file << "; nothing to run.\n";
        return 0;
    }

    // Everything past loading is wrapped so no failure can escape main (the program shall end by
    // finishing main in all scenarios).
    try {
        auto run_factory = std::make_unique<drone_mapper::SimulationRunFactoryImpl>();
        drone_mapper::SimulationManager simulation{std::move(run_factory), logger};
        const drone_mapper::types::SimulationManagerReport report =
            simulation.run(composition, output_path);

        drone_mapper::writeSimulationOutput(report, output_path, composition.composition_file,
                                            composition_paths);

        std::cout << "Ran " << report.runs.size() << " simulation run(s); wrote "
                  << (output_path / "simulation_output.yaml") << "\n";
    } catch (const std::exception& e) {
        logger.log("FATAL", std::string{"unrecoverable error: "} + e.what());
        std::cerr << "Unrecoverable error: " << e.what() << "; finishing.\n";
    }
    return 0;
}