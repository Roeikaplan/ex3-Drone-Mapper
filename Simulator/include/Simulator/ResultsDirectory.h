/**
 * @file ResultsDirectory.h
 * @brief Creation of the timestamped folder every artefact of one run is written into.
 */

#pragma once

#include <Simulator/CommandLineArgs.h>

#include <filesystem>
#include <string>

namespace simulator {

/**
 * @brief The outcome of trying to create a run's results directory.
 * @note Carries its failure rather than throwing, matching how the command line reports problems -
 *       `main` prints it and returns, and nothing crashes.
 */
struct ResultsDirectory {
    /**
     * @brief The directory created.
     * @note Empty when creation failed; check `ok()` before using it.
     */
    std::filesystem::path path{};

    /**
     * @brief Why creation failed.
     * @note Empty on success. Printed verbatim by `main`, so it must read as a complete sentence
     *       to a user who has never seen the code.
     */
    std::string error{};

    /**
     * @brief Whether the directory was created.
     * @return True when no error was recorded.
     */
    [[nodiscard]] bool ok() const noexcept { return error.empty(); }
};

/**
 * @brief Create the results directory for one run.
 * @param args An accepted command line; its paths must already have been validated.
 * @return The created directory, or the reason it could not be created.
 * @note The parent is always `args.varied_plugin_folder`. The assignment states this as two separate
 *       rules - results go under `mission_control_folder` in comparative mode and under
 *       `algorithms_folder` in competitive mode - but those are the same folder once the arguments
 *       are named by role rather than by key, so there is no branch here beyond the name prefix.
 * @note The name is `comparative_results_<time>` or `competition_<time>` with a UTC stamp, and is
 *       guaranteed not to collide with anything already present: a taken name is retried with a
 *       numeric suffix rather than reused, since a second-resolution stamp alone is not enough for
 *       two runs started in the same second.
 * @note Never throws; a filesystem failure becomes `error`.
 */
[[nodiscard]] ResultsDirectory createResultsDirectory(const CommandLineArgs& args);

} // namespace simulator
