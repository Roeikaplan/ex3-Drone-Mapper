/**
 * @file CommandLineArgs.h
 * @brief Parsing and validation of the simulator's command line.
 * @note Split into a pure parse and a separate filesystem validation so the parse rules can be
 *       table-tested without touching disk.
 */

#pragma once

#include <cstddef>
#include <filesystem>
#include <string>
#include <vector>

namespace simulator {

/**
 * @brief Which of the two run modes the simulator was asked for.
 * @note The spelling of the flags is fixed by the assignment: `-comparative` and `-competition`.
 *       The competitive *report* is nevertheless named `competitive_report`, so the two must not be
 *       assumed to share a spelling.
 */
enum class RunMode { Comparative, Competition };

/**
 * @brief A fully resolved command line.
 *
 * @note Architectural boundary: the two modes are the same shape with the plugin roles swapped -
 *       one kind is held fixed as a single `.so` while the other is varied across a folder. The
 *       fields name the **roles** rather than the argument keys, so downstream code needs one path
 *       instead of a branch per mode. `mode` still says which key name a report should print.
 * @note It also follows that the results directory always goes under `varied_plugin_folder`:
 *       `mission_control_folder` in comparative mode, `algorithms_folder` in competitive mode.
 */
struct CommandLineArgs {
    /// Which mode was requested.
    RunMode mode = RunMode::Comparative;

    /// The `simulation=` composition YAML.
    std::filesystem::path composition_file{};

    /**
     * @brief The single plugin held fixed for the whole run.
     * @note `algorithm=` in comparative mode, `mission_control=` in competitive mode.
     */
    std::filesystem::path fixed_plugin_file{};

    /**
     * @brief The folder of plugins varied across the run.
     * @note `mission_control_folder=` in comparative mode, `algorithms_folder=` in competitive mode.
     */
    std::filesystem::path varied_plugin_folder{};

    /**
     * @brief Worker threads to spawn *in addition to* the main thread.
     * @note 1 means the main thread does the work and no worker is spawned. A supplied 0 is
     *       normalised to 1, since zero additional threads is exactly what 1 already means.
     */
    std::size_t num_threads = 1;

    /// Whether `-verbose` was given.
    bool verbose = false;
};

/**
 * @brief The outcome of parsing, carrying every problem found rather than only the first.
 * @note Collecting rather than fail-fast is a requirement: the assignment asks for an error naming
 *       *every* unsupported argument and detailing *every* missing one.
 */
struct CommandLineParseResult {
    CommandLineArgs args{};
    std::vector<std::string> errors{};

    /**
     * @brief Whether the command line was usable.
     * @return True when no error was recorded.
     */
    [[nodiscard]] bool ok() const noexcept { return errors.empty(); }
};

/**
 * @brief Parse the command line into a typed configuration.
 * @param argc Argument count as given to `main`.
 * @param argv Argument vector as given to `main`; `argv[0]` is skipped.
 * @return The resolved arguments plus every problem found.
 * @note Never throws and never exits. A malformed command line is reported through the result so the
 *       caller can print usage and return from `main`.
 * @note Performs no filesystem access at all; call `validateCommandLinePaths` for that.
 */
[[nodiscard]] CommandLineParseResult parseCommandLine(int argc, char** argv);

/**
 * @brief Check that the paths a command line named actually exist and are usable.
 * @param args Arguments produced by `parseCommandLine`.
 * @param errors Error list to append to, normally the one `parseCommandLine` returned.
 * @note Only paths that were actually supplied are checked. A missing argument has already been
 *       reported as missing, and reporting it again as nonexistent would double every error on a
 *       bare invocation.
 * @note Never throws: all filesystem queries use their `std::error_code` overloads.
 */
void validateCommandLinePaths(const CommandLineArgs& args, std::vector<std::string>& errors);

/**
 * @brief The usage text printed whenever a command line is rejected.
 * @return A multi-line synopsis of both modes and every argument.
 */
[[nodiscard]] std::string commandLineUsage();

} // namespace simulator
