#pragma once

#include <filesystem>
#include <fstream>
#include <optional>
#include <string_view>

namespace drone_mapper {

/**
 * @brief Immediate error logger: writes each error to `std::cerr` and, optionally, to a file.
 *
 * Errors are flushed the moment they occur — never buffered or deferred — to satisfy the assignment
 * rule that all errors be logged immediately when they happen. The default-constructed logger writes
 * only to `std::cerr`, which keeps component tests from touching the filesystem; the path constructor
 * additionally appends to a log file (created lazily, `std::ios::app`).
 *
 * @note Architectural boundary: this is the single "log immediately" sink shared by the top layer —
 *       `main`, `CompositionLoader`, and `SimulationManager` all funnel recoverable and group-level
 *       errors here rather than each choosing its own logging mechanism.
 * @note Two sinks, per the assignment: the **error log** (`errors.log`) receives *every* error
 *       immediately, while **`input_errors.txt`** receives only recovered *input-file* errors and is
 *       created lazily — so it exists only when the input actually had recoverable problems.
 */
class ErrorLogger {
public:
    /**
     * @brief Construct a stderr-only logger (no file output).
     */
    ErrorLogger() = default;
    /**
     * @brief Construct a logger that mirrors to `std::cerr` and appends to file sinks.
     * @param errors_log_file Destination error log; its parent directories are created if missing. If
     *        it cannot be opened the logger degrades to stderr-only rather than throwing.
     * @param input_errors_file Optional destination for recovered *input-file* errors
     *        (`input_errors.txt`). Not opened here: it is created lazily on the first
     *        `logInputError` call, so no file appears when the input had no errors. Empty disables it.
     */
    explicit ErrorLogger(std::filesystem::path errors_log_file,
                         std::filesystem::path input_errors_file = {});

    /**
     * @brief Log one error immediately (flushed) to the error log and stderr.
     * @param code Short machine-readable error code (e.g. "DRONE_STEP_ERROR").
     * @param message Human-readable detail.
     */
    void log(std::string_view code, std::string_view message);

    /**
     * @brief Log a recovered input-file error: to the error log (like `log`) AND to `input_errors.txt`.
     * @param code Short machine-readable error code (e.g. "CONFIG_MISSING_KEY").
     * @param message Human-readable detail.
     * @note `input_errors.txt` is created on the first call (only if input errors occur, per the
     *       assignment). Every input error is still an error, so it also goes to the error log.
     */
    void logInputError(std::string_view code, std::string_view message);

private:
    std::optional<std::ofstream> file_{};
    std::filesystem::path input_errors_path_{};
    std::optional<std::ofstream> input_file_{};
};

} // namespace drone_mapper