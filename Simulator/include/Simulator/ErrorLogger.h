/**
 * @file ErrorLogger.h
 * @brief The single immediate-write error sink shared by every layer of the simulator.
 */

#pragma once

#include <cstddef>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <optional>
#include <string_view>

namespace simulator {

/**
 * @brief Writes every error to `std::cerr` and, when constructed with a path, to a log file.
 *
 * Each line is flushed the moment it is produced, never buffered for a later pass, which is what the
 * rule "log every error immediately at the point it occurs" actually demands.
 *
 * @note Architectural boundary: this is the one place the top layer funnels errors through - the
 *       plugin loader, the composition loader, the run manager, and `main` all report here rather
 *       than each inventing its own mechanism.
 * @note Thread-safety posture: this is one of only two shared mutable objects in the design (the
 *       other is `std::cout`). It is deliberately synchronised, and it is synchronised *because* it
 *       is shared - by contrast the results table needs no lock at all, since concurrent tasks write
 *       to disjoint cells. Mixing those two postures up is how a lock ends up in the wrong place.
 * @note Neither copyable nor movable: it owns a mutex and an open stream, and is always passed by
 *       reference.
 */
class ErrorLogger {
public:
    /**
     * @brief Construct a logger that writes only to `std::cerr`.
     * @note Exists so component tests can exercise error paths without touching the filesystem.
     */
    ErrorLogger() = default;

    /**
     * @brief Construct a logger that mirrors to `std::cerr` and appends to a file.
     * @param errors_log_file Destination log; missing parent directories are created.
     * @note Best effort by design. If the file cannot be created or opened, the logger silently
     *       degrades to stderr-only rather than throwing - the logger must never be the thing that
     *       crashes the program it exists to report on.
     */
    explicit ErrorLogger(std::filesystem::path errors_log_file);

    ErrorLogger(const ErrorLogger&) = delete;
    ErrorLogger& operator=(const ErrorLogger&) = delete;
    ErrorLogger(ErrorLogger&&) = delete;
    ErrorLogger& operator=(ErrorLogger&&) = delete;

    /**
     * @brief Record one error immediately.
     * @param code Short machine-readable code, e.g. "PLUGIN_LOAD_FAILED".
     * @param message Human-readable detail.
     * @note Both sinks are written under a single lock, so a line can never be split by another
     *       thread on either the file or the terminal, and the two transcripts stay correlated.
     */
    void log(std::string_view code, std::string_view message);

    /**
     * @brief Record a recovered input-file error.
     * @param code Short machine-readable code, e.g. "CONFIG_MISSING_KEY".
     * @param message Human-readable detail.
     * @note Goes to the same file as `log` - Assignment 3 asks only for an error log, unlike
     *       Assignment 2 which mandated a separate `input_errors.txt`. The entry point is kept
     *       distinct anyway so call sites state that the problem was recoverable input rather than a
     *       failure, so the count is available separately, and so a second sink could be restored in
     *       one place if it is ever wanted again.
     */
    void logInputError(std::string_view code, std::string_view message);

    /**
     * @brief How many errors have been recorded in total.
     * @return The count, including those recorded through `logInputError`.
     */
    [[nodiscard]] std::size_t errorCount() const;

    /**
     * @brief How many of the recorded errors were recovered input problems.
     * @return The count of `logInputError` calls.
     * @note A non-zero value means the input files themselves were malformed but usable, which is a
     *       different situation from the run having failed.
     */
    [[nodiscard]] std::size_t inputErrorCount() const;

    /**
     * @brief The file this logger writes to.
     * @return The configured path, or an empty path for a stderr-only logger.
     * @note Non-empty does not guarantee the file opened; see the path constructor's note.
     */
    [[nodiscard]] const std::filesystem::path& file() const noexcept { return file_path_; }

private:
    /**
     * @brief Write one formatted line to both sinks.
     * @param code Short machine-readable code.
     * @param message Human-readable detail.
     * @note Assumes the caller already holds `mutex_`.
     */
    void writeLocked(std::string_view code, std::string_view message);

    mutable std::mutex mutex_{};
    std::filesystem::path file_path_{};
    std::optional<std::ofstream> file_{};
    std::size_t error_count_ = 0;
    std::size_t input_error_count_ = 0;
};

} // namespace simulator
