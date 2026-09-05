/**
 * @file PluginLifecycleLog.h
 * @brief The audit trail for when each plugin library was mapped and unmapped.
 */

#pragma once

#include <filesystem>
#include <fstream>
#include <mutex>
#include <optional>
#include <string_view>

namespace simulator {

/**
 * @brief Records every `dlopen` and `dlclose` as it happens, with the live mapped count.
 *
 * The lazy plugin lifecycle - load on first use, unload after the last run that needs it, never
 * reload - is invisible in the results it produces. This log is what makes it checkable: one line
 * per event, in the order the events happened, naming the thread that caused each one.
 *
 * @note Architectural boundary: this is a *reporting* sink, not part of the mechanism. Nothing in
 *       `PluginRegistry` reads it back, and a run with no log file behaves identically - which is
 *       why the null-sink constructor exists rather than the registry taking a pointer.
 * @note Thread-safety posture: synchronised, for the same reason `ErrorLogger` is. Events are
 *       produced from worker threads, and a torn line would destroy the ordering the log exists to
 *       show. It joins the error log and `std::cout` as the third deliberately locked shared object.
 * @note Neither copyable nor movable: it owns a mutex and an open stream, and is always passed by
 *       reference.
 */
class PluginLifecycleLog {
public:
    /**
     * @brief Construct a log that discards everything.
     * @note Exists so component tests can drive the registry without touching the filesystem.
     */
    PluginLifecycleLog() = default;

    /**
     * @brief Construct a log that appends to a file.
     * @param log_file Destination; missing parent directories are created.
     * @note Best effort, exactly like `ErrorLogger`: a file that cannot be opened degrades to a
     *       silent sink rather than throwing. Losing the audit trail must never cost the run.
     */
    explicit PluginLifecycleLog(std::filesystem::path log_file);

    PluginLifecycleLog(const PluginLifecycleLog&) = delete;
    PluginLifecycleLog& operator=(const PluginLifecycleLog&) = delete;
    PluginLifecycleLog(PluginLifecycleLog&&) = delete;
    PluginLifecycleLog& operator=(PluginLifecycleLog&&) = delete;

    /**
     * @brief Record one lifecycle event.
     * @param event What happened: `LOAD`, `LOAD_FAILED`, or `UNLOAD`.
     * @param file The plugin the event concerns.
     * @param detail Extra context, such as a loader diagnostic; may be empty.
     * @note The live mapped count is read from `pluginLibraryStats()` *inside the lock*, so the
     *       number on a line always reflects the state right after that line's event rather than
     *       whatever a racing thread did next.
     */
    void record(std::string_view event, const std::filesystem::path& file, std::string_view detail);

    /**
     * @brief Write the closing summary of the whole run.
     * @param files_discovered How many distinct `.so` files the registry knew about.
     * @param files_loaded How many of them were ever actually mapped.
     * @note Written into the log as well as printed by `main`, so the file stands on its own as
     *       evidence without the terminal transcript beside it.
     */
    void recordSummary(std::size_t files_discovered, std::size_t files_loaded);

    /**
     * @brief The file this log writes to.
     * @return The configured path, or an empty path for a discarding log.
     */
    [[nodiscard]] const std::filesystem::path& file() const noexcept { return file_path_; }

private:
    mutable std::mutex mutex_{};
    std::filesystem::path file_path_{};
    std::optional<std::ofstream> file_{};
};

} // namespace simulator
