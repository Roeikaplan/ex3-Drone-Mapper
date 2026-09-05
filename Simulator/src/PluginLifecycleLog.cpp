/**
 * @file PluginLifecycleLog.cpp
 * @brief Formatting and synchronised writing of plugin load/unload events.
 */

#include <Simulator/PluginLifecycleLog.h>

#include <Simulator/PluginLibrary.h>
#include <Simulator/UtcTime.h>

#include <sstream>
#include <system_error>
#include <thread>
#include <utility>

namespace simulator {

/**
 * @brief Construct a log that appends to a file.
 * @param log_file Destination; missing parent directories are created.
 * @note Append rather than truncate, matching `ErrorLogger`, so a results directory that is reused
 *       keeps both transcripts rather than one of them silently replacing the other.
 */
PluginLifecycleLog::PluginLifecycleLog(std::filesystem::path log_file)
    : file_path_(std::move(log_file)) {
    std::error_code ec;
    if (file_path_.has_parent_path()) {
        std::filesystem::create_directories(file_path_.parent_path(), ec);
    }

    std::ofstream stream(file_path_, std::ios::app);
    if (stream) {
        file_ = std::move(stream);
    }
}

/**
 * @brief Record one lifecycle event.
 * @param event What happened: `LOAD`, `LOAD_FAILED`, or `UNLOAD`.
 * @param file The plugin the event concerns.
 * @param detail Extra context, such as a loader diagnostic; may be empty.
 * @note The filename rather than the full path: the reports name plugins that way, and a reader
 *       correlating this log against `competitive_report.yaml` should not have to.
 * @note `std::endl` flushes, which is the whole point - a log that survives only if the process
 *       exits cleanly is no use for diagnosing the case where it does not.
 */
void PluginLifecycleLog::record(std::string_view event, const std::filesystem::path& file,
                                std::string_view detail) {
    const std::lock_guard<std::mutex> guard(mutex_);
    if (!file_) {
        return;
    }

    std::ostringstream thread_id;
    thread_id << std::this_thread::get_id();

    const PluginLibraryStats stats = pluginLibraryStats();

    *file_ << utcIso8601() << " thread=" << thread_id.str() << " " << event << " "
           << file.filename().string() << " mapped=" << stats.currently_open;
    if (!detail.empty()) {
        *file_ << " (" << detail << ")";
    }
    *file_ << std::endl;
}

/**
 * @brief Write the closing summary of the whole run.
 * @param files_discovered How many distinct `.so` files the registry knew about.
 * @param files_loaded How many of them were ever actually mapped.
 * @note `opens == files_loaded` is the "loaded once" property stated as an equation, and
 *       `mapped_at_end == 0` is "unloaded when no longer needed". Both are written out rather than
 *       left for the reader to derive from the event lines.
 */
void PluginLifecycleLog::recordSummary(std::size_t files_discovered, std::size_t files_loaded) {
    const std::lock_guard<std::mutex> guard(mutex_);
    if (!file_) {
        return;
    }

    const PluginLibraryStats stats = pluginLibraryStats();
    *file_ << "summary discovered=" << files_discovered << " loaded=" << files_loaded
           << " dlopen=" << stats.opens << " dlclose=" << stats.closes
           << " peak_mapped=" << stats.peak_open << " mapped_at_end=" << stats.currently_open
           << std::endl;
}

} // namespace simulator
