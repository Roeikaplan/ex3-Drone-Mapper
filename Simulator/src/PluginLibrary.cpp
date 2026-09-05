/**
 * @file PluginLibrary.cpp
 * @brief The only translation unit in the project that calls `dlopen` and `dlclose`.
 * @note Both calls may now happen on a worker thread rather than only on the main thread, because
 *       a library is mapped by the first run that needs it and unmapped by the last one to finish
 *       with it. `dlopen`/`dlclose` are themselves thread-safe; what is *not* safe concurrently is
 *       the registrar's load-then-claim, which is why `PluginRegistry` serialises the loads.
 */

#include <Simulator/PluginLibrary.h>

#include <dlfcn.h>

#include <atomic>
#include <utility>

namespace simulator {
namespace {

/**
 * @brief The process-wide loader counters behind `pluginLibraryStats`.
 * @note Namespace-scope atomics rather than members: the interesting quantity is how many libraries
 *       are mapped *at once*, which no single instance can observe.
 */
std::atomic<std::size_t> g_opens{0};
std::atomic<std::size_t> g_closes{0};
std::atomic<std::size_t> g_currently_open{0};
std::atomic<std::size_t> g_peak_open{0};

/**
 * @brief Record a successful load and update the high-water mark.
 * @note The compare-exchange loop is the standard atomic-max: `peak` only ever moves up, and a
 *       racing thread that already published a higher value wins without either being lost.
 */
void countOpen() noexcept {
    g_opens.fetch_add(1, std::memory_order_relaxed);
    const std::size_t live = g_currently_open.fetch_add(1, std::memory_order_relaxed) + 1;

    std::size_t peak = g_peak_open.load(std::memory_order_relaxed);
    while (peak < live &&
           !g_peak_open.compare_exchange_weak(peak, live, std::memory_order_relaxed)) {
    }
}

/**
 * @brief Record an unload.
 */
void countClose() noexcept {
    g_closes.fetch_add(1, std::memory_order_relaxed);
    g_currently_open.fetch_sub(1, std::memory_order_relaxed);
}

} // namespace

/**
 * @brief Read the process-wide loader counters.
 * @return The current snapshot.
 */
PluginLibraryStats pluginLibraryStats() noexcept {
    PluginLibraryStats stats{};
    stats.opens = g_opens.load(std::memory_order_relaxed);
    stats.closes = g_closes.load(std::memory_order_relaxed);
    stats.currently_open = g_currently_open.load(std::memory_order_relaxed);
    stats.peak_open = g_peak_open.load(std::memory_order_relaxed);
    return stats;
}

/**
 * @brief Zero the process-wide loader counters.
 */
void resetPluginLibraryStats() noexcept {
    g_opens.store(0, std::memory_order_relaxed);
    g_closes.store(0, std::memory_order_relaxed);
    g_currently_open.store(0, std::memory_order_relaxed);
    g_peak_open.store(0, std::memory_order_relaxed);
}

PluginLibrary::PluginLibrary(std::filesystem::path file) : file_(std::move(file)) {
    /**
     * @note `dlerror()` is only meaningful immediately after a failed `dl*` call and it latches
     *       until read. Clearing it first stops a previous failure being misreported as ours.
     */
    dlerror();

    handle_ = dlopen(file_.c_str(), RTLD_NOW | RTLD_LOCAL);
    if (handle_ == nullptr) {
        const char* message = dlerror();
        error_ = message != nullptr ? message : "dlopen failed without a diagnostic";
        return;
    }

    countOpen();
}

PluginLibrary::~PluginLibrary() {
    close();
}

PluginLibrary::PluginLibrary(PluginLibrary&& other) noexcept
    : file_(std::move(other.file_)), handle_(other.handle_), error_(std::move(other.error_)) {
    /**
     * @note Clearing the source handle is what keeps the close count at one. Without it, moving a
     *       library into its slot would `dlclose` it the moment the temporary died.
     */
    other.handle_ = nullptr;
}

PluginLibrary& PluginLibrary::operator=(PluginLibrary&& other) noexcept {
    if (this != &other) {
        close();
        file_ = std::move(other.file_);
        handle_ = other.handle_;
        error_ = std::move(other.error_);
        other.handle_ = nullptr;
    }
    return *this;
}

void PluginLibrary::close() noexcept {
    if (handle_ != nullptr) {
        /**
         * @note The return value is deliberately ignored. A failing `dlclose` leaves the library
         *       mapped, which is harmless here, and there is nothing useful to do about it during
         *       teardown - whereas throwing or aborting would violate the never-crash rule.
         */
        dlclose(handle_);
        handle_ = nullptr;
        countClose();
    }
}

} // namespace simulator
