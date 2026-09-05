/**
 * @file PluginLibrary.h
 * @brief RAII ownership of a single dynamically loaded plugin.
 */

#pragma once

#include <cstddef>
#include <filesystem>
#include <string>

namespace simulator {

/**
 * @brief A snapshot of every `dlopen`/`dlclose` the process has performed.
 *
 * @note These counters exist to make the lazy plugin lifecycle *observable*. "Loaded once, unloaded
 *       when no longer needed" is a claim about behaviour that leaves no trace in the results, so
 *       without a count there is nothing for a test - or a reader of `plugin_lifecycle.log` - to
 *       check. `peak_open` is the interesting one: under the eager scheme it equalled the number of
 *       plugins in the folder, and under this one it is bounded by the number of live threads.
 */
struct PluginLibraryStats {
    /// Successful `dlopen` calls since the last reset.
    std::size_t opens = 0;

    /// `dlclose` calls since the last reset.
    std::size_t closes = 0;

    /// Libraries currently mapped.
    std::size_t currently_open = 0;

    /// The largest value `currently_open` ever reached.
    std::size_t peak_open = 0;
};

/**
 * @brief Process-wide loader counters.
 * @return A consistent-enough snapshot for reporting; each field is read atomically.
 * @note Safe to call from any thread. The fields are read one at a time, so a snapshot taken while
 *       loading is in flight can be internally skewed by one - which is why the summary line is
 *       printed after every worker has joined.
 */
[[nodiscard]] PluginLibraryStats pluginLibraryStats() noexcept;

/**
 * @brief Zero the process-wide loader counters.
 * @note For tests, which need a known starting point. Never called by the simulator itself.
 */
void resetPluginLibraryStats() noexcept;

/**
 * @brief Owns one `dlopen` handle and closes it exactly once.
 *
 * Construction attempts the load and records `dlerror()` on failure rather than throwing, so a
 * broken plugin degrades into a reportable failure instead of aborting the batch.
 *
 * @note Architectural boundary: `dlclose` happens **only** in the destructor, and the owning
 *       `PluginSlot` must not let that destructor run until every plugin-derived object *and* the
 *       factory that library registered is gone. Unmapping a library whose `std::function` targets
 *       are still alive crashes during static destruction, after `main` has already returned.
 * @note Move-only. A moved-from instance holds no handle, which is what keeps the close count at
 *       exactly one when these are moved into a slot.
 */
class PluginLibrary {
public:
    /**
     * @brief Construct without loading anything.
     * @note A slot holds one of these from the moment it is created until the run that needs it
     *       starts, so the empty state is the normal state for most of a slot's life. It is not a
     *       moved-from carcass.
     */
    PluginLibrary() = default;

    /**
     * @brief Load a shared object.
     * @param file Path to the `.so`; kept for diagnostics and for pairing with its factory.
     * @note Uses `RTLD_NOW | RTLD_LOCAL`. `RTLD_NOW` surfaces an unresolvable symbol here, where it
     *       can be contained, instead of mid-run on first call. `RTLD_LOCAL` keeps the plugin's
     *       globals out of the global symbol namespace so two teams' identically named classes
     *       cannot collide or interpose - the property the lowercase-namespace decision relies on.
     */
    explicit PluginLibrary(std::filesystem::path file);

    /**
     * @brief Close the handle if one is held.
     * @note The only place `dlclose` is ever called.
     */
    ~PluginLibrary();

    PluginLibrary(PluginLibrary&& other) noexcept;
    PluginLibrary& operator=(PluginLibrary&& other) noexcept;
    PluginLibrary(const PluginLibrary&) = delete;
    PluginLibrary& operator=(const PluginLibrary&) = delete;

    /**
     * @brief Whether the library loaded.
     * @return True when a live handle is held.
     */
    [[nodiscard]] bool valid() const noexcept { return handle_ != nullptr; }

    /**
     * @brief The loader diagnostic captured when loading failed.
     * @return The `dlerror()` text, or an empty string on success.
     * @note This message is the entire diagnosis for the most common plugin failure - an
     *       unexported registration constructor - so callers must surface it verbatim.
     */
    [[nodiscard]] const std::string& error() const noexcept { return error_; }

    /**
     * @brief The file this library was loaded from.
     * @return The path as supplied at construction.
     */
    [[nodiscard]] const std::filesystem::path& file() const noexcept { return file_; }

private:
    /**
     * @brief Close and forget the handle.
     * @note Idempotent, so the destructor and the move-assignment operator can share it.
     */
    void close() noexcept;

    std::filesystem::path file_{};
    void* handle_ = nullptr;
    std::string error_{};
};

} // namespace simulator
