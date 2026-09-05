/**
 * @file PluginRegistry.h
 * @brief Load-on-first-use, unload-after-last-use, never-reload ownership of plugin libraries.
 */

#pragma once

#include <Common/MappingAlgorithmFactory.h>
#include <Common/MissionControlFactory.h>
#include <Simulator/ErrorLogger.h>
#include <Simulator/PluginDiscovery.h>
#include <Simulator/PluginLibrary.h>
#include <Simulator/PluginLifecycleLog.h>

#include <atomic>
#include <cstddef>
#include <filesystem>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

namespace simulator {

/**
 * @brief Thrown when a run cannot be built because one of its plugins could not be loaded.
 *
 * @note A distinct type so the failure can be scored without being re-reported. The registry logs a
 *       load failure once, at the moment it happens; every run of that plugin then fails for the
 *       same reason, and logging all of them would turn one real error into dozens of identical
 *       lines and inflate the run's error count accordingly.
 */
class PluginUnavailable : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

/**
 * @brief One plugin file and everything the registry knows about it.
 *
 * A slot exists from discovery until the program ends, but the **library it names is mapped only
 * between the first run that needs it and the last run that finishes with it**.
 *
 * @note Architectural boundary: the state machine is deliberately one-way -
 *       `NotLoaded -> Loaded -> Unloaded` or `NotLoaded -> Failed`, with no edge back to
 *       `NotLoaded`. That is what makes "never load the same library twice" a structural property
 *       rather than something the scheduler happens not to do. A failure is just as sticky as a
 *       success: a `.so` that could not be loaded is never retried either.
 * @note `pending_uses_` is the whole trick. The task table is fully known before dispatch, so the
 *       exact number of runs that will ever need this file is countable in advance. When it reaches
 *       zero, no future run can possibly need the library - so unloading is safe by construction,
 *       with no scheduling assumption and no risk of having to reload.
 * @note Only `PluginRegistry` may mutate a slot; everything here is deliberately read-only to the
 *       outside. The registry's mutex is what orders loads against the registrar's claim.
 */
class PluginSlot {
public:
    /**
     * @brief What has happened to this file so far.
     */
    enum class State {
        NotLoaded, ///< Discovered; never yet mapped.
        Loaded,    ///< Mapped, with a usable factory claimed.
        Failed,    ///< A load was attempted once and did not yield a usable factory.
        Unloaded   ///< Was loaded, its last user finished, and it has been `dlclose`d.
    };

    /**
     * @brief Describe a discovered file.
     * @param file Canonical path to the `.so`.
     * @param kind What the caller expects it to register.
     */
    PluginSlot(std::filesystem::path file, PluginKind kind);

    PluginSlot(const PluginSlot&) = delete;
    PluginSlot& operator=(const PluginSlot&) = delete;
    PluginSlot(PluginSlot&&) = delete;
    PluginSlot& operator=(PluginSlot&&) = delete;

    /**
     * @brief The file this slot names.
     * @return The canonical path.
     */
    [[nodiscard]] const std::filesystem::path& file() const noexcept { return file_; }

    /**
     * @brief What this slot is expected to register.
     * @return The expected kind.
     */
    [[nodiscard]] PluginKind kind() const noexcept { return kind_; }

    /**
     * @brief Where this file stands.
     * @return The current state.
     * @note Safe to read from any thread; the value is published with release/acquire ordering
     *       against the fields the loader filled in.
     */
    [[nodiscard]] State state() const noexcept { return state_.load(std::memory_order_acquire); }

    /**
     * @brief Whether a load was attempted and failed.
     * @return True when this plugin can never provide a factory.
     */
    [[nodiscard]] bool failed() const noexcept { return state() == State::Failed; }

    /**
     * @brief Why the load failed.
     * @return The diagnostic, or an empty string when no load has failed.
     * @note Only meaningful once `failed()` is true, which is also the point at which the string is
     *       guaranteed visible to the reading thread.
     */
    [[nodiscard]] const std::string& failureReason() const noexcept { return failure_reason_; }

    /**
     * @brief How many `dlopen` attempts this file has seen.
     * @return 0 or 1, always - anything else is the bug this counter exists to catch.
     */
    [[nodiscard]] std::size_t loadAttempts() const noexcept {
        return load_attempts_.load(std::memory_order_relaxed);
    }

    /**
     * @brief Whether this slot's factory came from somewhere other than a shared library.
     * @return True for an adopted factory, which has no handle and nothing to unmap.
     */
    [[nodiscard]] bool adopted() const noexcept { return adopted_; }

    /**
     * @brief How many reserved uses have not yet been released.
     * @return The outstanding count.
     */
    [[nodiscard]] std::size_t pendingUses() const noexcept {
        return pending_uses_.load(std::memory_order_relaxed);
    }

private:
    friend class PluginRegistry;

    std::filesystem::path file_{};
    PluginKind kind_ = PluginKind::Algorithm;

    std::atomic<State> state_{State::NotLoaded};
    std::atomic<std::size_t> pending_uses_{0};
    std::atomic<std::size_t> load_attempts_{0};
    bool adopted_ = false;

    /**
     * @note Declared **after** the factories on purpose. Members are destroyed in reverse
     *       declaration order, so if a slot is ever destroyed while still loaded, the factories -
     *       whose targets live in the library's code - die before the `dlclose`. The registry
     *       unloads explicitly and never relies on this, but the layout should not contradict the
     *       rule it enforces.
     */
    common::MappingAlgorithmFactory algorithm_factory_{};
    common::MissionControlFactory mission_control_factory_{};
    PluginLibrary library_{};

    std::string failure_reason_{};
};

/**
 * @brief Owns every plugin slot and performs the only loads and unloads in the program.
 *
 * This replaces the eager loader that mapped every `.so` in the folder up front. The lifecycle is
 * now: discover files (no loading), reserve one use per run that will need each file, load a library
 * the first time a run actually asks for its factory, and unload it the moment its last run
 * finishes - which typically happens on a worker thread, mid-batch.
 *
 * @note Architectural boundary: **the load path is serialised by `load_mutex_`, and it must be.**
 *       The file-to-factory association is inferred temporally - the registrar's factory count
 *       before a `dlopen` versus after - so exactly one load may be in flight at a time. The
 *       serialisation moved here from "everything happens on the main thread"; the inference itself
 *       is unchanged.
 * @note Thread-safety posture: `acquire` takes the mutex only on a slot's *first* use and reads a
 *       relaxed-fast-path atomic afterwards, so the steady state costs one atomic load per run. That
 *       is the "do not lock if you can avoid it" rule applied where it actually matters.
 * @note A factory pointer handed out by `acquire` stays valid only while the caller holds a use. It
 *       does, by construction: a run releases its use in a destructor that runs after the run - and
 *       therefore after every plugin instance - is gone.
 */
class PluginRegistry {
public:
    /**
     * @brief Everything a discovery pass found.
     */
    struct Discovery {
        /// One slot per usable file, in canonical sorted order. Non-owning; the registry owns them.
        std::vector<PluginSlot*> slots{};

        /// Paths that could not even be enumerated. Per-file load failures are not these.
        std::vector<PluginFailure> failures{};
    };

    /**
     * @brief Construct over the sinks failures and events are reported to.
     * @param logger Error sink; must outlive this object.
     * @param lifecycle Load/unload audit trail; must outlive this object.
     */
    PluginRegistry(ErrorLogger& logger, PluginLifecycleLog& lifecycle);

    /**
     * @brief Close anything still mapped.
     * @note Normally finds nothing to do - every slot has already unloaded itself. It matters for
     *       the paths where runs never happened at all.
     */
    ~PluginRegistry();

    PluginRegistry(const PluginRegistry&) = delete;
    PluginRegistry& operator=(const PluginRegistry&) = delete;
    PluginRegistry(PluginRegistry&&) = delete;
    PluginRegistry& operator=(PluginRegistry&&) = delete;

    /**
     * @brief Find the plugin files at a path and give each one a slot.
     * @param file_or_folder A single `.so`, or a folder to enumerate non-recursively.
     * @param kind The kind those files are expected to register.
     * @return The slots, in a stable order, plus any traversal failures.
     * @note **Loads nothing.** This is what lets the whole task table be built before a single
     *       library is mapped.
     * @note A path already discovered returns the *same* slot rather than a second one, so a file
     *       reachable twice - as the fixed plugin and again inside the varied folder, or as a
     *       symlink beside its target - is still loaded once and unloaded once.
     * @note A file already discovered under the *other* kind is reported as a failure instead: one
     *       library cannot fill both roles, and giving it a second slot would give it a second
     *       `dlopen`.
     */
    [[nodiscard]] Discovery discover(const std::filesystem::path& file_or_folder, PluginKind kind);

    /**
     * @brief Take ownership of an algorithm factory that did not come from a library.
     * @param factory The factory to serve.
     * @return A slot already in the `Loaded` state.
     * @note For factories with no `.so` behind them - the component tests wire fake plugins this way,
     *       so that they exercise the same acquire path production does instead of a parallel one.
     *       There is no handle, so releasing such a slot unmaps nothing.
     */
    PluginSlot& adoptAlgorithm(common::MappingAlgorithmFactory factory);

    /**
     * @brief Take ownership of a mission-control factory that did not come from a library.
     * @param factory The factory to serve.
     * @return A slot already in the `Loaded` state.
     * @note As `adoptAlgorithm`.
     */
    PluginSlot& adoptMissionControl(common::MissionControlFactory factory);

    /**
     * @brief Declare that @p uses more runs will need this plugin.
     * @param slot The plugin.
     * @param uses How many runs; may be zero.
     * @note Must be called for every run before dispatch. An under-count unloads a library while a
     *       later run still needs it - which, because reloading is forbidden, turns into that run
     *       failing rather than into a quiet reload.
     */
    void reserve(PluginSlot& slot, std::size_t uses);

    /**
     * @brief Get this plugin's algorithm factory, loading the library if this is its first use.
     * @param slot The plugin.
     * @return The factory, or `nullptr` when the plugin could not be loaded or is not an algorithm.
     * @note The caller must hold a reserved use for @p slot; the returned pointer is valid for
     *       exactly as long as that use is.
     */
    [[nodiscard]] const common::MappingAlgorithmFactory* acquireAlgorithm(PluginSlot& slot);

    /**
     * @brief Get this plugin's mission-control factory, loading the library if this is its first use.
     * @param slot The plugin.
     * @return The factory, or `nullptr` when the plugin could not be loaded or is not a mission
     *         control.
     * @note As `acquireAlgorithm`: valid only while the caller's use is held.
     */
    [[nodiscard]] const common::MissionControlFactory* acquireMissionControl(PluginSlot& slot);

    /**
     * @brief Give back one reserved use, unloading the library if it was the last.
     * @param slot The plugin.
     * @note Called from worker threads, from a destructor, so it never throws. This is where every
     *       mid-batch `dlclose` originates.
     */
    void release(PluginSlot& slot) noexcept;

    /**
     * @brief Unload anything still mapped, whatever its outstanding use count.
     * @note The final sweep, on the main thread, for slots whose runs never happened. Must not be
     *       called while any worker is running.
     */
    void releaseAll() noexcept;

    /**
     * @brief How many distinct plugin files have been discovered.
     * @return The slot count.
     */
    [[nodiscard]] std::size_t discoveredCount() const noexcept;

    /**
     * @brief How many distinct plugin files were ever successfully loaded.
     * @return The count, which the loader's `dlopen` total must match once failed loads are
     *         accounted for.
     */
    [[nodiscard]] std::size_t loadedCount() const noexcept;

private:
    /**
     * @brief Map the library and claim the one factory it is expected to register.
     * @param slot The plugin; must be `NotLoaded`, and the caller must hold `load_mutex_`.
     * @note This is the load-then-claim pattern: record the registrar's counts, `dlopen` exactly one
     *       file, and require the delta to be exactly one factory of the expected kind. Anything
     *       else is dropped *while the library is still mapped*, so a stray factory can neither be
     *       attributed to the next plugin loaded nor outlive the code it points into.
     */
    void loadLocked(PluginSlot& slot);

    /**
     * @brief Load @p slot if it has never been attempted, and report whether a factory is available.
     * @param slot The plugin.
     * @return True when the slot is `Loaded`.
     * @note Double-checked: a relaxed-acquire read outside the lock, re-read inside it. The check is
     *       sound here because the state is an atomic, published after the factory it describes.
     */
    [[nodiscard]] bool ensureLoaded(PluginSlot& slot);

    /**
     * @brief Destroy this slot's factory and unmap its library.
     * @param slot The plugin; the caller must hold `load_mutex_`.
     * @note The ordering inside is the same ordering `main` performs for the whole program, applied
     *       to one file: the factory dies first, then the library. Reversing it unmaps the code the
     *       `std::function`'s destructor is about to run.
     */
    void unloadLocked(PluginSlot& slot) noexcept;

    ErrorLogger& logger_;
    PluginLifecycleLog& lifecycle_;

    /**
     * @note Guards loading, unloading, and the slot table itself. One mutex rather than one per
     *       slot, because the registrar's count-delta claim is global state: two slots loading at
     *       once would each see the other's registration.
     */
    mutable std::mutex load_mutex_{};

    /**
     * @note `unique_ptr` because slots are handed out by pointer and must not move when the vector
     *       grows. They are also non-copyable and non-movable, holding an atomic and a live handle.
     */
    std::vector<std::unique_ptr<PluginSlot>> slots_{};
};

} // namespace simulator
