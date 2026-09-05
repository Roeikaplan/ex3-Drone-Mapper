/**
 * @file PluginRegistry.cpp
 * @brief The lazy plugin lifecycle: first use maps a library, last use unmaps it, nothing reloads.
 * @note Every failure path here records the reason on the slot and returns. Nothing throws out of
 *       `acquire*`, because an unloadable plugin must degrade to a `-1` score rather than end the
 *       run - the caller turns a null factory into that score.
 */

#include <Simulator/PluginRegistry.h>

#include <Simulator/Registrar.h>

#include <optional>
#include <utility>

namespace simulator {

/**
 * @brief Describe a discovered file.
 * @param file Canonical path to the `.so`.
 * @param kind What the caller expects it to register.
 */
PluginSlot::PluginSlot(std::filesystem::path file, PluginKind kind)
    : file_(std::move(file)), kind_(kind) {}

/**
 * @brief Construct over the sinks failures and events are reported to.
 * @param logger Error sink.
 * @param lifecycle Load/unload audit trail.
 */
PluginRegistry::PluginRegistry(ErrorLogger& logger, PluginLifecycleLog& lifecycle)
    : logger_(logger), lifecycle_(lifecycle) {}

/**
 * @brief Close anything still mapped.
 * @note A safety net, not the mechanism. If this destructor ever has work to do, either no run
 *       needed that plugin or `main` skipped its explicit teardown - and the explicit teardown is
 *       what the ordering rules are written against, so it must stay.
 */
PluginRegistry::~PluginRegistry() {
    releaseAll();
}

/**
 * @brief Find the plugin files at a path and give each one a slot.
 * @param file_or_folder A single `.so`, or a folder to enumerate non-recursively.
 * @param kind The kind those files are expected to register.
 * @return The slots, in a stable order, plus any traversal failures.
 * @note Re-discovering a path returns the existing slot. That matters for the case where the same
 *       `.so` is named as the fixed plugin *and* sits in the varied folder: one file, one slot, one
 *       load - and one shared use count, so it unloads only when both roles are finished with it.
 */
PluginRegistry::Discovery PluginRegistry::discover(const std::filesystem::path& file_or_folder,
                                                   PluginKind kind) {
    Discovery discovery;
    const std::vector<std::filesystem::path> files =
        collectPluginFiles(file_or_folder, discovery.failures);

    const std::lock_guard<std::mutex> guard(load_mutex_);
    for (const std::filesystem::path& file : files) {
        PluginSlot* existing = nullptr;
        for (const std::unique_ptr<PluginSlot>& slot : slots_) {
            if (slot->file_ == file) {
                existing = slot.get();
                break;
            }
        }

        if (existing == nullptr) {
            slots_.push_back(std::make_unique<PluginSlot>(file, kind));
            discovery.slots.push_back(slots_.back().get());
            continue;
        }

        if (existing->kind_ != kind) {
            /**
             * @note One file cannot be both roles at once, and giving it a second slot would give it
             *       a second `dlopen`. It is reported against the role it was asked for *later*,
             *       which is the one it cannot fill.
             */
            discovery.failures.push_back(
                {file, std::string{"already in use as the "} + pluginKindName(existing->kind_) +
                           ", so it cannot also serve as the " + pluginKindName(kind)});
            continue;
        }

        discovery.slots.push_back(existing);
    }

    return discovery;
}

/**
 * @brief Take ownership of an algorithm factory that did not come from a library.
 * @param factory The factory to serve.
 * @return A slot already in the `Loaded` state.
 * @note The slot is born loaded, so `ensureLoaded` never touches it and no `dlopen` is ever
 *       attempted for it. The empty path is deliberate: there is no file to name.
 */
PluginSlot& PluginRegistry::adoptAlgorithm(common::MappingAlgorithmFactory factory) {
    const std::lock_guard<std::mutex> guard(load_mutex_);
    slots_.push_back(std::make_unique<PluginSlot>(std::filesystem::path{}, PluginKind::Algorithm));
    PluginSlot& slot = *slots_.back();
    slot.adopted_ = true;
    slot.algorithm_factory_ = std::move(factory);
    slot.state_.store(PluginSlot::State::Loaded, std::memory_order_release);
    return slot;
}

/**
 * @brief Take ownership of a mission-control factory that did not come from a library.
 * @param factory The factory to serve.
 * @return A slot already in the `Loaded` state.
 */
PluginSlot& PluginRegistry::adoptMissionControl(common::MissionControlFactory factory) {
    const std::lock_guard<std::mutex> guard(load_mutex_);
    slots_.push_back(
        std::make_unique<PluginSlot>(std::filesystem::path{}, PluginKind::MissionControl));
    PluginSlot& slot = *slots_.back();
    slot.adopted_ = true;
    slot.mission_control_factory_ = std::move(factory);
    slot.state_.store(PluginSlot::State::Loaded, std::memory_order_release);
    return slot;
}

/**
 * @brief Declare that more runs will need this plugin.
 * @param slot The plugin.
 * @param uses How many runs.
 * @note Relaxed ordering is enough: every reservation happens before dispatch, and starting a thread
 *       is itself a synchronisation point, so no worker can observe a count that is still being
 *       built up.
 */
void PluginRegistry::reserve(PluginSlot& slot, std::size_t uses) {
    slot.pending_uses_.fetch_add(uses, std::memory_order_relaxed);
}

/**
 * @brief Load a slot if it has never been attempted, and report whether a factory is available.
 * @param slot The plugin.
 * @return True when the slot is `Loaded`.
 * @note The fast path is the common one: after the first run of a plugin, every later run of it
 *       reads one atomic and takes no lock at all.
 */
bool PluginRegistry::ensureLoaded(PluginSlot& slot) {
    if (slot.state_.load(std::memory_order_acquire) != PluginSlot::State::NotLoaded) {
        return slot.state_.load(std::memory_order_acquire) == PluginSlot::State::Loaded;
    }

    const std::lock_guard<std::mutex> guard(load_mutex_);
    if (slot.state_.load(std::memory_order_relaxed) == PluginSlot::State::NotLoaded) {
        loadLocked(slot);
    }
    return slot.state_.load(std::memory_order_relaxed) == PluginSlot::State::Loaded;
}

/**
 * @brief Map the library and claim the one factory it is expected to register.
 * @param slot The plugin.
 * @note A library that opens but registers nothing usable is unmapped again *here*, before this
 *       function returns. The eager loader kept such handles until the end of the program; keeping
 *       them now would leave a useless mapping alive for the whole batch, which is precisely what
 *       this design exists to avoid.
 */
void PluginRegistry::loadLocked(PluginSlot& slot) {
    Registrar& registrar = Registrar::instance();

    slot.load_attempts_.fetch_add(1, std::memory_order_relaxed);

    const std::size_t algorithms_before = registrar.algorithmCount();
    const std::size_t mission_controls_before = registrar.missionControlCount();

    /**
     * @note The registration constructors these static initialisers call are declared in `common/`
     *       and defined only in this executable, and are resolved against it at `dlopen` time -
     *       which is why the simulator is built with `ENABLE_EXPORTS`.
     */
    PluginLibrary library{slot.file_};
    if (!library.valid()) {
        slot.failure_reason_ = library.error();
        slot.state_.store(PluginSlot::State::Failed, std::memory_order_release);
        logger_.log("PLUGIN_LOAD_FAILED", slot.file_.string() + ": " + slot.failure_reason_);
        lifecycle_.record("LOAD_FAILED", slot.file_, slot.failure_reason_);
        return;
    }

    const std::size_t new_algorithms = registrar.algorithmCount() - algorithms_before;
    const std::size_t new_mission_controls =
        registrar.missionControlCount() - mission_controls_before;

    const bool registered_algorithm = new_algorithms == 1;
    const bool wanted_algorithm = slot.kind_ == PluginKind::Algorithm;

    std::string rejection;
    if (new_algorithms + new_mission_controls != 1) {
        rejection = new_algorithms + new_mission_controls == 0 ? "loaded but registered nothing"
                                                               : "registered more than one factory";
    } else if (registered_algorithm != wanted_algorithm) {
        rejection = std::string{"registered a "} +
                    pluginKindName(registered_algorithm ? PluginKind::Algorithm
                                                        : PluginKind::MissionControl) +
                    " where a " + pluginKindName(slot.kind_) + " was expected";
    }

    if (!rejection.empty()) {
        /**
         * @note The strays are dropped while the library is still mapped. Their `std::function`
         *       targets are compiled into it, so destroying them after the `dlclose` below would
         *       jump into unmapped memory.
         */
        for (std::size_t i = 0; i < new_algorithms; ++i) {
            (void)registrar.takeLastAlgorithm();
        }
        for (std::size_t i = 0; i < new_mission_controls; ++i) {
            (void)registrar.takeLastMissionControl();
        }

        slot.failure_reason_ = std::move(rejection);
        library = PluginLibrary{};
        slot.state_.store(PluginSlot::State::Failed, std::memory_order_release);
        logger_.log("PLUGIN_LOAD_FAILED", slot.file_.string() + ": " + slot.failure_reason_);
        lifecycle_.record("LOAD_FAILED", slot.file_, slot.failure_reason_);
        return;
    }

    if (registered_algorithm) {
        std::optional<common::MappingAlgorithmFactory> factory = registrar.takeLastAlgorithm();
        slot.algorithm_factory_ = std::move(*factory);
    } else {
        std::optional<common::MissionControlFactory> factory = registrar.takeLastMissionControl();
        slot.mission_control_factory_ = std::move(*factory);
    }

    slot.library_ = std::move(library);

    /**
     * @note Released last, and it is the release that publishes the factory: a thread taking the
     *       lock-free fast path sees `Loaded` only once everything it is about to read is written.
     */
    slot.state_.store(PluginSlot::State::Loaded, std::memory_order_release);
    lifecycle_.record("LOAD", slot.file_, "");
}

/**
 * @brief Get this plugin's algorithm factory, loading the library if this is its first use.
 * @param slot The plugin.
 * @return The factory, or `nullptr` when it could not be loaded or is not an algorithm.
 */
const common::MappingAlgorithmFactory* PluginRegistry::acquireAlgorithm(PluginSlot& slot) {
    if (slot.kind_ != PluginKind::Algorithm || !ensureLoaded(slot)) {
        return nullptr;
    }
    return &slot.algorithm_factory_;
}

/**
 * @brief Get this plugin's mission-control factory, loading the library if this is its first use.
 * @param slot The plugin.
 * @return The factory, or `nullptr` when it could not be loaded or is not a mission control.
 */
const common::MissionControlFactory* PluginRegistry::acquireMissionControl(PluginSlot& slot) {
    if (slot.kind_ != PluginKind::MissionControl || !ensureLoaded(slot)) {
        return nullptr;
    }
    return &slot.mission_control_factory_;
}

/**
 * @brief Give back one reserved use, unloading the library if it was the last.
 * @param slot The plugin.
 * @note `acq_rel` on the decrement is what makes the unload safe: the thread that observes the count
 *       fall to zero synchronises with every other thread's release, so everything they did with the
 *       plugin happens-before the `dlclose`.
 * @note A slot that was never loaded, or that failed, decrements exactly the same way and simply
 *       finds nothing to unmap. Uniformity here is worth more than the branch it saves.
 */
void PluginRegistry::release(PluginSlot& slot) noexcept {
    if (slot.pending_uses_.fetch_sub(1, std::memory_order_acq_rel) != 1) {
        return;
    }

    const std::lock_guard<std::mutex> guard(load_mutex_);
    unloadLocked(slot);
}

/**
 * @brief Destroy this slot's factory and unmap its library.
 * @param slot The plugin.
 * @note Assigning an empty `std::function` destroys the old target - code inside the library - so it
 *       must happen before the handle is dropped. Both factory members are cleared regardless of
 *       kind: clearing an already-empty one costs nothing and removes a branch that could only ever
 *       be wrong.
 */
void PluginRegistry::unloadLocked(PluginSlot& slot) noexcept {
    if (slot.state_.load(std::memory_order_relaxed) != PluginSlot::State::Loaded) {
        return;
    }

    slot.algorithm_factory_ = {};
    slot.mission_control_factory_ = {};
    slot.library_ = PluginLibrary{};

    slot.state_.store(PluginSlot::State::Unloaded, std::memory_order_release);

    /**
     * @note An adopted factory had no library behind it, so there is no unload to report. Logging
     *       one would put a file-less `UNLOAD` line in the audit trail with no matching `LOAD`.
     */
    if (!slot.adopted_) {
        lifecycle_.record("UNLOAD", slot.file_, "");
    }
}

/**
 * @brief Unload anything still mapped, whatever its outstanding use count.
 */
void PluginRegistry::releaseAll() noexcept {
    const std::lock_guard<std::mutex> guard(load_mutex_);
    for (const std::unique_ptr<PluginSlot>& slot : slots_) {
        unloadLocked(*slot);
    }
}

/**
 * @brief How many distinct plugin files have been discovered.
 * @return The slot count.
 */
std::size_t PluginRegistry::discoveredCount() const noexcept {
    const std::lock_guard<std::mutex> guard(load_mutex_);
    return slots_.size();
}

/**
 * @brief How many distinct plugin files were ever successfully loaded.
 * @return The count, including those since unloaded - which is the interesting number, since under
 *         this design almost everything has been unloaded again by the time anyone asks.
 */
std::size_t PluginRegistry::loadedCount() const noexcept {
    const std::lock_guard<std::mutex> guard(load_mutex_);
    std::size_t loaded = 0;
    for (const std::unique_ptr<PluginSlot>& slot : slots_) {
        const PluginSlot::State state = slot->state_.load(std::memory_order_relaxed);
        if (!slot->adopted_ &&
            (state == PluginSlot::State::Loaded || state == PluginSlot::State::Unloaded)) {
            ++loaded;
        }
    }
    return loaded;
}

} // namespace simulator
