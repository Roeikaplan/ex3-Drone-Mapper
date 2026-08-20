/**
 * @file Registrar.h
 * @brief The Simulator-side destination for factories published by plugins at load time.
 * @note Paired with `RegistrationEntryPoints.cpp`, which defines the frozen registration
 *       constructors declared in `common/` and forwards them here.
 */

#pragma once

#include <Common/MappingAlgorithmFactory.h>
#include <Common/MissionControlFactory.h>

#include <cstddef>
#include <optional>
#include <vector>

namespace simulator {

/**
 * @brief Receives plugin factories from the frozen registration constructors.
 *
 * A plugin's `REGISTER_*` macro creates a global object whose constructor runs during `dlopen`.
 * That constructor is declared in `common/` and takes only a factory: no context parameter, no
 * `this` to route the call through. The destination must therefore be reachable from a free
 * function, which forces a singleton - this is not a stylistic choice.
 *
 * @note Deliberately unsynchronised. Every plugin is loaded serially on the main thread before
 *       any worker exists, which keeps the whole registration path lock-free.
 * @note `clear()` must run before any `PluginLibrary` is destroyed. The stored `std::function`
 *       objects hold targets whose code lives in the plugin's mapping, so destroying them after
 *       `dlclose` jumps into unmapped memory - and because this singleton outlives `main`, no
 *       scope-based ordering can prevent that.
 */
class Registrar {
public:
    /**
     * @brief The single instance.
     * @return A reference valid for the whole program.
     * @note Function-local static: constructed on first use, which is inside the first `dlopen`
     *       and therefore long after the executable's own statics are initialised. No
     *       static-initialisation-order hazard, and thread-safe initialisation for free.
     */
    [[nodiscard]] static Registrar& instance();

    /**
     * @brief Record a mapping-algorithm factory.
     * @param factory Callable that builds one `IMappingAlgorithm` from its dependencies.
     * @note Called from a plugin's static initialiser during `dlopen`, never by our own code.
     *       The factory's callable target lives in the plugin's code segment.
     */
    void addAlgorithm(common::MappingAlgorithmFactory factory);

    /**
     * @brief Record a mission-control factory.
     * @param factory Callable that builds one `IMissionControl` from its dependencies.
     * @note As `addAlgorithm`: invoked only from a plugin's static initialiser.
     */
    void addMissionControl(common::MissionControlFactory factory);

    /**
     * @brief How many algorithm factories are currently held.
     * @return The count, used by the loader to detect what a single `dlopen` registered.
     */
    [[nodiscard]] std::size_t algorithmCount() const noexcept;

    /**
     * @brief How many mission-control factories are currently held.
     * @return The count, used by the loader to detect what a single `dlopen` registered.
     */
    [[nodiscard]] std::size_t missionControlCount() const noexcept;

    /**
     * @brief Remove and return the most recently registered algorithm factory.
     * @return The factory, or `nullopt` when none is held.
     * @note This is the "claim" half of the load-then-claim pattern: the loader dlopens exactly
     *       one file, observes exactly one new factory, and takes it.
     */
    [[nodiscard]] std::optional<common::MappingAlgorithmFactory> takeLastAlgorithm();

    /**
     * @brief Remove and return the most recently registered mission-control factory.
     * @return The factory, or `nullopt` when none is held.
     * @note The claim half of load-then-claim; see `takeLastAlgorithm`.
     */
    [[nodiscard]] std::optional<common::MissionControlFactory> takeLastMissionControl();

    /**
     * @brief Drop every factory still held.
     * @note Mandatory teardown step. See the class note: this must happen before `dlclose`.
     */
    void clear() noexcept;

    Registrar(const Registrar&) = delete;
    Registrar& operator=(const Registrar&) = delete;
    Registrar(Registrar&&) = delete;
    Registrar& operator=(Registrar&&) = delete;

private:
    Registrar() = default;
    ~Registrar() = default;

    std::vector<common::MappingAlgorithmFactory> algorithms_{};
    std::vector<common::MissionControlFactory> mission_controls_{};
};

} // namespace simulator
