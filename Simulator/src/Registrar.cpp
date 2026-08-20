/**
 * @file Registrar.cpp
 * @brief Storage and claim operations for factories published by plugins.
 * @note Deliberately unsynchronised: every entry point here runs during serial, main-thread
 *       plugin loading, so adding a mutex would guard nothing.
 */

#include <Simulator/Registrar.h>

#include <utility>

namespace simulator {

Registrar& Registrar::instance() {
    /**
     * @note Meyers singleton: initialised on first use, which is inside the first `dlopen` and
     *       therefore after every namespace-scope static in this executable already exists. A
     *       namespace-scope instance would carry static-initialisation-order risk for no benefit.
     */
    static Registrar registrar;
    return registrar;
}

void Registrar::addAlgorithm(common::MappingAlgorithmFactory factory) {
    algorithms_.push_back(std::move(factory));
}

void Registrar::addMissionControl(common::MissionControlFactory factory) {
    mission_controls_.push_back(std::move(factory));
}

std::size_t Registrar::algorithmCount() const noexcept {
    return algorithms_.size();
}

std::size_t Registrar::missionControlCount() const noexcept {
    return mission_controls_.size();
}

std::optional<common::MappingAlgorithmFactory> Registrar::takeLastAlgorithm() {
    if (algorithms_.empty()) {
        return std::nullopt;
    }
    /**
     * @note Taken from the back because the loader claims immediately after a single `dlopen`,
     *       so the factory it is looking for is always the most recently appended one.
     */
    common::MappingAlgorithmFactory factory = std::move(algorithms_.back());
    algorithms_.pop_back();
    return std::make_optional(std::move(factory));
}

std::optional<common::MissionControlFactory> Registrar::takeLastMissionControl() {
    if (mission_controls_.empty()) {
        return std::nullopt;
    }
    common::MissionControlFactory factory = std::move(mission_controls_.back());
    mission_controls_.pop_back();
    return std::make_optional(std::move(factory));
}

void Registrar::clear() noexcept {
    /**
     * @note Destroying a `std::function` runs its target's manager function, which is code inside
     *       the plugin's mapping. Every caller must reach this point while the libraries are still
     *       loaded - see the teardown ordering in `PluginSmokeCheck.cpp`.
     */
    algorithms_.clear();
    mission_controls_.clear();
}

} // namespace simulator
