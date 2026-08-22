/**
 * @file ConfigIdentityIndex.h
 * @brief Recovery of a config's source-file name from the config object itself.
 */

#pragma once

#include <Simulator/CompositionPaths.h>
#include <Simulator/SimulationTypes.h>

#include <map>
#include <string>

namespace simulator {

/**
 * @brief Maps each config object in a composition to the name of the file it was loaded from.
 *
 * `ISimulationRunFactory::create` receives four configs by reference and nothing else - no indices,
 * no filenames. But an output map has to be named so the run that produced it is obvious, and the
 * only stable identity a `const&` carries is its address. This index is built once over the
 * composition and answers that lookup.
 *
 * @note Architectural boundary: this exists because the factory signature is frozen. Adding an
 *       identifier parameter to `create()` would be simpler and is not available; recovering
 *       identity by address is what keeps the interface untouched.
 * @note Lookups are read-only after construction, so `create()` can call them from many threads
 *       without synchronisation - which is the property phase 08's design depends on. Ex2 used a
 *       `run_index_++` counter instead; that is both a data race and a source of filenames that
 *       change with scheduling.
 * @note **The index stores raw addresses.** The composition must outlive it and must not be copied,
 *       moved, or resized afterwards. A stray copy does not crash - every lookup simply falls back
 *       to a placeholder, which is why the fallback is deliberately conspicuous.
 */
class ConfigIdentityIndex {
public:
    /**
     * @brief Construct an empty index.
     * @note Every lookup then returns the fallback. Useful only for tests that do not care about
     *       output-map naming.
     */
    ConfigIdentityIndex() = default;

    /**
     * @brief Index every config in a composition against its source file.
     * @param composition The parsed composition; must outlive this object and must not be copied.
     * @param paths Source paths recorded by `loadComposition`, index-parallel to @p composition.
     * @note A config whose path was not recorded gets a positional name instead of the fallback, so
     *       filenames stay distinguishable even when the paths were not requested.
     */
    ConfigIdentityIndex(const types::SimulationCompositionData& composition,
                        const CompositionPaths& paths);

    /**
     * @brief The name of the file a simulation config came from.
     * @param config A config belonging to the indexed composition.
     * @return Its source-file stem, a positional name, or the fallback.
     */
    [[nodiscard]] std::string nameOf(const types::SimulationConfigData& config) const;

    /**
     * @brief The name of the file a mission config came from.
     * @param config A config belonging to the indexed composition.
     * @return Its source-file stem, a positional name, or the fallback.
     */
    [[nodiscard]] std::string nameOf(const common::types::MissionConfigData& config) const;

    /**
     * @brief The name of the file a drone config came from.
     * @param config A config belonging to the indexed composition.
     * @return Its source-file stem, a positional name, or the fallback.
     */
    [[nodiscard]] std::string nameOf(const common::types::DroneConfigData& config) const;

    /**
     * @brief The name of the file a lidar config came from.
     * @param config A config belonging to the indexed composition.
     * @return Its source-file stem, a positional name, or the fallback.
     */
    [[nodiscard]] std::string nameOf(const common::types::LidarConfigData& config) const;

private:
    /**
     * @brief Look up one address.
     * @param address Address of a config object.
     * @return The recorded name, or the fallback when the address is not indexed.
     */
    [[nodiscard]] std::string lookup(const void* address) const;

    std::map<const void*, std::string> names_{};
};

} // namespace simulator
