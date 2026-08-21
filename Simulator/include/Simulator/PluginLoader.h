/**
 * @file PluginLoader.h
 * @brief Discovery, loading, and factory claiming for a folder (or a single file) of plugins.
 */

#pragma once

#include <Common/MappingAlgorithmFactory.h>
#include <Common/MissionControlFactory.h>
#include <Simulator/PluginLibrary.h>

#include <filesystem>
#include <string>
#include <system_error>
#include <vector>

namespace simulator {

/**
 * @brief List the shared objects sitting directly inside a folder.
 * @param folder Directory to enumerate; not descended into.
 * @param ec Set when the directory cannot be opened or traversed; cleared otherwise.
 * @return The `.so` files found, in whatever order the filesystem reported them.
 * @note Shared with `CommandLineArgs` validation, which needs to reject a plugin folder holding no
 *       `.so` at all. Keeping one definition of "is this a plugin file" stops the loader and the
 *       argument checker from ever disagreeing about what they are looking at.
 * @note Returns raw, unsorted paths. `PluginLoader::collect` canonicalises, sorts, and
 *       de-duplicates on top; validation only needs to know whether the result is empty.
 * @note Never throws: the `std::error_code` overloads of `directory_iterator` are used throughout.
 */
[[nodiscard]] std::vector<std::filesystem::path> enumerateSharedObjects(
    const std::filesystem::path& folder, std::error_code& ec);

/**
 * @brief One successfully loaded mapping-algorithm plugin.
 * @note The filename is carried alongside the factory because the registration constructor cannot
 *       report where it came from - the association is inferred by the loader, not supplied.
 */
struct LoadedAlgorithm {
    std::filesystem::path file{};
    common::MappingAlgorithmFactory factory{};
};

/**
 * @brief One successfully loaded mission-control plugin.
 */
struct LoadedMissionControl {
    std::filesystem::path file{};
    common::MissionControlFactory factory{};
};

/**
 * @brief A plugin that could not be loaded or did not register usably.
 * @note Named in the mode-level report's `errors:` list; every run it would have owned scores -1.
 */
struct PluginFailure {
    std::filesystem::path file{};
    std::string reason{};
};

/**
 * @brief Everything one or more `load()` calls produced.
 *
 * @note Architectural boundary: this report **owns the factories**, having taken them out of the
 *       `Registrar`. It must therefore be destroyed before the owning `PluginLoader` releases its
 *       handles - the same rule that applies to the registrar itself.
 */
struct PluginLoadReport {
    std::vector<LoadedAlgorithm> algorithms{};
    std::vector<LoadedMissionControl> mission_controls{};
    std::vector<PluginFailure> failures{};
};

/**
 * @brief Loads plugins up front, serially, on the main thread.
 *
 * @note Architectural boundary: serial loading is not a simplification, it is what makes the whole
 *       registration path lock-free. The file-to-factory association is inferred temporally - the
 *       registrar's count before a `dlopen` versus after - and that inference is only sound when
 *       exactly one load is in flight.
 * @note Holds every `PluginLibrary` for the program's lifetime. `releaseAll()` is the only way to
 *       unload, and it must not be called until all factories and plugin-derived objects are gone.
 */
class PluginLoader {
public:
    /**
     * @brief Which kind of plugin a folder is expected to contain.
     * @note Comparative mode varies mission controls; competitive mode varies algorithms. A `.so`
     *       registering the wrong kind is a failure for that plugin, not a fatal error.
     */
    enum class Kind { Algorithm, MissionControl };

    /**
     * @brief Load every plugin at a path and claim what each one registers.
     * @param file_or_folder A single `.so`, or a folder to enumerate non-recursively.
     * @param expected The kind the caller requires; anything else is recorded as a failure.
     * @param report Accumulates loaded plugins and failures; may already hold earlier results.
     * @note Never throws: filesystem and loader errors become entries in `report.failures`.
     */
    void load(const std::filesystem::path& file_or_folder, Kind expected, PluginLoadReport& report);

    /**
     * @brief Close every handle held.
     * @note Must run **after** the `PluginLoadReport` is destroyed and `Registrar::clear()` has
     *       been called. Calling it earlier unmaps code that live `std::function` targets and
     *       plugin-derived objects still point into.
     */
    void releaseAll() noexcept;

private:
    /**
     * @brief Resolve a path into the sorted, de-duplicated list of `.so` files to load.
     * @param file_or_folder A single `.so`, or a folder to enumerate.
     * @param report Sink for traversal failures.
     * @return Canonical paths in a stable order, possibly empty.
     * @note Sorting matters: `directory_iterator` order is unspecified, and plugin indices feed the
     *       task table and report ordering, which must be reproducible across runs.
     */
    [[nodiscard]] std::vector<std::filesystem::path> collect(
        const std::filesystem::path& file_or_folder, PluginLoadReport& report) const;

    /**
     * @brief Load one `.so` and claim the single factory it is expected to register.
     * @param file Canonical path to the shared object.
     * @param expected The kind the caller requires.
     * @param report Sink for the claimed factory or for the failure.
     * @note Implements load-then-claim: record the registrar's counts, `dlopen`, and require the
     *       delta to be exactly one factory of the expected kind.
     */
    void loadOne(const std::filesystem::path& file, Kind expected, PluginLoadReport& report);

    std::vector<PluginLibrary> libraries_{};
};

} // namespace simulator
