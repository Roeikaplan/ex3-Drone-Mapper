/**
 * @file PluginLoader.cpp
 * @brief Enumeration, serial loading, and factory claiming.
 * @note Every failure path here records an entry in the report and returns. Nothing throws out of
 *       `load()`, because an unloadable plugin must degrade to a `-1` score rather than end the run.
 */

#include <Simulator/PluginLoader.h>

#include <Simulator/Registrar.h>

#include <algorithm>
#include <optional>
#include <system_error>
#include <utility>

namespace simulator {
namespace {

namespace fs = std::filesystem;

/**
 * @brief Whether a path looks like a loadable plugin.
 * @param file Path to test.
 * @return True when the extension is `.so`.
 * @note Extension-only: a `.so` that is not a plugin at all still gets loaded and is then rejected
 *       by the claim step, which is the correct place to notice it.
 */
[[nodiscard]] bool isSharedObject(const fs::path& file) {
    return file.extension() == ".so";
}

/**
 * @brief Resolve a path to its canonical form.
 * @param file Path to resolve.
 * @return The canonical path, or @p file unchanged when it cannot be resolved.
 * @note Canonicalisation is what makes de-duplication work. `dlopen` refcounts by the underlying
 *       library, so a folder holding both a file and a symlink to it would yield a second handle
 *       with **zero** new registrations - and the claim step would wrongly call it broken.
 */
[[nodiscard]] fs::path canonicalOrSelf(const fs::path& file) {
    std::error_code ec;
    fs::path resolved = fs::canonical(file, ec);
    return ec ? file : resolved;
}

/**
 * @brief Human-readable name for a plugin kind.
 * @param kind The kind to name.
 * @return "algorithm" or "mission control".
 * @note Used only in failure messages, which name both what was found and what was expected.
 */
[[nodiscard]] const char* kindName(PluginLoader::Kind kind) {
    return kind == PluginLoader::Kind::Algorithm ? "algorithm" : "mission control";
}

} // namespace

/**
 * @brief List the shared objects sitting directly inside a folder.
 * @param folder Directory to enumerate; not descended into.
 * @param ec Set when the directory cannot be opened or traversed; cleared otherwise.
 * @return The `.so` files found, in whatever order the filesystem reported them.
 * @note Non-recursive by design: the assignment's folder arguments name a flat directory of a team's
 *       libraries, and descending into it would pick up unrelated build output.
 * @note An entry that cannot be stat'ed is skipped rather than failing the whole listing, so one
 *       unreadable file does not hide every usable plugin beside it.
 */
std::vector<fs::path> enumerateSharedObjects(const fs::path& folder, std::error_code& ec) {
    std::vector<fs::path> files;

    ec.clear();
    const fs::directory_iterator entries{folder, ec};
    if (ec) {
        return files;
    }

    for (const fs::directory_entry& entry : entries) {
        std::error_code entry_ec;
        if (entry.is_regular_file(entry_ec) && isSharedObject(entry.path())) {
            files.push_back(entry.path());
        }
    }

    return files;
}

/**
 * @brief Load every plugin at a path and claim what each one registers.
 * @param file_or_folder A single `.so`, or a folder to enumerate non-recursively.
 * @param expected The kind the caller requires; anything else is recorded as a failure.
 * @param report Accumulates loaded plugins and failures; may already hold earlier results.
 * @note Loads strictly one file at a time. That serialisation is what makes the claim in
 *       `loadOne` sound, and it is why nothing on the registration path needs a lock.
 */
void PluginLoader::load(const fs::path& file_or_folder, Kind expected, PluginLoadReport& report) {
    for (const fs::path& file : collect(file_or_folder, report)) {
        loadOne(file, expected, report);
    }
}

/**
 * @brief Resolve a path into the sorted, de-duplicated list of `.so` files to load.
 * @param file_or_folder A single `.so`, or a folder to enumerate.
 * @param report Sink for traversal failures.
 * @return Canonical paths in a stable order, possibly empty.
 * @note An empty result is not reported as an error here. The caller decides what "no usable
 *       plugins" means, because the assignment treats an empty folder argument as a CLI error.
 */
std::vector<fs::path> PluginLoader::collect(const fs::path& file_or_folder,
                                            PluginLoadReport& report) const {
    std::vector<fs::path> files;
    std::error_code ec;

    if (fs::is_regular_file(file_or_folder, ec)) {
        files.push_back(canonicalOrSelf(file_or_folder));
        return files;
    }

    if (!fs::is_directory(file_or_folder, ec)) {
        report.failures.push_back({file_or_folder, "not a readable file or directory"});
        return files;
    }

    std::error_code list_ec;
    files = enumerateSharedObjects(file_or_folder, list_ec);
    if (list_ec) {
        report.failures.push_back(
            {file_or_folder, "directory could not be traversed: " + list_ec.message()});
        files.clear();
        return files;
    }

    /**
     * @note Canonicalisation happens here rather than inside `enumerateSharedObjects` because only
     *       the loader needs it - argument validation just counts what is present.
     */
    for (fs::path& file : files) {
        file = canonicalOrSelf(file);
    }

    /**
     * @note `directory_iterator` order is unspecified - it is filesystem order, not alphabetical.
     *       Plugin indices feed the task table, the per-plugin reports, and comparative grouping,
     *       all of which must be reproducible, so the order is fixed here. The de-duplication that
     *       follows relies on the sort having already run.
     */
    std::sort(files.begin(), files.end());
    files.erase(std::unique(files.begin(), files.end()), files.end());
    return files;
}

/**
 * @brief Load one `.so` and claim the single factory it is expected to register.
 * @param file Canonical path to the shared object.
 * @param expected The kind the caller requires.
 * @param report Sink for the claimed factory or for the failure.
 * @note Implements load-then-claim: record the registrar's counts, `dlopen` exactly one file, and
 *       require the delta to be exactly one factory of the expected kind. Anything else is dropped
 *       while the library is still mapped, so a stray factory cannot be attributed to the next
 *       plugin loaded.
 */
void PluginLoader::loadOne(const fs::path& file, Kind expected, PluginLoadReport& report) {
    Registrar& registrar = Registrar::instance();

    const std::size_t algorithms_before = registrar.algorithmCount();
    const std::size_t mission_controls_before = registrar.missionControlCount();

    PluginLibrary library{file};
    if (!library.valid()) {
        report.failures.push_back({file, library.error()});
        return;
    }

    const std::size_t new_algorithms = registrar.algorithmCount() - algorithms_before;
    const std::size_t new_mission_controls =
        registrar.missionControlCount() - mission_controls_before;

    /**
     * @note The handle is kept whatever the library registered. It is mapped now, its static
     *       initialisers have already run, and only `releaseAll()` may close it.
     */
    libraries_.push_back(std::move(library));

    if (new_algorithms + new_mission_controls != 1) {
        for (std::size_t i = 0; i < new_algorithms; ++i) {
            (void)registrar.takeLastAlgorithm();
        }
        for (std::size_t i = 0; i < new_mission_controls; ++i) {
            (void)registrar.takeLastMissionControl();
        }
        report.failures.push_back({file, new_algorithms + new_mission_controls == 0
                                             ? "loaded but registered nothing"
                                             : "registered more than one factory"});
        return;
    }

    const bool registered_algorithm = new_algorithms == 1;
    if (registered_algorithm != (expected == Kind::Algorithm)) {
        if (registered_algorithm) {
            (void)registrar.takeLastAlgorithm();
        } else {
            (void)registrar.takeLastMissionControl();
        }
        report.failures.push_back(
            {file, std::string{"registered a "} +
                       kindName(registered_algorithm ? Kind::Algorithm : Kind::MissionControl) +
                       " where a " + kindName(expected) + " was expected"});
        return;
    }

    if (registered_algorithm) {
        std::optional<common::MappingAlgorithmFactory> factory = registrar.takeLastAlgorithm();
        report.algorithms.push_back({file, std::move(*factory)});
    } else {
        std::optional<common::MissionControlFactory> factory = registrar.takeLastMissionControl();
        report.mission_controls.push_back({file, std::move(*factory)});
    }
}

/**
 * @brief Close every handle held.
 * @note Every `dlclose` in the program happens here, through `PluginLibrary`'s destructor. Callers
 *       must already have destroyed the `PluginLoadReport` (which owns the claimed factories) and
 *       called `Registrar::clear()`; otherwise a `std::function` outlives the code segment holding
 *       its target and the process crashes during static destruction, after `main` has returned.
 */
void PluginLoader::releaseAll() noexcept {
    libraries_.clear();
}

} // namespace simulator
