/**
 * @file PluginDiscovery.cpp
 * @brief Folder traversal, canonicalisation, and ordering of plugin files.
 * @note Nothing here loads anything. Not one `dlopen` is reachable from this file, which is what
 *       lets the whole task table be built while every plugin is still on disk.
 */

#include <Simulator/PluginDiscovery.h>

#include <algorithm>

namespace simulator {
namespace {

namespace fs = std::filesystem;

/**
 * @brief Whether a path looks like a loadable plugin.
 * @param file Path to test.
 * @return True when the extension is `.so`.
 * @note Extension-only: a `.so` that is not a plugin at all is still discovered, and is rejected
 *       later by the claim step - which is the correct place to notice it, since only an attempted
 *       load can tell.
 */
[[nodiscard]] bool isSharedObject(const fs::path& file) {
    return file.extension() == ".so";
}

/**
 * @brief Resolve a path to its canonical form.
 * @param file Path to resolve.
 * @return The canonical path, or @p file unchanged when it cannot be resolved.
 * @note Canonicalisation is what makes de-duplication work. `dlopen` refcounts by the underlying
 *       library, so a folder holding both a file and a symlink to it would otherwise produce two
 *       slots for one library - and the second one's claim would see zero new registrations and
 *       wrongly call it broken.
 */
[[nodiscard]] fs::path canonicalOrSelf(const fs::path& file) {
    std::error_code ec;
    fs::path resolved = fs::canonical(file, ec);
    return ec ? file : resolved;
}

} // namespace

/**
 * @brief Human-readable name for a plugin kind.
 * @param kind The kind to name.
 * @return "algorithm" or "mission control".
 */
const char* pluginKindName(PluginKind kind) {
    return kind == PluginKind::Algorithm ? "algorithm" : "mission control";
}

/**
 * @brief List the shared objects sitting directly inside a folder.
 * @param folder Directory to enumerate; not descended into.
 * @param ec Set when the directory cannot be opened or traversed; cleared otherwise.
 * @return The `.so` files found, in filesystem order.
 * @note Non-recursive by design: the assignment's folder arguments name a flat directory of a team's
 *       libraries, and descending into it would pick up unrelated build output - including, now,
 *       the results directories this program itself writes underneath them.
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
 * @brief Resolve a path into the sorted, de-duplicated list of `.so` files it names.
 * @param file_or_folder A single `.so`, or a folder to enumerate.
 * @param failures Sink for traversal failures.
 * @return Canonical paths in a stable order, possibly empty.
 */
std::vector<fs::path> collectPluginFiles(const fs::path& file_or_folder,
                                         std::vector<PluginFailure>& failures) {
    std::vector<fs::path> files;
    std::error_code ec;

    if (fs::is_regular_file(file_or_folder, ec)) {
        files.push_back(canonicalOrSelf(file_or_folder));
        return files;
    }

    if (!fs::is_directory(file_or_folder, ec)) {
        failures.push_back({file_or_folder, "not a readable file or directory"});
        return files;
    }

    std::error_code list_ec;
    files = enumerateSharedObjects(file_or_folder, list_ec);
    if (list_ec) {
        failures.push_back({file_or_folder, "directory could not be traversed: " + list_ec.message()});
        files.clear();
        return files;
    }

    /**
     * @note Canonicalisation happens here rather than inside `enumerateSharedObjects` because only
     *       the registry needs it - argument validation just counts what is present.
     */
    for (fs::path& file : files) {
        file = canonicalOrSelf(file);
    }

    std::sort(files.begin(), files.end());
    files.erase(std::unique(files.begin(), files.end()), files.end());
    return files;
}

} // namespace simulator
