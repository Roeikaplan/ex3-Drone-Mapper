/**
 * @file PluginDiscovery.h
 * @brief Finding the plugin files a run will use, without loading any of them.
 * @note Split from loading on purpose. Discovery answers "which files exist", which the command-line
 *       validator, the task table, and the reports all need *before* anything is mapped; loading
 *       answers "give me the factory", which now happens as late as possible.
 */

#pragma once

#include <filesystem>
#include <string>
#include <system_error>
#include <vector>

namespace simulator {

/**
 * @brief Which kind of plugin a file or folder is expected to provide.
 * @note Comparative mode varies mission controls; competitive mode varies algorithms. A `.so`
 *       registering the wrong kind is a failure for that plugin, not a fatal error.
 */
enum class PluginKind { Algorithm, MissionControl };

/**
 * @brief Human-readable name for a plugin kind.
 * @param kind The kind to name.
 * @return "algorithm" or "mission control".
 * @note Used only in failure messages, which name both what was found and what was expected.
 */
[[nodiscard]] const char* pluginKindName(PluginKind kind);

/**
 * @brief A plugin that could not be found, loaded, or did not register usably.
 * @note Named in the mode-level report's `errors:` list; every run it would have owned scores -1.
 */
struct PluginFailure {
    std::filesystem::path file{};
    std::string reason{};
};

/**
 * @brief List the shared objects sitting directly inside a folder.
 * @param folder Directory to enumerate; not descended into.
 * @param ec Set when the directory cannot be opened or traversed; cleared otherwise.
 * @return The `.so` files found, in whatever order the filesystem reported them.
 * @note Shared with `CommandLineArgs` validation, which needs to reject a plugin folder holding no
 *       `.so` at all. Keeping one definition of "is this a plugin file" stops discovery and the
 *       argument checker from ever disagreeing about what they are looking at.
 * @note Returns raw, unsorted paths. `collectPluginFiles` canonicalises, sorts, and de-duplicates on
 *       top; validation only needs to know whether the result is empty.
 * @note Never throws: the `std::error_code` overloads of `directory_iterator` are used throughout.
 */
[[nodiscard]] std::vector<std::filesystem::path> enumerateSharedObjects(
    const std::filesystem::path& folder, std::error_code& ec);

/**
 * @brief Resolve a path into the sorted, de-duplicated list of `.so` files it names.
 * @param file_or_folder A single `.so`, or a folder to enumerate non-recursively.
 * @param failures Sink for traversal failures; appended to, never cleared.
 * @return Canonical paths in a stable order, possibly empty.
 * @note Sorting matters: `directory_iterator` order is unspecified, and plugin order feeds the task
 *       table, the per-plugin reports, and comparative grouping, all of which must be reproducible.
 * @note Canonicalisation is what makes de-duplication work, and de-duplication is what makes "loaded
 *       once" true for a folder holding both a file and a symlink to it: they resolve to one path
 *       and therefore to one slot.
 * @note An empty result is not an error here. The caller decides what "no usable plugins" means,
 *       because the assignment treats an empty folder argument as a command-line error.
 */
[[nodiscard]] std::vector<std::filesystem::path> collectPluginFiles(
    const std::filesystem::path& file_or_folder, std::vector<PluginFailure>& failures);

} // namespace simulator
