/**
 * @file PluginLibrary.h
 * @brief RAII ownership of a single dynamically loaded plugin.
 */

#pragma once

#include <filesystem>
#include <string>

namespace simulator {

/**
 * @brief Owns one `dlopen` handle and closes it exactly once.
 *
 * Construction attempts the load and records `dlerror()` on failure rather than throwing, so a
 * broken plugin degrades into a reportable failure instead of aborting the batch.
 *
 * @note Architectural boundary: `dlclose` happens **only** in the destructor, and the owning
 *       `PluginLoader` must not let that destructor run until every plugin-derived object *and*
 *       every stored factory is gone. Unmapping a library whose `std::function` targets are still
 *       alive crashes during static destruction, after `main` has already returned.
 * @note Move-only. A moved-from instance holds no handle, which is what keeps the close count at
 *       exactly one when these are stored in a `std::vector`.
 */
class PluginLibrary {
public:
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
