/**
 * @file PluginUse.h
 * @brief One run's claim on the two plugin libraries it needs, and the guard that gives it back.
 * @note Deliberately declaration-only: the task table stores a `PluginUse` in every cell, and
 *       nothing about a cell should require the registry's full definition. Only the guard's
 *       destructor needs it, and that lives in the `.cpp`.
 */

#pragma once

namespace simulator {

class PluginRegistry;
class PluginSlot;

/**
 * @brief The plugin pair one run holds a reserved use of.
 *
 * @note All three pointers are non-owning and are either all set or all null. A default-constructed
 *       `PluginUse` means "no registry is managing these libraries", which is what a directly
 *       constructed `SimulationManager` in a test uses; releasing it does nothing.
 */
struct PluginUse {
    PluginRegistry* registry = nullptr;
    PluginSlot* mission_control = nullptr;
    PluginSlot* algorithm = nullptr;
};

/**
 * @brief Releases a run's use of its two plugins when it goes out of scope.
 *
 * @note Architectural boundary: **this destructor is where a mid-batch `dlclose` is triggered**, so
 *       it must run after everything the plugins produced is gone. The one call site declares it
 *       *before* the run it guards, which is what makes the run - and with it both plugin instances
 *       - destroyed first.
 * @note Runs on every path out of a cell, including both catch arms. A run that throws must still
 *       give back its use, or the library it needed would stay mapped for the rest of the program.
 */
class PluginUseGuard {
public:
    /**
     * @brief Take ownership of one use of each plugin in @p use.
     * @param use The pair to release; may be empty.
     * @note Does not itself reserve anything. The uses were reserved before dispatch, when the task
     *       table was built and the exact count was known.
     */
    explicit PluginUseGuard(const PluginUse& use) noexcept;

    /**
     * @brief Give both uses back.
     * @note `noexcept`, like everything on this path: it is called during stack unwinding when a
     *       plugin throws.
     */
    ~PluginUseGuard();

    PluginUseGuard(const PluginUseGuard&) = delete;
    PluginUseGuard& operator=(const PluginUseGuard&) = delete;
    PluginUseGuard(PluginUseGuard&&) = delete;
    PluginUseGuard& operator=(PluginUseGuard&&) = delete;

private:
    PluginUse use_;
};

} // namespace simulator
