/**
 * @file PluginUse.cpp
 * @brief Handing a run's plugin uses back to the registry.
 */

#include <Simulator/PluginUse.h>

#include <Simulator/PluginRegistry.h>

namespace simulator {

/**
 * @brief Take ownership of one use of each plugin.
 * @param use The pair to release; may be empty.
 */
PluginUseGuard::PluginUseGuard(const PluginUse& use) noexcept : use_(use) {}

/**
 * @brief Give both uses back.
 * @note Order between the two does not matter - they are independent counts - but both must happen,
 *       which is why neither is conditional on the other.
 */
PluginUseGuard::~PluginUseGuard() {
    if (use_.registry == nullptr) {
        return;
    }

    if (use_.mission_control != nullptr) {
        use_.registry->release(*use_.mission_control);
    }
    if (use_.algorithm != nullptr) {
        use_.registry->release(*use_.algorithm);
    }
}

} // namespace simulator
