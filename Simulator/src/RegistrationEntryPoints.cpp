/**
 * @file RegistrationEntryPoints.cpp
 * @brief Definitions of the two registration constructors that `common/` declares but never defines.
 *
 * This file *is* the plugin mechanism. A plugin's `REGISTER_*` macro creates a global object whose
 * constructor is exactly one of these functions, so the compiled `.so` carries an **undefined**
 * symbol that the dynamic linker resolves against this executable at `dlopen` time. `nm -DC` on any
 * plugin shows it as `U common::MappingAlgorithmRegistration::MappingAlgorithmRegistration(...)`.
 *
 * @note The simulator target must export its dynamic symbols (`ENABLE_EXPORTS` / `-rdynamic`).
 *       Without it the build is completely clean and `dlopen` fails at runtime with
 *       "undefined symbol" - a failure no compiler or linker check will catch.
 * @note These definitions must never be compiled into a plugin. If both sides define the symbol the
 *       plugin resolves against itself and self-registration silently stops reaching the host.
 */

#include <Common/MappingAlgorithmRegistration.h>
#include <Common/MissionControlRegistration.h>

#include <Simulator/Registrar.h>

#include <utility>

namespace common {

/**
 * @brief Publish a mapping-algorithm factory to the host.
 * @param factory Callable that builds one `IMappingAlgorithm` from its dependencies.
 * @note Runs during `dlopen`, on the loading thread, before `dlopen` returns. Loading is serial,
 *       so neither this function nor the `Registrar` needs synchronisation.
 */
MappingAlgorithmRegistration::MappingAlgorithmRegistration(MappingAlgorithmFactory factory) {
    ::simulator::Registrar::instance().addAlgorithm(std::move(factory));
}

/**
 * @brief Publish a mission-control factory to the host.
 * @param factory Callable that builds one `IMissionControl` from its dependencies.
 * @note As above: invoked from a plugin's static initialiser during `dlopen`.
 */
MissionControlRegistration::MissionControlRegistration(MissionControlFactory factory) {
    ::simulator::Registrar::instance().addMissionControl(std::move(factory));
}

} // namespace common
