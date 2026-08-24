/**
 * @file MissionControlRegistration.cpp
 * @brief The plugin's single call back into the host.
 *
 * @note This is the whole contract between a plugin and the simulator. The macro below creates a
 *       global object whose constructor runs during `dlopen`, handing the host a factory. The
 *       constructor itself is declared in `common/` and **defined in the simulator executable**, so
 *       this library compiles with an undefined symbol that the dynamic linker resolves at load
 *       time - which is why the simulator is built with exported dynamic symbols.
 * @note Nothing else in this library is reachable from outside. The host never sees
 *       `MissionControlImpl`, only an `IMissionControl` produced by the factory registered here.
 */

#include <MissionControl/MissionControlImpl.h>

#include <Common/MissionControlRegistration.h>

/**
 * @brief Global-scope alias required by the registration macro.
 * @note `REGISTER_MISSION_CONTROL(x)` token-pastes into `register_me_##x`, so a qualified name like
 *       `mission_control::MissionControlImpl` cannot be passed - it would paste into something that
 *       is not an identifier. The submitter ids ride along on the alias, which keeps the emitted
 *       symbol unique across the many teams' libraries a results folder may hold at once.
 */
using MissionControlImpl_323998450_211633813 = mission_control::MissionControlImpl;

REGISTER_MISSION_CONTROL(MissionControlImpl_323998450_211633813);
