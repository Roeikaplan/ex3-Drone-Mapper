/**
 * @file MappingAlgorithmRegistration.cpp
 * @brief The plugin's single call back into the host.
 *
 * @note The macro below creates a global object whose constructor runs during `dlopen`, handing the
 *       host a factory. That constructor is declared in `common/` and **defined in the simulator
 *       executable**, so this library compiles with an undefined symbol the dynamic linker resolves
 *       at load time - which is why the simulator is built with exported dynamic symbols.
 * @note Nothing else here is reachable from outside. The host never sees `MappingAlgorithmImpl`,
 *       only an `IMappingAlgorithm` produced by the factory registered below.
 */

#include <Algorithm/MappingAlgorithmImpl.h>

#include <Common/MappingAlgorithmRegistration.h>

/**
 * @brief Global-scope alias required by the registration macro.
 * @note `REGISTER_MAPPING_ALGORITHM(x)` token-pastes into `register_me_##x`, so a qualified name
 *       like `algorithm_323998450_211633813::MappingAlgorithmImpl` cannot be passed - it would paste into something that
 *       is not an identifier. The submitter ids ride along, keeping the emitted symbol unique across
 *       the many teams' libraries a competition folder holds at once.
 */
using MappingAlgorithmImpl_323998450_211633813 = algorithm_323998450_211633813::MappingAlgorithmImpl;

REGISTER_MAPPING_ALGORITHM(MappingAlgorithmImpl_323998450_211633813);
