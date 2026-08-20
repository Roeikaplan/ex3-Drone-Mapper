/**
 * @file StubAlgorithm.cpp
 * @brief A mapping-algorithm plugin that does nothing, kept permanently as a test double.
 *
 * Built into its own `.so` so the simulator can be driven end to end without depending on the real
 * algorithm. Phase 04 runs the whole pipeline against this, and phase 07 uses it to check that a
 * competitive ranking and an `errors:` list come out right.
 *
 * @note This file is a plugin: it links `common::common` and nothing else, and it deliberately
 *       compiles to a `.so` carrying an **undefined** symbol for the registration constructor,
 *       which the dynamic linker resolves against the simulator executable at `dlopen` time.
 */

#include <Common/IMappingAlgorithm.h>
#include <Common/MappingAlgorithmRegistration.h>

namespace fixtures {

/**
 * @brief Reports completion immediately without planning anything.
 *
 * @note Architectural boundary: it never touches `output_map_`, which is the correct posture for an
 *       algorithm anyway - the map is read-only to it, and only `DroneControl` may write voxels.
 * @note Returning `Finished` on the first call keeps every mission that uses this fixture to a
 *       single step, which makes step counts in later phases trivially predictable.
 */
class StubMappingAlgorithm final : public common::IMappingAlgorithm {
public:
    /**
     * @brief Inherit the dependencies constructor from the interface.
     * @note `IMappingAlgorithm` supplies one; `IMissionControl` does not, which is why the mission
     *       control fixture writes its own.
     */
    using common::IMappingAlgorithm::IMappingAlgorithm;

    /**
     * @brief Decide the next command.
     * @return A hover with no scan, and status `Finished`.
     * @note Both parameters are unnamed: this fixture ignores the pose and the previous scan, and
     *       naming them would trip `-Wunused-parameter` under `-Werror`.
     */
    [[nodiscard]] common::types::MappingStepCommand nextStep(
        const common::types::DroneState&, const common::types::LidarScanResult*) override {
        common::types::MappingStepCommand command{};
        command.movement = common::types::MovementCommand{};
        command.status = common::types::AlgorithmStatus::Finished;
        return command;
    }
};

} // namespace fixtures

/**
 * @brief Global-scope alias required by the registration macro.
 * @note `REGISTER_MAPPING_ALGORITHM(x)` token-pastes into `register_me_##x`, so a qualified name
 *       such as `fixtures::StubMappingAlgorithm` cannot be passed - it would paste into something
 *       that is not an identifier. Aliasing at global scope is the documented workaround.
 */
using StubMappingAlgorithm_Fixture = fixtures::StubMappingAlgorithm;

REGISTER_MAPPING_ALGORITHM(StubMappingAlgorithm_Fixture);
