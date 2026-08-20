/**
 * @file StubMissionControl.cpp
 * @brief A mission-control plugin that does nothing, kept permanently as a test double.
 *
 * Compiled twice into two `.so` files that differ only in the step count they report, via
 * `STUB_MC_STEPS`. Two are needed rather than one because comparative mode groups mission controls
 * by behaviour: with a single library there is nothing to group, so the mode cannot be tested at
 * all. Two libraries that behave differently give phase 07 a real pair to separate.
 *
 * @note Both builds define the same class and therefore emit the same `register_me_...` symbol.
 *       Loading them together in one process is a direct check that `RTLD_LOCAL` isolation works,
 *       which is the assumption the lowercase-namespace decision rests on.
 */

#include <Common/IMappingAlgorithm.h>
#include <Common/IMissionControl.h>
#include <Common/MissionControlFactory.h>
#include <Common/MissionControlRegistration.h>

#include <cstddef>

#ifndef STUB_MC_STEPS
/**
 * @brief Step count this build reports from `runMission`.
 * @note Supplied per target by CMake; the fallback only matters if the file is compiled standalone.
 */
#define STUB_MC_STEPS 1
#endif

namespace fixtures {

/**
 * @brief Runs a single algorithm step and reports completion.
 *
 * @note Architectural boundary: a mission control receives raw sensors and builds its own drone
 *       controller. This fixture skips that entirely - it only needs to prove the call chain, not
 *       to fly anything.
 * @note It never receives the hidden map, and could not read ground truth if it wanted to. That
 *       omission from `MissionControlDependencies` is deliberate plugin isolation.
 */
class StubMissionControl final : public common::IMissionControl {
public:
    /**
     * @brief Construct from the host-supplied dependencies.
     * @param dependencies Configs, sensors, output map, algorithm, output path, verbose flag.
     * @note `IMissionControl` declares no dependencies constructor to inherit, unlike
     *       `IMappingAlgorithm`, so one is written here. Only the algorithm reference is retained -
     *       everything else is unused by this fixture.
     */
    explicit StubMissionControl(common::MissionControlDependencies dependencies)
        : mapping_algorithm_(dependencies.mapping_algorithm) {}

    /**
     * @brief Run the mission.
     * @return `Completed` with `STUB_MC_STEPS` steps, unless the algorithm reports it is still working.
     * @note Calling into the algorithm is the point of this fixture: it exercises the full
     *       host to MissionControl `.so` to Algorithm `.so` chain, which no single plugin can prove
     *       on its own.
     * @note `nullptr` is passed for the latest scan because no scan has been taken - the same
     *       first-step bootstrap the real drone-stepping loop uses.
     */
    [[nodiscard]] common::types::MissionRunResult runMission() override {
        const common::types::MappingStepCommand command =
            mapping_algorithm_.nextStep(common::types::DroneState{}, nullptr);

        common::types::MissionRunResult result{};
        result.status = command.status == common::types::AlgorithmStatus::Working
                            ? common::types::MissionRunStatus::MaxSteps
                            : common::types::MissionRunStatus::Completed;
        result.steps = static_cast<std::size_t>(STUB_MC_STEPS);
        return result;
    }

private:
    /**
     * @brief The algorithm this mission control drives.
     * @note Non-owning. The host builds the algorithm first and guarantees it outlives every
     *       mission control constructed against it.
     */
    common::IMappingAlgorithm& mapping_algorithm_;
};

} // namespace fixtures

/**
 * @brief Global-scope alias required by the registration macro.
 * @note See the equivalent note in `StubAlgorithm.cpp`: the macro token-pastes its argument, so a
 *       qualified name cannot be passed.
 */
using StubMissionControl_Fixture = fixtures::StubMissionControl;

REGISTER_MISSION_CONTROL(StubMissionControl_Fixture);
