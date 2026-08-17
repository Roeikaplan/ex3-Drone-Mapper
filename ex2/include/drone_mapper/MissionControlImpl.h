#pragma once

#include <drone_mapper/IDroneControl.h>
#include <drone_mapper/IMap3D.h>
#include <drone_mapper/IMissionControl.h>
#include <drone_mapper/IMutableMap3D.h>

#include <filesystem>

namespace drone_mapper {

/**
 * @brief Concrete mission controller that drives one mapping mission to completion.
 *
 * Owns the per-step loop: it repeatedly calls `IDroneControl::step()` until the drone reports the
 * algorithm has finished (`Completed`), a step fails fatally (`Error`), or the `max_steps` budget is
 * exhausted (`MaxSteps`). Once the loop settles it persists the run-owned output map to
 * `output_map_file` and returns a `MissionRunResult` with the outcome, the step count, and any
 * collected errors.
 *
 * @note Architectural boundary: `MissionControl` owns **termination policy and map persistence**,
 *       but **not scoring**. Scoring compares the finished output map against the hidden map and is
 *       done one layer up in `SimulationRunImpl::run()` via `MapsComparison` — which is why
 *       `MissionRunResult` carries no score. A single drone-step `Error` is a fatal collision /
 *       illegal move, so the mission **stops on the first error** — a collision ends the run with a
 *       failure notice — rather than replanning from a bad pose; the run is then scored `-1`
 *       upstream. `hidden_map` is retained for symmetry with the
 *       other components even though the loop itself does not consult it.
 */
class MissionControlImpl final : public IMissionControl {
public:
    /**
     * @brief Construct with configs and non-owning references to run-owned dependencies.
     * @param mission Mission behaviour, notably `max_steps` bounding the step loop.
     * @param drone Drone capabilities (retained for symmetry; not read by the loop).
     * @param hidden_map Ground-truth map (retained for symmetry; scoring lives in `SimulationRun`).
     * @param output_map Writable map the drone fills in; saved to `output_map_file` once the mission
     *        settles.
     * @param drone_control Controller queried once per step for the next `DroneStepResult`.
     * @param output_map_file Destination path the accumulated output map is written to.
     */
    MissionControlImpl(types::MissionConfigData mission,
                       types::DroneConfigData drone,
                       const IMap3D& hidden_map,
                       IMutableMap3D& output_map,
                       IDroneControl& drone_control,
                       std::filesystem::path output_map_file);

    /**
     * @brief Run the mission loop to completion and persist the output map.
     * @return A `MissionRunResult`: `Completed` when the drone reports it is finished, `MaxSteps`
     *         when the `max_steps` budget is exhausted first, or `Error` (with the failing step's
     *         message in `errors`) on the first fatal drone-step error. `steps` is the number of
     *         steps executed. The output map is saved to `output_map_file` before returning.
     */
    [[nodiscard]] types::MissionRunResult runMission() override;

private:
    types::MissionConfigData mission_;
    types::DroneConfigData drone_;
    const IMap3D& hidden_map_;
    IMutableMap3D& output_map_;
    IDroneControl& drone_control_;
    std::filesystem::path output_map_file_;
};

} // namespace drone_mapper
