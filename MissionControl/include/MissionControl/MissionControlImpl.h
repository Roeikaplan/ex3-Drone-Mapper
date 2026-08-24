/**
 * @file MissionControlImpl.h
 * @brief The mission loop: step until done, then save what was mapped.
 */

#pragma once

#include <MissionControl/DroneControlImpl.h>

#include <Common/IMissionControl.h>
#include <Common/MissionControlFactory.h>

#include <filesystem>
#include <fstream>
#include <optional>

namespace mission_control {

/**
 * @brief Drives a mission to completion and persists the map it produced.
 *
 * @note Architectural boundary: **this builds its own drone controller** from the raw sensors it is
 *       given. `MissionControlDependencies` deliberately hands over a lidar, a GPS, and an actuator
 *       rather than a ready-made controller, because how a mission drives its drone is mission
 *       policy rather than simulator infrastructure.
 * @note **It never receives the hidden map.** That omission is the isolation the whole plugin design
 *       rests on: without ground truth, a mission control cannot produce a perfect map except by
 *       actually flying and scanning. Scoring stays on the simulator side.
 * @note The loop itself is deliberately thin. Everything interesting - validation, ordering, scan
 *       conversion - lives one level down in `DroneControlImpl`, so this class reads as the policy it
 *       is: how long to keep going, and when to stop.
 */
class MissionControlImpl final : public common::IMissionControl {
public:
    /**
     * @brief Construct from the host-supplied dependencies.
     * @param dependencies Configs, sensors, output map, algorithm, output path, and verbose flag.
     * @note `IMissionControl` declares no dependencies constructor to inherit, unlike
     *       `IMappingAlgorithm`, so this one is written out.
     */
    explicit MissionControlImpl(common::MissionControlDependencies dependencies);

    /**
     * @brief Run the mission to completion.
     * @return The outcome, the number of steps taken, and any error that ended it.
     * @note Three ways to finish, and the report distinguishes them: the algorithm declares itself
     *       done (`Completed`), a step is refused or fails (`Error`), or the step budget runs out
     *       (`MaxSteps`). The budget outcome is the default, overwritten only when the loop breaks.
     * @note The map is saved **once**, after the loop settles. Only the final map is scored, and
     *       saving per step would mean thousands of writes per mission for no benefit.
     */
    [[nodiscard]] types::MissionRunResult runMission() override;

private:
    /**
     * @brief Append one row to the verbose trace.
     * @param step Zero-based step index.
     * @param state The drone's pose before the step ran.
     * @param result What the step returned.
     * @note Does nothing unless verbose output was requested and the file opened.
     */
    void traceStep(std::size_t step, const types::DroneState& state,
                   const types::DroneStepResult& result);

    types::MissionConfigData mission_;
    common::IMutableMap3D& output_map_;
    std::filesystem::path output_map_file_;
    bool verbose_;

    /**
     * @brief Per-step trace, opened only when verbose output was requested.
     * @note Its absence is meaningful: a run without `-verbose` leaves no trace file at all rather
     *       than an empty one.
     */
    std::optional<std::ofstream> trace_{};

    /**
     * @brief The drone this mission drives.
     * @note Declared last so it is destroyed first, before the members its references were taken
     *       from - although in practice everything it references is owned by the run rather than by
     *       this class.
     */
    DroneControlImpl drone_control_;
};

} // namespace mission_control
