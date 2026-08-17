#pragma once

#include <drone_mapper/Types.h>

namespace drone_mapper {

/**
 * @brief Contract for running a single mapping mission to completion.
 *
 * Abstracts the per-step loop away from the simulation run: the run owns the concrete controller and
 * calls `runMission()` exactly once, without knowing how the loop terminates or saves its map.
 *
 * @note **Do not change this interface.**
 */
class IMissionControl {
public:
    virtual ~IMissionControl() = default;

    /**
     * @brief Drive the mission's step loop and report its outcome.
     * @return A `MissionRunResult` carrying the terminal status (`Completed` / `MaxSteps` /
     *         `Error`), the number of steps executed, and any collected errors.
     */
    [[nodiscard]] virtual types::MissionRunResult runMission() = 0;
};

} // namespace drone_mapper
