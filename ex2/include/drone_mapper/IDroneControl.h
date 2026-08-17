#pragma once

#include <drone_mapper/Types.h>

namespace drone_mapper {

/**
 * @brief Contract for executing one mapping step at a time.
 *
 * Sits between mission control and the (mock or real) hardware. Mission control repeatedly calls
 * `step()` and inspects the returned status to decide whether to continue, stop, or report an error,
 * without knowing how a step plans, validates, moves, or scans.
 *
 * @note **Do not change this interface.**
 */
class IDroneControl {
public:
    virtual ~IDroneControl() = default;

    /**
     * @brief Perform a single mapping step (plan -> move -> scan).
     * @return A `DroneStepResult`: `Continue` while the algorithm is still working, `Completed` once
     *         it reports it is finished, or `Error` (with a message) on a failed step.
     */
    [[nodiscard]] virtual types::DroneStepResult step() = 0;
    /**
     * @brief Current drone state (pose plus step index).
     */
    [[nodiscard]] virtual types::DroneState state() const = 0;
};

} // namespace drone_mapper
