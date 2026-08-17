#pragma once

#include <drone_mapper/Types.h>

namespace drone_mapper {

/**
 * @brief Contract for one fully-wired simulation scenario that can be executed exactly once.
 *
 * The factory builds a concrete run; the manager calls `run()` on it without knowing how the
 * mission is driven or scored.
 *
 * @note **Do not change this interface.**
 */
class ISimulationRun {
public:
    virtual ~ISimulationRun() = default;

    /**
     * @brief Execute the scenario and return its simulation-level result.
     * @return A `SimulationResult` with the run's configs, resolution status, mission result(s),
     *         output-map path/config, and final score.
     */
    // Changed: a run now returns simulation-level data, including score and output-map metadata.
    [[nodiscard]] virtual types::SimulationResult run() = 0;
};

} // namespace drone_mapper
