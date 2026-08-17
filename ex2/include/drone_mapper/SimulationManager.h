#pragma once

#include <drone_mapper/ErrorLogger.h>
#include <drone_mapper/ISimulation.h>
#include <drone_mapper/ISimulationRunFactory.h>

#include <memory>

namespace drone_mapper {

/**
 * @brief Top-level runner: expands the composition and aggregates every run's result.
 *
 * For each `(simulation, mission, drone, lidar)` combination in the cartesian product it asks the
 * injected factory to build a run, executes it, and collects the `SimulationResult`s into a
 * `SimulationManagerReport`.
 *
 * @note Architectural boundary: the manager owns **orchestration, aggregation, and group-level error
 *       handling** (a run that throws is logged and scored -1 so the batch continues), but performs
 *       no file I/O — writing `simulation_output.yaml` is a separate concern
 *       (`SimulationOutputWriter`) so `run()` stays pure and unit-testable.
 */
class SimulationManager final : public ISimulation {
public:
    /**
     * @brief Construct with the run factory and the shared error sink.
     * @param run_factory Factory that wires one run per combination; must be non-null.
     * @param logger Immediate error sink for group-level run failures; must outlive the manager.
     */
    SimulationManager(std::unique_ptr<ISimulationRunFactory> run_factory, ErrorLogger& logger);

    /**
     * @brief Run every combination in the composition and aggregate the report.
     * @param composition Simulations × missions × drones × lidars to expand.
     * @param output_path Base directory handed to each run (for its output map).
     * @return A `SimulationManagerReport` with metadata (timestamp, metric, score range, error
     *         score) and one `SimulationResult` per combination; failed combinations score -1.
     */
    // Changed: matches ISimulation's new SimulationManagerReport return type.
    [[nodiscard]] types::SimulationManagerReport run(const types::SimulationCompositionData& composition,
                                              const std::filesystem::path& output_path) override; // output - to save the output map for example

private:
    std::unique_ptr<ISimulationRunFactory> run_factory_;
    ErrorLogger& logger_;
};

} // namespace drone_mapper
