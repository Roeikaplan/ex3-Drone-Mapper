#pragma once

#include <drone_mapper/ISimulationRunFactory.h>

#include <cstddef>

namespace drone_mapper {

/**
 * @brief Concrete run factory: the single wiring point for one simulation scenario.
 *
 * For one `(simulation, mission, drone, lidar)` tuple, `create()` loads the hidden map, builds the
 * writable output map at the resolved output resolution, constructs every component injecting
 * non-owning references between siblings, and transfers `unique_ptr` ownership of the whole graph
 * into a `SimulationRunImpl`.
 *
 * @note Nearly stateless by design (all inputs arrive per call), except for `run_index_`, a counter
 *       used only to give each run a unique output-map filename so cartesian-product runs do not
 *       overwrite one another. This does not change the `ISimulationRunFactory` interface.
 */
class SimulationRunFactoryImpl final : public ISimulationRunFactory {
public:
    /**
     * @brief Wire and return one fully constructed, ready-to-run simulation node.
     * @param simulation Hidden map path/resolution/offset and initial drone pose.
     * @param mission Mission behaviour, boundaries, and requested output resolution factor.
     * @param drone Drone capabilities.
     * @param lidar LiDAR sensor configuration.
     * @param output_path Base directory; the run's output map is written under
     *        `output_path/output_results/`.
     * @return An owning `SimulationRunImpl` (as `ISimulationRun`) with all dependencies injected.
     */
    [[nodiscard]] std::unique_ptr<ISimulationRun>
    create(const types::SimulationConfigData& simulation,
           const types::MissionConfigData& mission,
           const types::DroneConfigData& drone,
           const types::LidarConfigData& lidar,
           const std::filesystem::path& output_path) override;

private:
    /// Monotonic per-process counter disambiguating output-map filenames across runs.
    std::size_t run_index_ = 0;
};

} // namespace drone_mapper
