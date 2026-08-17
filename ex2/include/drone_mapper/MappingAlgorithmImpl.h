#pragma once

#include <drone_mapper/IMappingAlgorithm.h>

#include <array>
#include <cstdint>
#include <deque>
#include <set>

namespace drone_mapper {

/**
 * @brief Deterministic frontier-based autonomous mapping strategy.
 *
 * Explores a bounded voxel region by repeatedly: scanning the current cell in every axis
 * direction, then breadth-first searching the already-observed `Empty` space for the nearest
 * frontier (an `Empty` cell adjacent to a still-`Unmapped` cell) and compiling a safe path to it
 * into per-step movement commands. It plans purely off the read-only `output_map_` (which already
 * reflects every prior scan) and the incoming `DroneState`, so it keeps no pose of its own.
 *
 * @note Architectural boundary: because the planner only ever traverses cells it has already
 *       observed to be `Empty`, ground-truth collision safety lives here rather than in
 *       `DroneControl` (which has no hidden map). Exploration terminates with `Finished`, or
 *       `FinishedWithUnmappableVoxels` if in-bounds cells remain unreachable through `Empty` space.
 */
class MappingAlgorithmImpl final : public IMappingAlgorithm {
public:
    // Inherit the (mission, lidar, drone, output_map) constructor from the interface unchanged.
    using IMappingAlgorithm::IMappingAlgorithm;

    /**
     * @brief Decide the next command from the current pose and the last scan.
     * @param state Current drone pose and step index (planning reads position/heading).
     * @param latest_scan Previous scan, or `nullptr` on the first step (already applied to the
     *        output map by the caller before this call).
     * @return A `MappingStepCommand`: a scan while surveying a cell, a movement micro-step while
     *         travelling to a frontier, and status `Finished`/`FinishedWithUnmappableVoxels`
     *         once no reachable frontier remains.
     */
    [[nodiscard]] types::MappingStepCommand nextStep(const types::DroneState& state,
                                                     const types::LidarScanResult* latest_scan) override;

private:
    /// Integer voxel coordinate `{i,j,k}`; ordered so it can key the visited set.
    using VoxelKey = std::array<std::int64_t, 3>;

    /// Queued micro-steps (scans + movements) toward the current target; refilled when drained.
    std::deque<types::MappingStepCommand> plan_{};
    /// Cells whose full scan-sweep has already been issued, to avoid rescanning.
    std::set<VoxelKey> scanned_cells_{};
    /// Latches once exploration is complete so further calls keep reporting the final status.
    bool finished_ = false;
};

} // namespace drone_mapper
