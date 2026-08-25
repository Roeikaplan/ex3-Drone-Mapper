/**
 * @file MappingAlgorithmImpl.h
 * @brief Deterministic frontier-based exploration of a bounded voxel region.
 */

#pragma once

#include <Common/IMappingAlgorithm.h>

#include <cstdint>
#include <deque>
#include <vector>

namespace algorithm {

/**
 * @brief Explores a region by repeatedly surveying a cell and travelling to the nearest frontier.
 *
 * Each cycle: scan the current voxel in all six axis directions, then breadth-first search the
 * already-observed `Empty` space for the closest cell that borders something still `Unmapped`, and
 * compile the route there into per-step movement commands.
 *
 * @note **This class carries the run's collision safety, and that is not obvious from where it
 *       sits.** The mission control can only refuse a move into a cell *already observed*
 *       `Occupied` - it has no ground truth. Because the search traverses only cells proven `Empty`
 *       by an actual scan, every path emitted here is through space the drone has already seen to be
 *       free. Relaxing that to include `Unmapped` or `PotentiallyOccupied` cells would fly the drone
 *       into walls, and nothing downstream would catch it.
 * @note Planning reads the output map and **never writes it** - the base class holds it as
 *       `const IMap3D&`, so this is compiler-enforced. Writing would also mean reasoning about its
 *       own guesses rather than about measurements.
 * @note Keeps no pose of its own. Every decision comes from the `DroneState` it is handed plus the
 *       map, so a drone that gets moved unexpectedly is planned for correctly on the next call.
 * @note Termination is guaranteed by never choosing an already-surveyed cell as a destination: the
 *       set of candidate targets strictly shrinks, so the search runs out rather than cycling.
 */
class MappingAlgorithmImpl final : public common::IMappingAlgorithm {
public:
    /**
     * @brief Inherit the dependencies constructor from the interface.
     * @note `IMappingAlgorithm` supplies one, unlike `IMissionControl`, so nothing more is needed.
     */
    using common::IMappingAlgorithm::IMappingAlgorithm;

    /**
     * @brief Decide the next command.
     * @param state The drone's current pose and step index.
     * @param latest_scan The previous scan, or null on the first call.
     * @return A scan while surveying, a movement micro-step while travelling, or a terminal status.
     * @note @p latest_scan is deliberately unused. The mission control has already applied it to the
     *       output map before this call, and the map is the accumulated picture of everything seen
     *       so far; re-reading the raw scan would double-count the most recent one.
     */
    [[nodiscard]] common::types::MappingStepCommand nextStep(
        const common::types::DroneState& state,
        const common::types::LidarScanResult* latest_scan) override;

private:
    /**
     * @brief Queued micro-steps for the current sweep or route.
     * @note Drained one per call. A whole route is compiled up front because the map cannot change
     *       while it executes - nothing scans mid-route - so re-planning each step would produce the
     *       same answer at a cost.
     */
    std::deque<common::types::MappingStepCommand> plan_{};

    /**
     * @brief Which cells have already had their scan sweep issued, indexed by linear voxel id.
     * @note Sized lazily on the first call, once the map's geometry is known. A flat array rather
     *       than a set: it is consulted for every node the search expands.
     */
    std::vector<std::uint8_t> surveyed_{};

    /**
     * @brief Latches once exploration is complete.
     * @note Without it, a finished run would re-search the whole grid on every remaining step of the
     *       mission's budget.
     */
    bool finished_ = false;
};

} // namespace algorithm
