/**
 * @file MappingAlgorithmImpl.cpp
 * @brief Frontier search, route compilation, and termination.
 * @note Angle convention throughout: `0 deg` = +X east, `90 deg` = +Y south. The same convention the
 *       actuator and the lidar use; a disagreement here would send the drone somewhere other than
 *       the cell the search chose.
 */

#include <Algorithm/MappingAlgorithmImpl.h>

#include <UserCommon/VoxelGrid.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <optional>

namespace algorithm {
namespace {

using common::cm;
using common::deg;
using user_common::VoxelGrid;
using user_common::VoxelIndex;

/**
 * @brief Tolerance below which an angle or distance is treated as already satisfied.
 * @note Guards the loops that split a turn or a move into chunks: without it, a residual of 1e-16
 *       would emit an endless stream of zero-sized commands.
 */
constexpr double kEpsilon = 1e-9;

/**
 * @brief The six axis-aligned neighbours of a voxel.
 * @note Six rather than twenty-six because the drone moves along one axis at a time. A diagonal
 *       "step" would have to be compiled into two moves anyway, and treating it as one would let the
 *       search cut a corner through a cell it never checked.
 */
constexpr std::array<VoxelIndex, 6> kNeighbours{{
    {1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1},
}};

/**
 * @brief Offset a voxel index.
 * @param base Starting index.
 * @param delta Offset to apply.
 * @return The component-wise sum.
 */
[[nodiscard]] VoxelIndex offsetBy(const VoxelIndex& base, const VoxelIndex& delta) {
    return VoxelIndex{base.x + delta.x, base.y + delta.y, base.z + delta.z};
}

/**
 * @brief Occupancy of a voxel, sampled at its centre.
 * @param map Map to consult.
 * @param grid Grid geometry.
 * @param index Voxel to sample.
 * @return What the map reports there.
 * @note Sampled at the centre rather than a corner so the map's own `floor` lands squarely on the
 *       intended cell instead of on a boundary where it could round either way.
 */
[[nodiscard]] common::types::VoxelOccupancy occupancyAt(const common::IMap3D& map,
                                                        const VoxelGrid& grid,
                                                        const VoxelIndex& index) {
    return map.atVoxel(grid.centreOf(index));
}

/**
 * @brief Whether a cell borders space that has never been observed.
 * @param map Map to consult.
 * @param grid Grid geometry.
 * @param index Cell to test; expected in-bounds and `Empty`.
 * @return True when any axis neighbour is in-bounds and still `Unmapped`.
 * @note This is the definition of a frontier: somewhere the drone can stand that has unseen space
 *       next to it, and is therefore worth travelling to.
 */
[[nodiscard]] bool bordersUnmapped(const common::IMap3D& map, const VoxelGrid& grid,
                                   const VoxelIndex& index) {
    for (const VoxelIndex& delta : kNeighbours) {
        const VoxelIndex neighbour = offsetBy(index, delta);
        if (grid.contains(neighbour) &&
            occupancyAt(map, grid, neighbour) == common::types::VoxelOccupancy::Unmapped) {
            return true;
        }
    }
    return false;
}

/**
 * @brief Breadth-first search for the nearest unsurveyed frontier reachable through `Empty` space.
 * @param map Map to consult.
 * @param grid Grid geometry.
 * @param surveyed Cells already used as scan positions, indexed by linear voxel id.
 * @param start The drone's current cell.
 * @return The route from @p start to the target inclusive, or `nullopt` when none is reachable.
 *
 * @note **The traversal condition is the safety guarantee.** A neighbour is only enqueued when the
 *       map already reports it `Empty`, so every cell on every returned route has been proven free
 *       by an actual scan. `Unmapped` and `PotentiallyOccupied` are both excluded - the latter
 *       exists precisely because the sensor could not rule out an obstacle.
 * @note Breadth-first, so the first target found is the closest by step count. Nearest-first is what
 *       keeps travel cheap relative to surveying, which matters when the step budget - not the map
 *       size - is the binding constraint.
 * @note `visited` and `parent` are flat arrays indexed by linear voxel id rather than node-keyed
 *       containers. The large scenarios reach roughly 200,000 cells and this reruns on every
 *       frontier arrival, so the difference between O(1) and O(log n) with an allocation per node is
 *       the difference between a fast mission and a slow one.
 */
[[nodiscard]] std::optional<std::vector<VoxelIndex>> planRouteToFrontier(
    const common::IMap3D& map, const VoxelGrid& grid, const std::vector<std::uint8_t>& surveyed,
    const VoxelIndex& start) {
    const std::size_t cells = grid.cellCount();
    if (cells == 0) {
        return std::nullopt;
    }

    std::vector<std::uint8_t> visited(cells, 0);
    std::vector<std::int64_t> parent(cells, -1);
    std::deque<VoxelIndex> queue{start};
    visited[grid.linearIndex(start)] = 1;

    while (!queue.empty()) {
        const VoxelIndex current = queue.front();
        queue.pop_front();
        const std::size_t current_linear = grid.linearIndex(current);

        const bool is_target = !(current == start) && surveyed[current_linear] == 0 &&
                               occupancyAt(map, grid, current) ==
                                   common::types::VoxelOccupancy::Empty &&
                               bordersUnmapped(map, grid, current);
        if (is_target) {
            std::vector<VoxelIndex> route{current};
            for (std::size_t at = current_linear; parent[at] >= 0;) {
                at = static_cast<std::size_t>(parent[at]);
                route.push_back(grid.indexFromLinear(at));
            }
            std::reverse(route.begin(), route.end());
            return route;
        }

        for (const VoxelIndex& delta : kNeighbours) {
            const VoxelIndex neighbour = offsetBy(current, delta);
            if (!grid.contains(neighbour)) {
                continue;
            }
            const std::size_t neighbour_linear = grid.linearIndex(neighbour);
            if (visited[neighbour_linear] != 0) {
                continue;
            }
            if (occupancyAt(map, grid, neighbour) != common::types::VoxelOccupancy::Empty) {
                continue;
            }
            visited[neighbour_linear] = 1;
            parent[neighbour_linear] = static_cast<std::int64_t>(current_linear);
            queue.push_back(neighbour);
        }
    }

    return std::nullopt;
}

/**
 * @brief Build a scan-only command.
 * @param relative Scan direction relative to the drone's current heading.
 * @return A `Working` command carrying only the scan orientation.
 */
[[nodiscard]] common::types::MappingStepCommand makeScan(const common::Orientation& relative) {
    common::types::MappingStepCommand command{};
    command.scan_orientation = relative;
    command.status = common::types::AlgorithmStatus::Working;
    return command;
}

/**
 * @brief Build a movement-only command.
 * @param movement Movement to issue.
 * @return A `Working` command carrying only the movement.
 */
[[nodiscard]] common::types::MappingStepCommand makeMove(
    const common::types::MovementCommand& movement) {
    common::types::MappingStepCommand command{};
    command.movement = movement;
    command.status = common::types::AlgorithmStatus::Working;
    return command;
}

/**
 * @brief Build a terminal command carrying only a final status.
 * @param status Status to report.
 * @return The command.
 */
[[nodiscard]] common::types::MappingStepCommand terminal(common::types::AlgorithmStatus status) {
    common::types::MappingStepCommand command{};
    command.status = status;
    return command;
}

/**
 * @brief Build the six-direction survey of the drone's current cell.
 * @param heading The drone's current orientation.
 * @return One scan command per axis direction.
 * @note Scan orientations are **relative to the heading**, because the lidar adds the heading back
 *       when it traces. Emitting absolute directions would rotate every scan by the drone's bearing.
 * @note All six directions unconditionally. Scanning only where something is already known to be
 *       unmapped would be cheaper, and is exactly the tuning deferred to a later pass - but it needs
 *       measurement to justify, and this is the baseline being measured against.
 */
[[nodiscard]] std::deque<common::types::MappingStepCommand> buildSurvey(
    const common::Orientation& heading) {
    struct WorldDirection {
        double horizontal_deg;
        double altitude_deg;
    };
    constexpr std::array<WorldDirection, 6> directions{{
        {0.0, 0.0}, {180.0, 0.0}, {90.0, 0.0}, {270.0, 0.0}, {0.0, 90.0}, {0.0, -90.0},
    }};

    std::deque<common::types::MappingStepCommand> plan;
    for (const WorldDirection& direction : directions) {
        plan.push_back(makeScan(common::Orientation{
            direction.horizontal_deg * common::horizontal_angle[deg] - heading.horizontal,
            direction.altitude_deg * common::altitude_angle[deg] - heading.altitude,
        }));
    }
    return plan;
}

/**
 * @brief Reduce an angle to its shortest signed equivalent.
 * @param degrees Raw angular difference.
 * @return The equivalent angle in (-180, 180].
 * @note Without this a 350-degree correction would be emitted as ten chunks of turning rather than
 *       one short turn the other way.
 */
[[nodiscard]] double shortestSignedDegrees(double degrees) {
    double normalised = std::fmod(degrees, 360.0);
    if (normalised <= -180.0) {
        normalised += 360.0;
    } else if (normalised > 180.0) {
        normalised -= 360.0;
    }
    return normalised;
}

/**
 * @brief Append the rotations that bring the drone to a bearing.
 * @param plan Command queue to append to.
 * @param planned_heading_deg In/out: the heading the plan will have reached; updated to @p target_deg.
 * @param target_deg Bearing to face.
 * @param max_rotate_deg Per-command rotation limit.
 * @note The heading is *planned*, not measured. The whole route is compiled before any of it runs,
 *       so there is no pose to read - the bookkeeping has to track what the emitted commands will
 *       have done.
 * @note A non-positive limit emits the turn as one command rather than looping forever. The mission
 *       control will refuse it, which is a visible failure; an infinite loop would not be.
 */
void appendRotation(std::deque<common::types::MappingStepCommand>& plan,
                    double& planned_heading_deg, double target_deg, double max_rotate_deg) {
    const double delta = shortestSignedDegrees(target_deg - planned_heading_deg);
    if (std::abs(delta) < kEpsilon) {
        planned_heading_deg = target_deg;
        return;
    }

    const common::types::RotationDirection direction =
        delta > 0.0 ? common::types::RotationDirection::Left
                    : common::types::RotationDirection::Right;
    const double chunk = max_rotate_deg > 0.0 ? max_rotate_deg : std::abs(delta);

    for (double remaining = std::abs(delta); remaining > kEpsilon;) {
        const double step = std::min(chunk, remaining);
        plan.push_back(makeMove(common::types::MovementCommand{
            common::types::MovementCommandType::Rotate, direction,
            step * common::horizontal_angle[deg], 0.0 * cm}));
        remaining -= step;
    }
    planned_heading_deg = target_deg;
}

/**
 * @brief Append the advances covering one voxel of forward travel.
 * @param plan Command queue to append to.
 * @param distance_cm Distance to cover.
 * @param max_advance_cm Per-command limit.
 */
void appendAdvance(std::deque<common::types::MappingStepCommand>& plan, double distance_cm,
                   double max_advance_cm) {
    const double chunk = max_advance_cm > 0.0 ? max_advance_cm : distance_cm;
    for (double remaining = distance_cm; remaining > kEpsilon;) {
        const double step = std::min(chunk, remaining);
        plan.push_back(makeMove(common::types::MovementCommand{
            common::types::MovementCommandType::Advance, common::types::RotationDirection::Left,
            0.0 * common::horizontal_angle[deg], step * cm}));
        remaining -= step;
    }
}

/**
 * @brief Append the elevations covering one voxel of altitude change.
 * @param plan Command queue to append to.
 * @param signed_distance_cm Distance to cover; the sign gives up or down.
 * @param max_elevate_cm Per-command limit.
 */
void appendElevate(std::deque<common::types::MappingStepCommand>& plan, double signed_distance_cm,
                   double max_elevate_cm) {
    const double sign = signed_distance_cm >= 0.0 ? 1.0 : -1.0;
    const double magnitude = std::abs(signed_distance_cm);
    const double chunk = max_elevate_cm > 0.0 ? max_elevate_cm : magnitude;

    for (double remaining = magnitude; remaining > kEpsilon;) {
        const double step = std::min(chunk, remaining);
        plan.push_back(makeMove(common::types::MovementCommand{
            common::types::MovementCommandType::Elevate, common::types::RotationDirection::Left,
            0.0 * common::horizontal_angle[deg], (sign * step) * cm}));
        remaining -= step;
    }
}

/**
 * @brief Compile a voxel route into rotate, advance, and elevate commands.
 * @param route Cells to traverse; consecutive entries differ on exactly one axis.
 * @param start_heading The heading the drone begins with.
 * @param resolution_cm Voxel edge length, and therefore the distance of one hop.
 * @param drone The vehicle's per-command limits.
 * @return The ordered commands realising the route.
 * @note Horizontal hops turn to face the axis and then advance; vertical hops elevate without
 *       turning, since altitude is not a bearing.
 * @note Every command respects the drone's limits, so the mission control never has to refuse one.
 *       A planner that emitted over-large commands would stall the mission on its first move.
 */
[[nodiscard]] std::deque<common::types::MappingStepCommand> buildRouteCommands(
    const std::vector<VoxelIndex>& route, const common::Orientation& start_heading,
    double resolution_cm, const common::types::DroneConfigData& drone) {
    std::deque<common::types::MappingStepCommand> plan;
    double planned_heading_deg = start_heading.horizontal.force_numerical_value_in(deg);

    const double max_rotate_deg = std::abs(drone.max_rotate.force_numerical_value_in(deg));
    const double max_advance_cm = std::abs(drone.max_advance.force_numerical_value_in(cm));
    const double max_elevate_cm = std::abs(drone.max_elevate.force_numerical_value_in(cm));

    for (std::size_t i = 1; i < route.size(); ++i) {
        const VoxelIndex& from = route[i - 1];
        const VoxelIndex& to = route[i];
        const std::int64_t dx = to.x - from.x;
        const std::int64_t dy = to.y - from.y;
        const std::int64_t dz = to.z - from.z;

        if (dz != 0) {
            appendElevate(plan, dz > 0 ? resolution_cm : -resolution_cm, max_elevate_cm);
            continue;
        }

        const double target_deg = dx > 0 ? 0.0 : dx < 0 ? 180.0 : dy > 0 ? 90.0 : 270.0;
        appendRotation(plan, planned_heading_deg, target_deg, max_rotate_deg);
        appendAdvance(plan, resolution_cm, max_advance_cm);
    }

    return plan;
}

/**
 * @brief Classify how exploration ended.
 * @param map The finished map.
 * @param grid Grid geometry to sweep.
 * @return `Finished` when every in-bounds cell was observed, otherwise
 *         `FinishedWithUnmappableVoxels`.
 * @note The distinction is real information for the report: a region behind a doorway too narrow to
 *       enter, or sealed entirely, is a different outcome from one the drone simply covered.
 */
[[nodiscard]] common::types::AlgorithmStatus classifyEnding(const common::IMap3D& map,
                                                            const VoxelGrid& grid) {
    for (std::int64_t i = 0; i < grid.sizeX(); ++i) {
        for (std::int64_t j = 0; j < grid.sizeY(); ++j) {
            for (std::int64_t k = 0; k < grid.sizeZ(); ++k) {
                if (occupancyAt(map, grid, VoxelIndex{i, j, k}) ==
                    common::types::VoxelOccupancy::Unmapped) {
                    return common::types::AlgorithmStatus::FinishedWithUnmappableVoxels;
                }
            }
        }
    }
    return common::types::AlgorithmStatus::Finished;
}

} // namespace

/**
 * @brief Decide the next command.
 * @param state The drone's current pose and step index.
 * @param latest_scan The previous scan, or null on the first call; unused.
 * @return The next micro-step, or a terminal status.
 * @note The order of the checks below is the whole strategy: finish once, drain what is queued,
 *       survey where you stand, then travel to the nearest frontier.
 * @note The very first call always reaches the survey branch, because the drone's own cell starts
 *       `Unmapped` and has never been surveyed. That is what bootstraps everything - scanning turns
 *       the cell `Empty`, which gives the search somewhere to stand.
 */
common::types::MappingStepCommand MappingAlgorithmImpl::nextStep(
    const common::types::DroneState& state, const common::types::LidarScanResult* latest_scan) {
    (void)latest_scan;

    if (finished_) {
        return terminal(common::types::AlgorithmStatus::Finished);
    }

    if (!plan_.empty()) {
        const common::types::MappingStepCommand command = plan_.front();
        plan_.pop_front();
        return command;
    }

    const VoxelGrid grid = VoxelGrid::from(output_map_.getMapConfig());
    const std::optional<VoxelIndex> current = grid.indexOf(state.position);
    if (!current) {
        /**
         * @note Either the geometry is degenerate or the drone sits outside the region it was asked
         *       to map. Neither is recoverable from here, and guessing a move would be the one thing
         *       that could turn a bad configuration into a collision.
         */
        finished_ = true;
        return terminal(common::types::AlgorithmStatus::Finished);
    }

    if (surveyed_.size() != grid.cellCount()) {
        surveyed_.assign(grid.cellCount(), 0);
    }

    const std::size_t current_linear = grid.linearIndex(*current);
    if (surveyed_[current_linear] == 0) {
        surveyed_[current_linear] = 1;
        plan_ = buildSurvey(state.heading);
        const common::types::MappingStepCommand command = plan_.front();
        plan_.pop_front();
        return command;
    }

    const std::optional<std::vector<VoxelIndex>> route =
        planRouteToFrontier(output_map_, grid, surveyed_, *current);
    if (!route || route->size() < 2) {
        finished_ = true;
        return terminal(classifyEnding(output_map_, grid));
    }

    plan_ = buildRouteCommands(*route, state.heading, grid.resolutionCm(), drone_config_);
    if (plan_.empty()) {
        finished_ = true;
        return terminal(classifyEnding(output_map_, grid));
    }

    const common::types::MappingStepCommand command = plan_.front();
    plan_.pop_front();
    return command;
}

} // namespace algorithm
