#include <drone_mapper/MappingAlgorithmImpl.h>

#include <drone_mapper/IMap3D.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <map>
#include <optional>
#include <set>
#include <vector>

namespace drone_mapper {
namespace {

// Integer voxel coordinate. Same concrete type as the header's scanned-cell key, so the set can
// be passed straight through; std::array supplies the lexicographic ordering set/map need.
using VoxelKey = std::array<std::int64_t, 3>;

// Flattened, unit-stripped view of the map geometry so the grid math stays in plain doubles/ints.
struct Grid {
    double ox = 0.0, oy = 0.0, oz = 0.0; // origin (map offset), cm
    double res = 0.0;                    // voxel edge length, cm
    std::int64_t nx = 0, ny = 0, nz = 0; // voxel counts per axis
};

/**
 * @brief Flatten a MapConfig into plain-scalar grid geometry for the planner.
 * @param config Map offset, resolution, and boundaries.
 * @return A Grid holding origin/resolution in cm and per-axis voxel counts.
 * @note Voxel counts ceil the boundary span (a partial trailing voxel is kept), mirroring
 *       `Map3DImpl::makeEmptyArray` so the planner's grid matches the array it queries.
 */
[[nodiscard]] Grid makeGrid(const types::MapConfig& config) {
    Grid grid{};
    grid.res = config.resolution.force_numerical_value_in(cm);
    grid.ox = config.offset.x.force_numerical_value_in(cm);
    grid.oy = config.offset.y.force_numerical_value_in(cm);
    grid.oz = config.offset.z.force_numerical_value_in(cm);
    // Voxels per axis span the boundaries; ceil so a partial trailing voxel still exists — this
    // mirrors Map3DImpl::makeEmptyArray so the planner's grid matches the array it queries.
    const auto axis = [res = grid.res](auto min_q, auto max_q) -> std::int64_t {
        if (!(res > 0.0)) {
            return 0;
        }
        const double span = (max_q - min_q).force_numerical_value_in(cm);
        const double count = std::ceil(span / res);
        return count > 0.0 ? static_cast<std::int64_t>(count) : std::int64_t{0};
    };
    const types::MappingBounds& b = config.boundaries;
    grid.nx = axis(b.min_x, b.max_x);
    grid.ny = axis(b.min_y, b.max_y);
    grid.nz = axis(b.min_height, b.max_height);
    return grid;
}

/**
 * @brief Test whether a voxel index lies inside the grid.
 * @param grid Grid geometry (per-axis counts).
 * @param v Voxel index.
 * @return True iff every component of v is within [0, n) for its axis.
 */
[[nodiscard]] bool inBounds(const Grid& grid, const VoxelKey& v) {
    return v[0] >= 0 && v[0] < grid.nx && v[1] >= 0 && v[1] < grid.ny && v[2] >= 0 && v[2] < grid.nz;
}

/**
 * @brief Map a world position to the voxel that contains it.
 * @param grid Grid geometry.
 * @param p World position in cm.
 * @return The containing voxel index, or std::nullopt when p is outside the grid (or the
 *         resolution is non-positive).
 * @note floor selects the cell containing the point, the same convention as `Map3DImpl::locate`.
 */
[[nodiscard]] std::optional<VoxelKey> toVoxel(const Grid& grid, const Position3D& p) {
    if (!(grid.res > 0.0)) {
        return std::nullopt;
    }
    const double gx = (p.x.force_numerical_value_in(cm) - grid.ox) / grid.res;
    const double gy = (p.y.force_numerical_value_in(cm) - grid.oy) / grid.res;
    const double gz = (p.z.force_numerical_value_in(cm) - grid.oz) / grid.res;
    const VoxelKey v{static_cast<std::int64_t>(std::floor(gx)),
                     static_cast<std::int64_t>(std::floor(gy)),
                     static_cast<std::int64_t>(std::floor(gz))};
    return inBounds(grid, v) ? std::optional<VoxelKey>{v} : std::nullopt;
}

/**
 * @brief World position of a voxel's centre.
 * @param grid Grid geometry.
 * @param v Voxel index.
 * @return The centre position, re-attaching the per-axis quantity specs after plain-scalar math.
 * @note The centre round-trips through `toVoxel` back to the same index.
 */
[[nodiscard]] Position3D voxelCenter(const Grid& grid, const VoxelKey& v) {
    const double x = grid.ox + (static_cast<double>(v[0]) + 0.5) * grid.res;
    const double y = grid.oy + (static_cast<double>(v[1]) + 0.5) * grid.res;
    const double z = grid.oz + (static_cast<double>(v[2]) + 0.5) * grid.res;
    return Position3D{x * x_extent[cm], y * y_extent[cm], z * z_extent[cm]};
}

/**
 * @brief Occupancy of a voxel, sampled at its centre.
 * @param map Map to query.
 * @param grid Grid geometry.
 * @param v Voxel index.
 * @return The map's VoxelOccupancy at the voxel centre.
 */
[[nodiscard]] types::VoxelOccupancy occAt(const IMap3D& map, const Grid& grid, const VoxelKey& v) {
    return map.atVoxel(voxelCenter(grid, v));
}

// 6-connected axis neighbours (the drone moves one voxel along a single axis at a time).
constexpr std::array<VoxelKey, 6> kNeighbors{{
    {{1, 0, 0}}, {{-1, 0, 0}}, {{0, 1, 0}}, {{0, -1, 0}}, {{0, 0, 1}}, {{0, 0, -1}},
}};

/**
 * @brief Component-wise sum of a voxel index and a neighbour delta.
 * @param a Base voxel index.
 * @param d Delta to add.
 * @return a + d per component.
 */
[[nodiscard]] VoxelKey add(const VoxelKey& a, const VoxelKey& d) {
    return VoxelKey{a[0] + d[0], a[1] + d[1], a[2] + d[2]};
}

/**
 * @brief Whether a voxel is a vantage point from which scanning can reveal new space.
 * @param map Map to query.
 * @param grid Grid geometry.
 * @param v Voxel index (expected in-bounds and Empty).
 * @return True iff any 6-connected neighbour is in-bounds and still Unmapped.
 * @note Such a cell is an Empty cell adjacent to still-Unmapped space — worth travelling to.
 */
[[nodiscard]] bool hasUnmappedNeighbor(const IMap3D& map, const Grid& grid, const VoxelKey& v) {
    for (const VoxelKey& d : kNeighbors) {
        const VoxelKey n = add(v, d);
        if (inBounds(grid, n) && occAt(map, grid, n) == types::VoxelOccupancy::Unmapped) {
            return true;
        }
    }
    return false;
}

/**
 * @brief Breadth-first search for the nearest unscanned frontier cell reachable through Empty space.
 * @param map Map to query.
 * @param grid Grid geometry.
 * @param scanned Cells already used as scan destinations (excluded as targets).
 * @param cur Current drone voxel (the BFS root).
 * @return The path cur..target inclusive, or std::nullopt when no frontier is reachable.
 * @note Traversal only crosses Empty cells so every returned step is safe; excluding scanned
 *       cells as targets guarantees termination (a cell is never chosen as a destination twice).
 */
[[nodiscard]] std::optional<std::vector<VoxelKey>>
planPathToFrontier(const IMap3D& map, const Grid& grid, const std::set<VoxelKey>& scanned,
                   const VoxelKey& cur) {
    std::map<VoxelKey, VoxelKey> parent;
    std::set<VoxelKey> visited{cur};
    std::deque<VoxelKey> queue{cur};

    while (!queue.empty()) {
        const VoxelKey c = queue.front();
        queue.pop_front();

        const bool is_target = c != cur && scanned.find(c) == scanned.end()
            && occAt(map, grid, c) == types::VoxelOccupancy::Empty
            && hasUnmappedNeighbor(map, grid, c);
        if (is_target) {
            std::vector<VoxelKey> path{c};
            for (VoxelKey step = c; step != cur;) {
                step = parent.at(step);
                path.push_back(step);
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        for (const VoxelKey& d : kNeighbors) {
            const VoxelKey n = add(c, d);
            if (!inBounds(grid, n) || visited.count(n) != 0) {
                continue;
            }
            // Only drive through cells we have already proven Empty.
            if (occAt(map, grid, n) != types::VoxelOccupancy::Empty) {
                continue;
            }
            visited.insert(n);
            parent.emplace(n, c);
            queue.push_back(n);
        }
    }
    return std::nullopt;
}

/**
 * @brief Build a scan-only step command at a heading-relative orientation.
 * @param relative Scan orientation relative to the current heading.
 * @return A Working MappingStepCommand carrying only scan_orientation.
 */
[[nodiscard]] types::MappingStepCommand makeScan(const Orientation& relative) {
    types::MappingStepCommand cmd{};
    cmd.scan_orientation = relative;
    cmd.status = types::AlgorithmStatus::Working;
    return cmd;
}

/**
 * @brief Build a movement-only step command.
 * @param move Movement to issue.
 * @return A Working MappingStepCommand carrying only the movement.
 */
[[nodiscard]] types::MappingStepCommand makeMove(const types::MovementCommand& move) {
    types::MappingStepCommand cmd{};
    cmd.movement = move;
    cmd.status = types::AlgorithmStatus::Working;
    return cmd;
}

/**
 * @brief Build a six-direction scan sweep around the current pose.
 * @param heading Current drone heading; scans are expressed relative to it (MockLidar adds it back).
 * @return One scan command per axis direction (+X, -X, +Y, -Y, up, down).
 * @note The FOV cone (fov_circles) widens coverage around each of the six beams.
 */
[[nodiscard]] std::deque<types::MappingStepCommand> buildSweep(const Orientation& heading) {
    struct WorldDir {
        double horizontal_deg;
        double altitude_deg;
    };
    constexpr std::array<WorldDir, 6> dirs{{
        {0.0, 0.0}, {180.0, 0.0}, {90.0, 0.0}, {270.0, 0.0}, {0.0, 90.0}, {0.0, -90.0},
    }};

    std::deque<types::MappingStepCommand> plan;
    for (const WorldDir& d : dirs) {
        const HorizontalAngle rel_h = d.horizontal_deg * horizontal_angle[deg] - heading.horizontal;
        const AltitudeAngle rel_alt = d.altitude_deg * altitude_angle[deg] - heading.altitude;
        plan.push_back(makeScan(Orientation{rel_h, rel_alt}));
    }
    return plan;
}

/**
 * @brief Normalise an angle to its shortest signed representation.
 * @param degrees Raw angular difference in degrees.
 * @return The equivalent angle in (-180, 180].
 */
[[nodiscard]] double normalizeSignedDeg(double degrees) {
    double n = std::fmod(degrees, 360.0);
    if (n <= -180.0) {
        n += 360.0;
    } else if (n > 180.0) {
        n -= 360.0;
    }
    return n;
}

/**
 * @brief Append rotation micro-steps that turn to a desired heading.
 * @param plan Command queue to append to.
 * @param planned_heading_deg In/out: current planned heading; updated to desired_deg.
 * @param desired_deg Target heading in degrees.
 * @param max_rotate_deg Per-step rotation limit; the turn is split to stay within it.
 * @note Left increases heading, Right decreases. A zero/negative limit emits the turn as a single
 *       step (DroneControl rejects it downstream) rather than looping forever.
 */
void appendRotation(std::deque<types::MappingStepCommand>& plan, double& planned_heading_deg,
                    double desired_deg, double max_rotate_deg) {
    const double delta = normalizeSignedDeg(desired_deg - planned_heading_deg);
    if (std::abs(delta) < 1e-9) {
        planned_heading_deg = desired_deg;
        return;
    }
    const types::RotationDirection dir =
        delta > 0.0 ? types::RotationDirection::Left : types::RotationDirection::Right;
    // Guard against a zero/negative limit: emit the turn as a single step (DroneControl will
    // reject it downstream) rather than looping forever.
    const double chunk = max_rotate_deg > 0.0 ? max_rotate_deg : std::abs(delta);
    for (double remaining = std::abs(delta); remaining > 1e-9;) {
        const double step = std::min(chunk, remaining);
        plan.push_back(makeMove(types::MovementCommand{types::MovementCommandType::Rotate, dir,
                                                       step * horizontal_angle[deg], 0.0 * cm}));
        remaining -= step;
    }
    planned_heading_deg = desired_deg;
}

/**
 * @brief Append advance micro-steps covering one voxel forward along the current heading.
 * @param plan Command queue to append to.
 * @param res_cm Voxel edge length (distance to advance) in cm.
 * @param max_advance_cm Per-step advance limit; the move is split to stay within it.
 * @note A zero/negative limit advances the whole voxel in a single step.
 */
void appendAdvance(std::deque<types::MappingStepCommand>& plan, double res_cm, double max_advance_cm) {
    const double chunk = max_advance_cm > 0.0 ? max_advance_cm : res_cm;
    for (double remaining = res_cm; remaining > 1e-9;) {
        const double step = std::min(chunk, remaining);
        plan.push_back(makeMove(types::MovementCommand{types::MovementCommandType::Advance,
                                                       types::RotationDirection::Left,
                                                       0.0 * horizontal_angle[deg], step * cm}));
        remaining -= step;
    }
}

/**
 * @brief Append elevate micro-steps covering one voxel of altitude change.
 * @param plan Command queue to append to.
 * @param signed_res_cm Voxel edge length in cm; the sign gives up (+) / down (-).
 * @param max_elevate_cm Per-step elevation limit; the move is split to stay within it.
 * @note A zero/negative limit elevates the whole voxel in a single step.
 */
void appendElevate(std::deque<types::MappingStepCommand>& plan, double signed_res_cm,
                   double max_elevate_cm) {
    const double sign = signed_res_cm >= 0.0 ? 1.0 : -1.0;
    const double magnitude = std::abs(signed_res_cm);
    const double chunk = max_elevate_cm > 0.0 ? max_elevate_cm : magnitude;
    for (double remaining = magnitude; remaining > 1e-9;) {
        const double step = std::min(chunk, remaining);
        plan.push_back(makeMove(types::MovementCommand{types::MovementCommandType::Elevate,
                                                       types::RotationDirection::Left,
                                                       0.0 * horizontal_angle[deg],
                                                       (sign * step) * cm}));
        remaining -= step;
    }
}

/**
 * @brief Compile an axis-aligned voxel path into rotate/advance/elevate micro-steps.
 * @param path Voxel path (consecutive cells differ on a single axis).
 * @param start_heading Heading the drone begins the path with.
 * @param res_cm Voxel edge length in cm.
 * @param drone Drone limits (max_rotate / max_advance / max_elevate) capping each micro-step.
 * @return The ordered command queue realising the path.
 * @note Horizontal moves rotate to face the axis (0=+X, 90=+Y, 180=-X, 270=-Y) then advance;
 *       vertical moves elevate.
 */
[[nodiscard]] std::deque<types::MappingStepCommand>
buildPathCommands(const std::vector<VoxelKey>& path, const Orientation& start_heading,
                  double res_cm, const types::DroneConfigData& drone) {
    std::deque<types::MappingStepCommand> plan;
    double planned_heading_deg = start_heading.horizontal.force_numerical_value_in(deg);
    const double max_rotate_deg = std::abs(drone.max_rotate.force_numerical_value_in(deg));
    const double max_advance_cm = std::abs(drone.max_advance.force_numerical_value_in(cm));
    const double max_elevate_cm = std::abs(drone.max_elevate.force_numerical_value_in(cm));

    for (std::size_t i = 1; i < path.size(); ++i) {
        const VoxelKey& a = path[i - 1];
        const VoxelKey& b = path[i];
        const std::int64_t dx = b[0] - a[0];
        const std::int64_t dy = b[1] - a[1];
        const std::int64_t dz = b[2] - a[2];

        if (dz != 0) {
            appendElevate(plan, dz > 0 ? res_cm : -res_cm, max_elevate_cm);
            continue;
        }
        const double desired_deg = dx > 0 ? 0.0 : dx < 0 ? 180.0 : dy > 0 ? 90.0 : 270.0;
        appendRotation(plan, planned_heading_deg, desired_deg, max_rotate_deg);
        appendAdvance(plan, res_cm, max_advance_cm);
    }
    return plan;
}

/**
 * @brief Classify how the mapping run ended.
 * @param map Finished output map.
 * @param grid Grid geometry to sweep.
 * @return Finished if every in-bounds cell was mapped, else FinishedWithUnmappableVoxels.
 * @note Distinguishes a clean finish from one where in-bounds cells stayed Unmapped
 *       (occluded/unreachable through Empty space). Small maps make this full pass cheap.
 */
[[nodiscard]] types::AlgorithmStatus finalStatus(const IMap3D& map, const Grid& grid) {
    for (std::int64_t i = 0; i < grid.nx; ++i) {
        for (std::int64_t j = 0; j < grid.ny; ++j) {
            for (std::int64_t k = 0; k < grid.nz; ++k) {
                if (occAt(map, grid, VoxelKey{i, j, k}) == types::VoxelOccupancy::Unmapped) {
                    return types::AlgorithmStatus::FinishedWithUnmappableVoxels;
                }
            }
        }
    }
    return types::AlgorithmStatus::Finished;
}

/**
 * @brief Build a terminal step command carrying only a final status.
 * @param status Algorithm status to report.
 * @return A MappingStepCommand with no movement or scan, just the status.
 */
[[nodiscard]] types::MappingStepCommand terminal(types::AlgorithmStatus status) {
    types::MappingStepCommand cmd{};
    cmd.status = status;
    return cmd;
}

} // namespace

types::MappingStepCommand MappingAlgorithmImpl::nextStep(const types::DroneState& state,
                                                         const types::LidarScanResult* latest_scan) {
    // The caller has already applied latest_scan to output_map_, so we plan off the map rather
    // than the raw scan.
    (void)latest_scan;

    if (finished_) {
        return terminal(types::AlgorithmStatus::Finished);
    }

    // Drain any queued micro-steps (a scan sweep or a path to a frontier) before replanning.
    if (!plan_.empty()) {
        const types::MappingStepCommand cmd = plan_.front();
        plan_.pop_front();
        return cmd;
    }

    const Grid grid = makeGrid(output_map_.getMapConfig());
    const std::optional<VoxelKey> cur = toVoxel(grid, state.position);
    if (!cur) {
        // Degenerate geometry or the drone sits outside the mappable grid: nothing safe to do.
        finished_ = true;
        return terminal(types::AlgorithmStatus::Finished);
    }

    // Bootstrap / revisit: survey the current cell before moving on. The first ever call lands
    // here with the cell still Unmapped; scanning marks it Empty and unlocks BFS thereafter.
    if (scanned_cells_.find(*cur) == scanned_cells_.end()) {
        scanned_cells_.insert(*cur);
        plan_ = buildSweep(state.heading);
        const types::MappingStepCommand cmd = plan_.front();
        plan_.pop_front();
        return cmd;
    }

    // Head for the nearest unscanned frontier through known-Empty space.
    const std::optional<std::vector<VoxelKey>> path =
        planPathToFrontier(output_map_, grid, scanned_cells_, *cur);
    if (!path || path->size() < 2) {
        finished_ = true;
        return terminal(finalStatus(output_map_, grid));
    }

    plan_ = buildPathCommands(*path, state.heading, grid.res, drone_config_);
    if (plan_.empty()) {
        finished_ = true;
        return terminal(finalStatus(output_map_, grid));
    }
    const types::MappingStepCommand cmd = plan_.front();
    plan_.pop_front();
    return cmd;
}

} // namespace drone_mapper
