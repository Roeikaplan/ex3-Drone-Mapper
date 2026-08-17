#include <drone_mapper/SimulationRunFactoryImpl.h>

#include <drone_mapper/DroneControlImpl.h>
#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/MappingAlgorithmImpl.h>
#include <drone_mapper/MissionControlImpl.h>
#include <drone_mapper/MockGPS.h>
#include <drone_mapper/MockLidar.h>
#include <drone_mapper/MockMovement.h>
#include <drone_mapper/SimulationRunImpl.h>

#include <filesystem>
#include <memory>
#include <string>
#include <utility>

namespace drone_mapper {
namespace {

/**
 * @brief Build the full-extent MapConfig for a freshly loaded hidden map array.
 * @param array The loaded `.npy` voxel grid; its `Shape()` gives per-axis voxel counts.
 * @param offset World position of the array origin (voxel index {0,0,0}).
 * @param res Voxel edge length (cm) the hidden map was stored at.
 * @return A MapConfig whose boundaries span `[offset, offset + Shape()*res]` on each axis, with the
 *         given offset and resolution.
 * @note The hidden map's boundaries MUST be real (not a default MapConfig): `MapsComparison::compare`
 *       walks the grid defined by its *origin*'s boundaries, and an empty grid there yields a false
 *       perfect score. Axis mapping matches `Map3DImpl` (`shape[0]=x, [1]=y, [2]=z`). Unit idiom:
 *       strip to a plain cm scalar, scale, then re-attach the per-axis quantity spec.
 */
[[nodiscard]] types::MapConfig hiddenMapConfig(const NpyArray& array, const Position3D& offset,
                                               PhysicalLength res) {
    const NpyArray::shape_t& shape = array.Shape();
    const double res_cm = res.force_numerical_value_in(cm);
    const double sx = shape.size() > 0 ? static_cast<double>(shape[0]) : 0.0;
    const double sy = shape.size() > 1 ? static_cast<double>(shape[1]) : 0.0;
    const double sz = shape.size() > 2 ? static_cast<double>(shape[2]) : 0.0;

    const types::MappingBounds bounds{
        offset.x, offset.x + (sx * res_cm) * x_extent[cm],
        offset.y, offset.y + (sy * res_cm) * y_extent[cm],
        offset.z, offset.z + (sz * res_cm) * z_extent[cm],
    };
    return types::MapConfig{bounds, offset, res};
}

/**
 * @brief Resolve the output map's voxel resolution from the GPS precision and the requested scaling
 *        factor, and classify how the request was handled.
 * @param gps_resolution Mission GPS precision (cm); the baseline the factor scales. Per the PDF the
 *        `output_mapping_resolution_factor` is "relative ... vs GPS".
 * @param map_resolution Hidden-map voxel edge length (cm); a fallback baseline used only when the
 *        mission provides no usable (positive) GPS resolution, so the output grid is still valid.
 * @param factor Requested output scaling: output_resolution = baseline * factor. A value >= 1
 *        coarsens (or matches) the grid and is honoured; a value < 1 demands finer detail than the
 *        single supported resolution and is rejected.
 * @return {resolution, status}: {baseline * factor, Accepted} when factor >= 1; {baseline,
 *         IgnoredTooSmall} when 0 < factor < 1; {baseline, Ignored} otherwise (the x1 fallback).
 * @note "Missing -> 1" defaulting is the YAML layer's job (see `parseMission`), so a well-formed
 *       mission arrives here with factor >= 1. This helper only classifies the request; the "ignored"
 *       status it returns is logged to the error log by `SimulationManager` (from the run result),
 *       keeping the factory free of the shared error sink.
 */
[[nodiscard]] std::pair<PhysicalLength, types::ResolutionRequestStatus>
resolveOutputResolution(PhysicalLength gps_resolution, PhysicalLength map_resolution, double factor) {
    const double gps_cm = gps_resolution.force_numerical_value_in(cm);
    const double base_cm = gps_cm > 0.0 ? gps_cm : map_resolution.force_numerical_value_in(cm);
    if (factor >= 1.0) {
        const double res_cm = base_cm * factor;
        return {res_cm * cm, types::ResolutionRequestStatus::Accepted};
    }
    // factor < 1: an unsupported, finer-than-supported request -> fall back to the x1 baseline. The
    // "ignored" notice is logged by SimulationManager from the returned status (carried into the
    // result), which keeps the run factory free of the shared error sink.
    const auto status = factor > 0.0 ? types::ResolutionRequestStatus::IgnoredTooSmall
                                     : types::ResolutionRequestStatus::Ignored;
    return {base_cm * cm, status};
}

/**
 * @brief Translate a world position by the map offset.
 * @param position A position expressed relative to the map origin (as configs give it).
 * @param offset The map's world offset (`map_axes_offset`).
 * @return `position + offset` per axis.
 * @note Configs express the initial drone position and mission bounds *relative to the map origin*;
 *       the hidden/output map grids are anchored at the offset. Translating brings the drone (and its
 *       movement-validation bounds) into the same world frame as the maps, so a nonzero offset places
 *       the drone inside the map rather than below it.
 */
[[nodiscard]] Position3D offsetPosition(const Position3D& position, const Position3D& offset) {
    return Position3D{position.x + offset.x, position.y + offset.y, position.z + offset.z};
}

/**
 * @brief Translate mapping bounds by the map offset (each axis min/max).
 * @param bounds Bounds expressed relative to the map origin.
 * @param offset The map's world offset.
 * @return The bounds shifted into world coordinates.
 */
[[nodiscard]] types::MappingBounds offsetBounds(const types::MappingBounds& bounds,
                                                const Position3D& offset) {
    return types::MappingBounds{
        bounds.min_x + offset.x,      bounds.max_x + offset.x,
        bounds.min_y + offset.y,      bounds.max_y + offset.y,
        bounds.min_height + offset.z, bounds.max_height + offset.z,
    };
}

} // namespace

std::unique_ptr<ISimulationRun>
SimulationRunFactoryImpl::create(const types::SimulationConfigData& simulation,
                                 const types::MissionConfigData& mission,
                                 const types::DroneConfigData& drone,
                                 const types::LidarConfigData& lidar,
                                 const std::filesystem::path& output_path) {
    // Hidden ground-truth map: load the array from disk and give it full-extent boundaries so the
    // scoring grid (origin = hidden map) is real. Only MockLidar and MapsComparison read it.
    auto hidden_array = Map3DImpl::loadArray(simulation.map_filename);
    const types::MapConfig hidden_config =
        hiddenMapConfig(*hidden_array, simulation.map_offset, simulation.map_resolution);
    auto hidden_map = std::make_unique<Map3DImpl>(hidden_array, hidden_config);

    // Writable output map: sized to the mission region at the resolved output resolution. Empty
    // mission bounds yield a 0-voxel array (safe), so a misconfigured mission scores 0, not a crash.
    const auto [output_resolution, resolution_status] = resolveOutputResolution(
        mission.gps_resolution, simulation.map_resolution, mission.output_mapping_resolution_factor);
    // Anchor the output grid at the mission region's world origin: map_offset + the bounds' minimum
    // corner. The array spans (max - min) per axis, so a mission whose region starts away from the
    // map origin (a non-zero boundary minimum) is placed where the drone actually flies, instead of
    // at the map origin. For a zero minimum this equals map_offset (a no-op).
    const Position3D bounds_min{mission.mission_bounds.min_x, mission.mission_bounds.min_y,
                                mission.mission_bounds.min_height};
    const Position3D output_origin = offsetPosition(bounds_min, simulation.map_offset);
    const types::MapConfig output_config{mission.mission_bounds, output_origin, output_resolution};
    auto output_map =
        std::make_unique<Map3DImpl>(Map3DImpl::makeEmptyArray(output_config), output_config);

    // Configs give the drone's start pose and the mission bounds *relative to the map origin*; the
    // maps are anchored at map_offset. Translate both into world coordinates so the drone flies inside
    // the map (a nonzero offset otherwise spawns it below/outside the map) and its movement validation
    // uses the same frame as the grids. For an offset of 0 this is a no-op.
    const Position3D drone_world = offsetPosition(simulation.initial_drone_position, simulation.map_offset);
    types::MissionConfigData world_mission = mission;
    world_mission.mission_bounds = offsetBounds(mission.mission_bounds, simulation.map_offset);

    auto gps = std::make_unique<MockGPS>(
        drone_world, Orientation{simulation.initial_angle, 0.0 * altitude_angle[deg]},
        mission.gps_resolution);
    auto movement = std::make_unique<MockMovement>(*gps);
    auto lidar_impl = std::make_unique<MockLidar>(lidar, *hidden_map, *gps);
    auto mapping_algorithm = std::make_unique<MappingAlgorithmImpl>(mission, lidar, drone, *output_map);

    // DroneControl validates movement against the mission bounds; give it the world-frame bounds so
    // the check aligns with the drone's (now world-frame) position and the offset-anchored output grid.
    auto drone_control = std::make_unique<DroneControlImpl>(
        drone,
        world_mission,
        *lidar_impl,
        *gps,
        *movement,
        *output_map,
        *mapping_algorithm);

    // Unique per-run output file under output_results/ so cartesian-product runs don't overwrite
    // each other. run_index_ is the factory's only state, purely to disambiguate these filenames.
    const std::filesystem::path results_dir = output_path / "output_results";
    std::error_code ec;
    std::filesystem::create_directories(results_dir, ec);
    const std::string stem = simulation.map_filename.stem().string();
    const std::filesystem::path output_map_file =
        results_dir / (stem + "_run" + std::to_string(run_index_++) + ".npy");

    auto mission_control = std::make_unique<MissionControlImpl>(
        mission,
        drone,
        *hidden_map,
        *output_map,
        *drone_control,
        output_map_file);

    return std::make_unique<SimulationRunImpl>(
        std::move(hidden_map),
        std::move(output_map),
        std::move(gps),
        std::move(movement),
        std::move(lidar_impl),
        std::move(mapping_algorithm),
        std::move(drone_control),
        std::move(mission_control),
        simulation,
        mission,
        resolution_status,
        output_map_file);
}

} // namespace drone_mapper