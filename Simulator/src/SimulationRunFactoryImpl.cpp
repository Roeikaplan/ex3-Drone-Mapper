/**
 * @file SimulationRunFactoryImpl.cpp
 * @brief Wiring one run: map geometry, coordinate translation, sensors, and plugin instances.
 * @note Unit idiom throughout: strip to a plain centimetre scalar with `force_numerical_value_in`,
 *       do the arithmetic in doubles, re-attach the axis quantity spec.
 */

#include <Simulator/SimulationRunFactoryImpl.h>

#include <Simulator/Map3DImpl.h>
#include <Simulator/MockGPS.h>
#include <Simulator/MockLidar.h>
#include <Simulator/MockMovement.h>
#include <Simulator/SimulationRunImpl.h>

#include <utility>

namespace simulator {
namespace {

using common::cm;
using common::x_extent;
using common::y_extent;
using common::z_extent;

/**
 * @brief Build the full-extent geometry for a freshly loaded ground-truth array.
 * @param array The loaded voxel grid; its shape gives the per-axis voxel counts.
 * @param offset World position of voxel index {0,0,0}.
 * @param resolution Voxel edge length the map was stored at.
 * @return A config spanning `[offset, offset + shape * resolution]` on each axis.
 * @note The hidden map's boundaries **must** be real rather than a default `MapConfig`. Scoring
 *       walks the grid defined by the *origin's* boundaries, so an empty grid there would compare
 *       nothing at all and hand every run a false perfect score.
 * @note Axis mapping matches `Map3DImpl`: shape[0] is X, [1] is Y, [2] is Z.
 */
[[nodiscard]] common::types::MapConfig hiddenMapConfig(const NpyArray& array,
                                                       const common::Position3D& offset,
                                                       common::PhysicalLength resolution) {
    const NpyArray::shape_t& shape = array.Shape();
    const double res_cm = resolution.force_numerical_value_in(cm);
    const double sx = shape.size() > 0 ? static_cast<double>(shape[0]) : 0.0;
    const double sy = shape.size() > 1 ? static_cast<double>(shape[1]) : 0.0;
    const double sz = shape.size() > 2 ? static_cast<double>(shape[2]) : 0.0;

    const common::types::MappingBounds bounds{
        offset.x, offset.x + (sx * res_cm) * x_extent[cm],
        offset.y, offset.y + (sy * res_cm) * y_extent[cm],
        offset.z, offset.z + (sz * res_cm) * z_extent[cm],
    };
    return common::types::MapConfig{bounds, offset, resolution};
}

/**
 * @brief Decide the output map's voxel resolution and classify the request.
 * @param gps_resolution Mission GPS precision; the baseline the factor scales.
 * @param map_resolution Hidden-map voxel edge, used only when no usable GPS resolution was given.
 * @param factor Requested scaling: `output = baseline * factor`.
 * @return The resolution to use, paired with how the request was treated.
 * @note A factor of 1 or more coarsens or matches the grid and is honoured. Below 1 asks for finer
 *       detail than a single supported resolution can give, so the baseline is used instead and the
 *       request is marked ignored.
 * @note This only *classifies*. `SimulationManager` emits the log line from the returned status,
 *       which keeps the factory free of the shared error sink and therefore free of a lock.
 */
[[nodiscard]] std::pair<common::PhysicalLength, types::ResolutionRequestStatus>
resolveOutputResolution(common::PhysicalLength gps_resolution,
                        common::PhysicalLength map_resolution, double factor) {
    const double gps_cm = gps_resolution.force_numerical_value_in(cm);
    const double base_cm = gps_cm > 0.0 ? gps_cm : map_resolution.force_numerical_value_in(cm);

    if (factor >= 1.0) {
        return {base_cm * factor * cm, types::ResolutionRequestStatus::Accepted};
    }
    const auto status = factor > 0.0 ? types::ResolutionRequestStatus::IgnoredTooSmall
                                     : types::ResolutionRequestStatus::Ignored;
    return {base_cm * cm, status};
}

/**
 * @brief Translate a position by the map offset.
 * @param position Position expressed relative to the map origin, as the configs give it.
 * @param offset The map's world offset.
 * @return The position in world coordinates.
 * @note Configs describe the drone's start and the mission bounds *relative to the map origin*,
 *       while the voxel grids are anchored at the offset. Without this the drone spawns outside the
 *       map whenever the offset is non-zero - `house_simulation.yaml`'s `height_offset: 150` is
 *       exactly that case.
 */
[[nodiscard]] common::Position3D offsetPosition(const common::Position3D& position,
                                                const common::Position3D& offset) {
    return common::Position3D{position.x + offset.x, position.y + offset.y, position.z + offset.z};
}

/**
 * @brief Translate mapping bounds by the map offset.
 * @param bounds Bounds expressed relative to the map origin.
 * @param offset The map's world offset.
 * @return The bounds in world coordinates.
 * @note Must be applied whenever `offsetPosition` is. Translating the pose but not the bounds puts
 *       the drone inside the map and then validates its moves against a region it is not in.
 */
[[nodiscard]] common::types::MappingBounds offsetBounds(const common::types::MappingBounds& bounds,
                                                        const common::Position3D& offset) {
    return common::types::MappingBounds{
        bounds.min_x + offset.x,      bounds.max_x + offset.x,
        bounds.min_y + offset.y,      bounds.max_y + offset.y,
        bounds.min_height + offset.z, bounds.max_height + offset.z,
    };
}

} // namespace

/**
 * @brief Bind a factory to one plugin pair.
 * @param mission_control_factory Produces the mission control for every run this builds.
 * @param algorithm_factory Produces the mapping algorithm for every run this builds.
 * @param plugin_label Name distinguishing this pair's output files.
 * @param identity Source-file names for the configs; must outlive this object.
 * @param verbose Whether missions should write verbose output.
 */
SimulationRunFactoryImpl::SimulationRunFactoryImpl(
    common::MissionControlFactory mission_control_factory,
    common::MappingAlgorithmFactory algorithm_factory, std::string plugin_label,
    const ConfigIdentityIndex& identity, bool verbose)
    : mission_control_factory_(std::move(mission_control_factory)),
      algorithm_factory_(std::move(algorithm_factory)),
      plugin_label_(std::move(plugin_label)),
      identity_(identity),
      verbose_(verbose) {}

/**
 * @brief Compose the output map's filename from the four configs and the plugin.
 * @param output_path Directory the file belongs in.
 * @param simulation_config Simulation this run uses.
 * @param mission_config Mission this run uses.
 * @param drone_config Drone this run uses.
 * @param lidar_config Lidar this run uses.
 * @return An absolute path naming every dimension of the run.
 * @note Double underscores separate the fields because the source stems themselves contain single
 *       underscores, so a single separator would be ambiguous to read back.
 */
std::filesystem::path SimulationRunFactoryImpl::outputMapFile(
    const std::filesystem::path& output_path,
    const types::SimulationConfigData& simulation_config,
    const common::types::MissionConfigData& mission_config,
    const common::types::DroneConfigData& drone_config,
    const common::types::LidarConfigData& lidar_config) const {
    const std::string name = plugin_label_ + "__" + identity_.nameOf(simulation_config) + "__" +
                             identity_.nameOf(mission_config) + "__" +
                             identity_.nameOf(drone_config) + "__" +
                             identity_.nameOf(lidar_config) + ".npy";
    return output_path / name;
}

/**
 * @brief Build one run.
 * @param simulation_config Ground-truth map and starting pose.
 * @param mission_config Step budget, bounds, and resolution request.
 * @param drone_config The vehicle's limits.
 * @param lidar_config The sensor's geometry.
 * @param output_path Directory the run's output map is written into.
 * @return A run ready to execute.
 * @throws std::runtime_error when the ground-truth map cannot be read.
 * @note Construction order matters and mirrors the dependency graph: the maps first, then the
 *       sensors that read them, then the algorithm, then the mission control that drives it.
 * @note The mission control receives **no hidden map**. That omission from
 *       `MissionControlDependencies` is the deliberate isolation that stops a third-party plugin
 *       reading ground truth and writing a perfect map without flying.
 * @note The mission control also receives no drone controller. It builds its own inside its `.so`,
 *       because how a mission drives a drone is mission policy rather than simulator infrastructure.
 */
std::unique_ptr<ISimulationRun> SimulationRunFactoryImpl::create(
    const types::SimulationConfigData& simulation_config,
    const common::types::MissionConfigData& mission_config,
    const common::types::DroneConfigData& drone_config,
    const common::types::LidarConfigData& lidar_config,
    const std::filesystem::path& output_path) {
    std::unique_ptr<NpyArray> hidden_array = Map3DImpl::loadArray(simulation_config.map_filename);
    const common::types::MapConfig hidden_config = hiddenMapConfig(
        *hidden_array, simulation_config.map_offset, simulation_config.map_resolution);
    auto hidden_map = std::make_unique<Map3DImpl>(std::move(hidden_array), hidden_config);

    const auto [output_resolution, resolution_status] =
        resolveOutputResolution(mission_config.gps_resolution, simulation_config.map_resolution,
                                mission_config.output_mapping_resolution_factor);

    /**
     * @note The output grid is anchored at the mission region's world origin rather than the map's,
     *       so a mission whose bounds start away from the map origin is placed where the drone
     *       actually flies. With a zero minimum this reduces to the map offset.
     */
    const common::Position3D bounds_min{mission_config.mission_bounds.min_x,
                                        mission_config.mission_bounds.min_y,
                                        mission_config.mission_bounds.min_height};
    const common::types::MapConfig output_config{
        mission_config.mission_bounds, offsetPosition(bounds_min, simulation_config.map_offset),
        output_resolution};
    auto output_map =
        std::make_unique<Map3DImpl>(Map3DImpl::makeEmptyArray(output_config), output_config);

    const common::Position3D drone_world =
        offsetPosition(simulation_config.initial_drone_position, simulation_config.map_offset);

    /**
     * @note The mission control is handed *world-frame* bounds so its own movement validation agrees
     *       with the drone's translated pose and with the offset-anchored output grid.
     */
    common::types::MissionConfigData world_mission = mission_config;
    world_mission.mission_bounds =
        offsetBounds(mission_config.mission_bounds, simulation_config.map_offset);

    auto gps = std::make_unique<MockGPS>(
        drone_world,
        common::Orientation{simulation_config.initial_angle, 0.0 * common::altitude_angle[common::deg]},
        mission_config.gps_resolution);
    auto movement = std::make_unique<MockMovement>(*gps);
    auto lidar = std::make_unique<MockLidar>(lidar_config, *hidden_map, *gps);

    std::unique_ptr<common::IMappingAlgorithm> algorithm =
        algorithm_factory_(common::MappingAlgorithmDependencies{world_mission, lidar_config,
                                                                drone_config, *output_map});

    const std::filesystem::path output_map_file = outputMapFile(
        output_path, simulation_config, mission_config, drone_config, lidar_config);

    std::unique_ptr<common::IMissionControl> mission_control =
        mission_control_factory_(common::MissionControlDependencies{world_mission, drone_config,
                                                                    *lidar, *gps, *movement,
                                                                    *output_map, *algorithm,
                                                                    output_map_file, verbose_});

    return std::make_unique<SimulationRunImpl>(
        std::move(hidden_map), std::move(output_map), std::move(gps), std::move(movement),
        std::move(lidar), std::move(algorithm), std::move(mission_control), simulation_config,
        mission_config, resolution_status, output_map_file);
}

} // namespace simulator
