/**
 * @file CompositionLoader.cpp
 * @brief The YAML-to-structs boundary: the only translation unit that depends on yaml-cpp.
 * @note Ported from Assignment 2's loader and rewritten against the Ex3 namespaces and types. The
 *       parsing logic is proven; what changed is the surrounding shape - the inline config layout is
 *       gone, the composition types moved into `simulator::types`, and no exception may escape.
 * @note Two tiers of recovery, neither fatal: a bad *field* falls back to a default, and a bad
 *       *referenced file* degrades to an all-defaults entry. Only the composition document itself
 *       failing to parse leaves nothing to run.
 */

#include <Simulator/CompositionLoader.h>

#include <yaml-cpp/yaml.h>

#include <cstddef>
#include <exception>
#include <string>
#include <utility>
#include <vector>

namespace simulator {
namespace {

namespace fs = std::filesystem;

using common::altitude_angle;
using common::cm;
using common::deg;
using common::horizontal_angle;
using common::x_extent;
using common::y_extent;
using common::z_extent;

/**
 * @brief Look up a child of a YAML node without ever throwing.
 * @param node Parent node; may be undefined, invalid, or not a map at all.
 * @param key Child key to read.
 * @return The child node, or an undefined node when @p node cannot hold children.
 * @note yaml-cpp's `operator[]` on a *const* node hands back an invalid "zombie" for a missing key,
 *       and indexing that zombie again throws `YAML::InvalidNode`. Every lookup in this file goes
 *       through here so a config that omits a whole section - `boundaries`, say - degrades to
 *       defaults instead of aborting the load, which is the entire point of the per-field recovery
 *       tier.
 */
[[nodiscard]] YAML::Node childNode(const YAML::Node& node, const std::string& key) {
    if (!node || !node.IsMap()) {
        return YAML::Node(YAML::NodeType::Undefined);
    }
    return node[key];
}

/**
 * @brief Read one scalar from a YAML map, falling back to a default when absent or invalid.
 * @tparam T Target scalar type.
 * @param node Parent YAML map; may itself be undefined, in which case every key reads as missing.
 * @param key Key to read.
 * @param fallback Value returned when the key is missing or not convertible to @p T.
 * @param log Error sink; every recovery is reported immediately.
 * @param ctx Dotted context path for readable messages, e.g. "drone_config".
 * @return The parsed value, or @p fallback on any problem.
 * @note Never throws. This is the per-field recovery point that keeps a partially malformed config
 *       from aborting the whole load, which is why it swallows `YAML::Exception` rather than letting
 *       a bad value propagate.
 */
template <typename T>
[[nodiscard]] T readScalar(const YAML::Node& node, const std::string& key, T fallback,
                           ErrorLogger& log, const std::string& ctx) {
    const YAML::Node value = childNode(node, key);
    if (!value) {
        log.logInputError("CONFIG_MISSING_KEY", ctx + "." + key + " missing; using default.");
        return fallback;
    }
    try {
        return value.as<T>();
    } catch (const YAML::Exception&) {
        log.logInputError("CONFIG_BAD_VALUE", ctx + "." + key + " invalid; using default.");
        return fallback;
    }
}

/**
 * @brief Parse an x/y/z-style position node.
 * @param node Position map, e.g. `initial_drone_position` or `map_axes_offset`.
 * @param kx Key holding the X component.
 * @param ky Key holding the Y component.
 * @param kz Key holding the Z component.
 * @param log Error sink.
 * @param ctx Context path for messages.
 * @return The position with its per-axis quantity specs attached, in centimetres.
 * @note The two node kinds this serves use different key names, which is why the keys are
 *       parameters rather than baked in.
 * @note Unit idiom: read a plain scalar, then re-attach the axis quantity spec. `XLength`, `YLength`
 *       and `ZLength` are distinct specs, so this is what stops an axis being silently swapped.
 */
[[nodiscard]] common::Position3D parsePosition(const YAML::Node& node, const std::string& kx,
                                               const std::string& ky, const std::string& kz,
                                               ErrorLogger& log, const std::string& ctx) {
    return common::Position3D{
        readScalar<double>(node, kx, 0.0, log, ctx) * x_extent[cm],
        readScalar<double>(node, ky, 0.0, log, ctx) * y_extent[cm],
        readScalar<double>(node, kz, 0.0, log, ctx) * z_extent[cm],
    };
}

/**
 * @brief Parse a `boundaries` node into mapping bounds.
 * @param node The `boundaries` map; an undefined node yields all-zero bounds plus logged recoveries.
 * @param log Error sink.
 * @return The mission mapping bounds in centimetres.
 * @note Zero bounds are a legitimate outcome rather than an error: the run factory sizes a zero-voxel
 *       output map from them, which scores 0 instead of crashing.
 */
[[nodiscard]] common::types::MappingBounds parseBounds(const YAML::Node& node, ErrorLogger& log) {
    const YAML::Node xb = childNode(node, "x_boundary");
    const YAML::Node yb = childNode(node, "y_boundary");
    const YAML::Node hb = childNode(node, "height_boundary");

    common::types::MappingBounds bounds{};
    bounds.min_x = readScalar<double>(xb, "min_cm", 0.0, log, "boundaries.x_boundary") * x_extent[cm];
    bounds.max_x = readScalar<double>(xb, "max_cm", 0.0, log, "boundaries.x_boundary") * x_extent[cm];
    bounds.min_y = readScalar<double>(yb, "min_cm", 0.0, log, "boundaries.y_boundary") * y_extent[cm];
    bounds.max_y = readScalar<double>(yb, "max_cm", 0.0, log, "boundaries.y_boundary") * y_extent[cm];
    bounds.min_height =
        readScalar<double>(hb, "min_cm", 0.0, log, "boundaries.height_boundary") * z_extent[cm];
    bounds.max_height =
        readScalar<double>(hb, "max_cm", 0.0, log, "boundaries.height_boundary") * z_extent[cm];
    return bounds;
}

/**
 * @brief Parse one `drone_config` body.
 * @param node Drone config map.
 * @param log Error sink.
 * @return The drone's capabilities.
 * @note `dimensions_cm` is the sphere **diameter** the drone can pass through, while the struct
 *       stores a **radius**. Halving here means nothing downstream has to know that.
 */
[[nodiscard]] common::types::DroneConfigData parseDrone(const YAML::Node& node, ErrorLogger& log) {
    common::types::DroneConfigData drone{};
    const double diameter = readScalar<double>(node, "dimensions_cm", 0.0, log, "drone_config");
    drone.radius = (diameter / 2.0) * cm;
    drone.max_rotate =
        readScalar<double>(node, "max_rotate_deg", 0.0, log, "drone_config") * horizontal_angle[deg];
    drone.max_advance = readScalar<double>(node, "max_advance_cm", 0.0, log, "drone_config") * cm;
    drone.max_elevate = readScalar<double>(node, "max_elevate_cm", 0.0, log, "drone_config") * cm;
    return drone;
}

/**
 * @brief Parse one `lidar_config` body.
 * @param node Lidar config map.
 * @param log Error sink.
 * @return The sensor configuration.
 */
[[nodiscard]] common::types::LidarConfigData parseLidar(const YAML::Node& node, ErrorLogger& log) {
    common::types::LidarConfigData lidar{};
    lidar.z_min = readScalar<double>(node, "z_min_cm", 0.0, log, "lidar_config") * cm;
    lidar.z_max = readScalar<double>(node, "z_max_cm", 0.0, log, "lidar_config") * cm;
    lidar.d = readScalar<double>(node, "d_cm", 0.0, log, "lidar_config") * cm;
    lidar.fov_circles =
        readScalar<std::size_t>(node, "fov_circles", std::size_t{0}, log, "lidar_config");
    return lidar;
}

/**
 * @brief Parse one `mission_config` body.
 * @param node Mission config map.
 * @param log Error sink.
 * @return The mission configuration.
 * @note A missing `output_mapping_resolution_factor` defaults to 1. A value below 1 is passed
 *       through unchanged rather than rejected: the run factory decides whether the request can be
 *       honoured and logs the outcome, so rejecting it here would report the same problem twice.
 */
[[nodiscard]] common::types::MissionConfigData parseMission(const YAML::Node& node,
                                                            ErrorLogger& log) {
    common::types::MissionConfigData mission{};
    mission.max_steps =
        readScalar<std::size_t>(node, "max_steps", std::size_t{0}, log, "mission_config");
    mission.gps_resolution =
        readScalar<double>(node, "gps_resolution_cm", 0.0, log, "mission_config") * cm;
    mission.output_mapping_resolution_factor =
        childNode(node, "output_mapping_resolution_factor")
            ? readScalar<double>(node, "output_mapping_resolution_factor", 1.0, log,
                                 "mission_config")
            : 1.0;
    mission.mission_bounds = parseBounds(childNode(node, "boundaries"), log);
    return mission;
}

/**
 * @brief Parse one `simulation_config` body.
 * @param node Simulation config map.
 * @param log Error sink.
 * @return The ground-truth map reference plus the drone's starting pose.
 * @note `map_filename` is recorded verbatim here and rebased by the caller. This function has no
 *       idea where the composition file lives, and giving it one would duplicate that knowledge.
 */
[[nodiscard]] types::SimulationConfigData parseSimulation(const YAML::Node& node, ErrorLogger& log) {
    types::SimulationConfigData simulation{};
    simulation.map_filename =
        readScalar<std::string>(node, "map_filename", std::string{}, log, "simulation_config");
    simulation.map_resolution =
        readScalar<double>(node, "map_resolution_cm", 0.0, log, "simulation_config") * cm;
    simulation.initial_drone_position =
        parsePosition(childNode(node, "initial_drone_position"), "x_cm", "y_cm", "height_cm", log,
                      "simulation_config.initial_drone_position");
    simulation.initial_angle =
        readScalar<double>(node, "initial_angle_deg", 0.0, log, "simulation_config") *
        horizontal_angle[deg];
    simulation.map_offset =
        parsePosition(childNode(node, "map_axes_offset"), "x_offset", "y_offset", "height_offset",
                      log, "simulation_config.map_axes_offset");
    return simulation;
}

/**
 * @brief Resolve a referenced path against the composition file's directory.
 * @param base_dir Directory holding the composition file.
 * @param relative Path as written in the composition; may already be absolute.
 * @return @p relative unchanged when absolute, otherwise `base_dir / relative`.
 * @note This convention applies to every relative path a composition names - the referenced config
 *       files *and* the `map_filename` inside a referenced simulation config. Resolving against the
 *       working directory instead would make a dataset runnable only from one place.
 */
[[nodiscard]] fs::path resolveRefPath(const fs::path& base_dir, const fs::path& relative) {
    return relative.is_absolute() ? relative : base_dir / relative;
}

/**
 * @brief Load a referenced config file and return its unwrapped body.
 * @param base_dir Composition file's directory, used to resolve @p relative.
 * @param relative Referenced file path as written in the composition.
 * @param wrapper_key The single top-level key the referenced file wraps its body under, such as
 *        "simulation_config".
 * @param log Error sink.
 * @return The inner config map, or an undefined node when the file or the key is missing.
 * @note Returning an undefined node rather than failing is the whole point: it flows into the same
 *       `parse*` function as a healthy node and comes out as all-defaults, so one bad reference
 *       degrades a single config instead of aborting the load.
 */
[[nodiscard]] YAML::Node loadReferencedConfig(const fs::path& base_dir, const fs::path& relative,
                                              const std::string& wrapper_key, ErrorLogger& log) {
    const fs::path path = resolveRefPath(base_dir, relative);
    try {
        const YAML::Node root = YAML::LoadFile(path.string());
        const YAML::Node inner = childNode(root, wrapper_key);
        if (!inner) {
            log.logInputError("CONFIG_REF_MISSING_KEY", path.string() + ": missing '" + wrapper_key +
                                                            "' section; using defaults.");
        }
        return inner;
    } catch (const YAML::Exception& error) {
        log.logInputError("CONFIG_REF_LOAD_FAILED", "could not load referenced config '" +
                                                        path.string() + "': " + error.what());
        return YAML::Node(YAML::NodeType::Undefined);
    }
}

/**
 * @brief Read a config entry's path string.
 * @param node A config entry from the composition.
 * @return The path it names, or an empty path when the entry is not a scalar.
 * @note Ex3 ships only the file-reference layout, so a non-scalar entry is malformed input rather
 *       than the inline layout Assignment 2 also accepted.
 */
[[nodiscard]] fs::path entryPath(const YAML::Node& node) {
    return node && node.IsScalar() ? fs::path{node.as<std::string>()} : fs::path{};
}

/**
 * @brief Parse every simulation group and its missions.
 * @param simulations The composition's `simulations` sequence.
 * @param base_dir Composition file's directory.
 * @param log Error sink.
 * @param composition Composition being filled.
 * @param out_paths Optional source-path sink.
 * @note A simulation with no missions is skipped entirely, and its paths are *not* recorded, so the
 *       parallel arrays stay aligned with the groups that were actually accepted.
 */
void parseSimulationGroups(const YAML::Node& simulations, const fs::path& base_dir, ErrorLogger& log,
                           types::SimulationCompositionData& composition,
                           CompositionPaths* out_paths) {
    for (const YAML::Node& entry : simulations) {
        const YAML::Node simulation_node = childNode(entry, "simulation_config");
        const fs::path simulation_path = entryPath(simulation_node);

        types::SimulationConfigData simulation = parseSimulation(
            loadReferencedConfig(base_dir, simulation_path, "simulation_config", log), log);

        /**
         * @note The map path is rebased one level deeper than the config that named it:
         *       `inputs/simulation/house_simulation.yaml` says `map/scenario_house.npy`, which must
         *       become `inputs/map/...` - relative to the composition, not to the file it was read
         *       from.
         */
        if (!simulation.map_filename.empty()) {
            simulation.map_filename = resolveRefPath(base_dir, simulation.map_filename);
        }

        std::vector<common::types::MissionConfigData> missions;
        std::vector<fs::path> mission_paths;
        const YAML::Node mission_nodes = childNode(entry, "mission_configs");
        if (mission_nodes && mission_nodes.IsSequence()) {
            for (const YAML::Node& mission_node : mission_nodes) {
                const fs::path mission_path = entryPath(mission_node);
                mission_paths.push_back(mission_path);
                missions.push_back(parseMission(
                    loadReferencedConfig(base_dir, mission_path, "mission_config", log), log));
            }
        }

        if (missions.empty()) {
            log.logInputError("CONFIG_NO_MISSIONS",
                              "a simulation has no mission_configs; skipping that simulation.");
            continue;
        }

        composition.simulation_mission_groups.emplace_back(std::move(simulation),
                                                           std::move(missions));
        if (out_paths != nullptr) {
            out_paths->simulation_paths.push_back(simulation_path);
            out_paths->mission_paths.push_back(std::move(mission_paths));
        }
    }
}

} // namespace

/**
 * @brief Parse a composition file into the simulations, missions, drones and lidars it names.
 * @param file Path to the composition YAML.
 * @param logger Sink for recoverable defects.
 * @param out_paths Optional sink for each config's source path.
 * @return The parsed composition, or the reason the file could not be read.
 * @note The whole body is wrapped: `YAML::LoadFile` throws on a malformed document, and the
 *       catch-all backstop exists because "the simulator never crashes" outranks any diagnostic
 *       value in letting something unexpected escape.
 * @note The file's existence is deliberately not re-checked. `validateCommandLinePaths` already did
 *       that, and reporting it twice would be noise.
 */
CompositionLoadResult loadComposition(const fs::path& file, ErrorLogger& logger,
                                      CompositionPaths* out_paths) {
    CompositionLoadResult result{};
    result.composition.composition_file = file;

    try {
        const fs::path base_dir = file.parent_path();
        const YAML::Node root = YAML::LoadFile(file.string());

        /**
         * @note The keys are accepted either at the document root or nested under
         *       `simulation_compositions`, which is how the shipped dataset writes them.
         */
        const YAML::Node nested = childNode(root, "simulation_compositions");
        const YAML::Node body = nested ? nested : root;

        const YAML::Node simulations = childNode(body, "simulations");
        if (simulations && simulations.IsSequence()) {
            parseSimulationGroups(simulations, base_dir, logger, result.composition, out_paths);
        } else {
            logger.logInputError("CONFIG_NO_SIMULATIONS",
                                 "no 'simulations' sequence found in composition file.");
        }

        const YAML::Node drones = childNode(body, "drone_configs");
        if (drones && drones.IsSequence()) {
            for (const YAML::Node& drone : drones) {
                const fs::path drone_path = entryPath(drone);
                if (out_paths != nullptr) {
                    out_paths->drone_paths.push_back(drone_path);
                }
                result.composition.drone_configs.push_back(parseDrone(
                    loadReferencedConfig(base_dir, drone_path, "drone_config", logger), logger));
            }
        } else {
            logger.logInputError("CONFIG_NO_DRONES",
                                 "no 'drone_configs' sequence found in composition file.");
        }

        const YAML::Node lidars = childNode(body, "lidar_configs");
        if (lidars && lidars.IsSequence()) {
            for (const YAML::Node& lidar : lidars) {
                const fs::path lidar_path = entryPath(lidar);
                if (out_paths != nullptr) {
                    out_paths->lidar_paths.push_back(lidar_path);
                }
                result.composition.lidar_configs.push_back(parseLidar(
                    loadReferencedConfig(base_dir, lidar_path, "lidar_config", logger), logger));
            }
        } else {
            logger.logInputError("CONFIG_NO_LIDARS",
                                 "no 'lidar_configs' sequence found in composition file.");
        }
    } catch (const YAML::Exception& error) {
        result.error = "could not parse composition file " + file.string() + ": " + error.what();
    } catch (const std::exception& error) {
        result.error =
            "could not load composition file " + file.string() + ": " + std::string{error.what()};
    } catch (...) {
        result.error = "could not load composition file " + file.string() + ": unknown failure";
    }

    return result;
}

} // namespace simulator
