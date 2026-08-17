#include <drone_mapper/CompositionLoader.h>

#include <drone_mapper/Units.h>

#include <yaml-cpp/yaml.h>

#include <cstddef>
#include <filesystem>
#include <string>
#include <utility>
#include <vector>

namespace drone_mapper {
namespace {

/**
 * @brief Read one scalar from a YAML map, falling back to a default (with a log) when absent/invalid.
 * @tparam T Target scalar type.
 * @param node Parent YAML map (may itself be undefined — then every key is treated as missing).
 * @param key Key to read.
 * @param fallback Value to return when the key is missing or not convertible to @p T.
 * @param log Error sink; missing/invalid keys are logged immediately.
 * @param ctx Dotted context path for readable log messages (e.g. "drone_config").
 * @return The parsed value, or @p fallback on any problem.
 * @note Never throws: this is the per-field recovery point that keeps a partially-malformed config
 *       from aborting the whole load.
 */
template <typename T>
[[nodiscard]] T readScalar(const YAML::Node& node, const std::string& key, T fallback,
                           ErrorLogger& log, const std::string& ctx) {
    if (!node || !node[key]) {
        log.logInputError("CONFIG_MISSING_KEY", ctx + "." + key + " missing; using default.");
        return fallback;
    }
    try {
        return node[key].as<T>();
    } catch (const YAML::Exception&) {
        log.logInputError("CONFIG_BAD_VALUE", ctx + "." + key + " invalid; using default.");
        return fallback;
    }
}

/**
 * @brief Parse an `{x,y,z}`-style position node into a `Position3D`.
 * @param node Position map (e.g. `initial_drone_position` or `map_axes_offset`).
 * @param kx,ky,kz Keys for the X/Y/Z components (the two node kinds use different key names).
 * @param log Error sink.
 * @param ctx Context path for logs.
 * @return The position with per-axis quantity specs attached (cm).
 */
[[nodiscard]] Position3D parsePosition(const YAML::Node& node, const std::string& kx,
                                       const std::string& ky, const std::string& kz,
                                       ErrorLogger& log, const std::string& ctx) {
    return Position3D{
        readScalar<double>(node, kx, 0.0, log, ctx) * x_extent[cm],
        readScalar<double>(node, ky, 0.0, log, ctx) * y_extent[cm],
        readScalar<double>(node, kz, 0.0, log, ctx) * z_extent[cm],
    };
}

/**
 * @brief Parse a `boundaries` node (`x_boundary`/`y_boundary`/`height_boundary`) into MappingBounds.
 * @param node The `boundaries` map (may be undefined -> all-zero bounds + logs).
 * @param log Error sink.
 * @return The mission mapping bounds in cm.
 */
[[nodiscard]] types::MappingBounds parseBounds(const YAML::Node& node, ErrorLogger& log) {
    const YAML::Node xb = node["x_boundary"];
    const YAML::Node yb = node["y_boundary"];
    const YAML::Node hb = node["height_boundary"];
    types::MappingBounds b{};
    b.min_x = readScalar<double>(xb, "min_cm", 0.0, log, "boundaries.x_boundary") * x_extent[cm];
    b.max_x = readScalar<double>(xb, "max_cm", 0.0, log, "boundaries.x_boundary") * x_extent[cm];
    b.min_y = readScalar<double>(yb, "min_cm", 0.0, log, "boundaries.y_boundary") * y_extent[cm];
    b.max_y = readScalar<double>(yb, "max_cm", 0.0, log, "boundaries.y_boundary") * y_extent[cm];
    b.min_height =
        readScalar<double>(hb, "min_cm", 0.0, log, "boundaries.height_boundary") * z_extent[cm];
    b.max_height =
        readScalar<double>(hb, "max_cm", 0.0, log, "boundaries.height_boundary") * z_extent[cm];
    return b;
}

/**
 * @brief Parse one `drone_config` node.
 * @param node Drone config map.
 * @param log Error sink.
 * @return The drone capabilities.
 * @note `dimensions_cm` is the sphere **diameter**; the struct stores the **radius**, so it is halved.
 */
[[nodiscard]] types::DroneConfigData parseDrone(const YAML::Node& node, ErrorLogger& log) {
    types::DroneConfigData d{};
    const double diameter = readScalar<double>(node, "dimensions_cm", 0.0, log, "drone_config");
    d.radius = (diameter / 2.0) * cm;
    d.max_rotate =
        readScalar<double>(node, "max_rotate_deg", 0.0, log, "drone_config") * horizontal_angle[deg];
    d.max_advance = readScalar<double>(node, "max_advance_cm", 0.0, log, "drone_config") * cm;
    d.max_elevate = readScalar<double>(node, "max_elevate_cm", 0.0, log, "drone_config") * cm;
    return d;
}

/**
 * @brief Parse one `lidar_config` node.
 * @param node Lidar config map.
 * @param log Error sink.
 * @return The lidar sensor configuration.
 */
[[nodiscard]] types::LidarConfigData parseLidar(const YAML::Node& node, ErrorLogger& log) {
    types::LidarConfigData l{};
    l.z_min = readScalar<double>(node, "z_min_cm", 0.0, log, "lidar_config") * cm;
    l.z_max = readScalar<double>(node, "z_max_cm", 0.0, log, "lidar_config") * cm;
    l.d = readScalar<double>(node, "d_cm", 0.0, log, "lidar_config") * cm;
    l.fov_circles = readScalar<std::size_t>(node, "fov_circles", std::size_t{0}, log, "lidar_config");
    return l;
}

/**
 * @brief Parse one `mission_config` node.
 * @param node Mission config map.
 * @param log Error sink.
 * @return The mission configuration.
 * @note A missing `output_mapping_resolution_factor` defaults to 1. A value < 1
 *       is passed through unchanged — the run factory decides IGNORED and logs it, so we do not
 *       reject it here (avoids double-logging).
 */
[[nodiscard]] types::MissionConfigData parseMission(const YAML::Node& node, ErrorLogger& log) {
    types::MissionConfigData m{};
    m.max_steps = readScalar<std::size_t>(node, "max_steps", std::size_t{0}, log, "mission_config");
    m.gps_resolution =
        readScalar<double>(node, "gps_resolution_cm", 0.0, log, "mission_config") * cm;
    m.output_mapping_resolution_factor =
        node["output_mapping_resolution_factor"]
            ? readScalar<double>(node, "output_mapping_resolution_factor", 1.0, log, "mission_config")
            : 1.0;
    m.mission_bounds = parseBounds(node["boundaries"], log);
    return m;
}

/**
 * @brief Parse one `simulation_config` node.
 * @param node Simulation config map.
 * @param log Error sink.
 * @return The hidden-map + initial-pose configuration.
 */
[[nodiscard]] types::SimulationConfigData parseSimulation(const YAML::Node& node, ErrorLogger& log) {
    types::SimulationConfigData s{};
    s.map_filename =
        readScalar<std::string>(node, "map_filename", std::string{}, log, "simulation_config");
    s.map_resolution =
        readScalar<double>(node, "map_resolution_cm", 0.0, log, "simulation_config") * cm;
    s.initial_drone_position = parsePosition(node["initial_drone_position"], "x_cm", "y_cm",
                                             "height_cm", log,
                                             "simulation_config.initial_drone_position");
    s.initial_angle = readScalar<double>(node, "initial_angle_deg", 0.0, log, "simulation_config") *
                      horizontal_angle[deg];
    s.map_offset = parsePosition(node["map_axes_offset"], "x_offset", "y_offset", "height_offset",
                                 log, "simulation_config.map_axes_offset");
    return s;
}

/**
 * @brief Resolve a referenced-config path against the composition file's base directory.
 * @param base_dir Directory of the composition file (its parent path).
 * @param rel Path as written in the composition (may be relative or absolute).
 * @return @p rel unchanged when absolute; otherwise `base_dir / rel`.
 * @note Convention: every relative path in a composition — both referenced config files AND the
 *       `map_filename` inside a referenced `simulation_config` — resolves against the composition
 *       file's directory, not the CWD. This makes a dataset like `inputs/sim_compose.yaml` run
 *       correctly from any working directory.
 */
[[nodiscard]] std::filesystem::path resolveRefPath(const std::filesystem::path& base_dir,
                                                   const std::filesystem::path& rel) {
    return rel.is_absolute() ? rel : base_dir / rel;
}

/**
 * @brief Load a referenced config file and return its unwrapped inner config node.
 * @param base_dir Composition file's directory, used to resolve @p rel.
 * @param rel Referenced file path as written in the composition.
 * @param wrapper_key The single top-level key the referenced file wraps its body under
 *        (e.g. "simulation_config"); the mandated file-reference layout wraps every config this way.
 * @param log Error sink; a missing file or missing wrapper key is logged and an Undefined node is
 *        returned, so the caller's `parse*` fills defaults and one bad reference never aborts the load.
 * @return The inner config map node, or an Undefined node on any failure.
 */
[[nodiscard]] YAML::Node loadReferencedConfig(const std::filesystem::path& base_dir,
                                              const std::filesystem::path& rel,
                                              const std::string& wrapper_key, ErrorLogger& log) {
    const std::filesystem::path path = resolveRefPath(base_dir, rel);
    try {
        const YAML::Node root = YAML::LoadFile(path.string());
        const YAML::Node inner = root[wrapper_key];
        if (!inner) {
            log.logInputError("CONFIG_REF_MISSING_KEY",
                    path.string() + ": missing '" + wrapper_key + "' section; using defaults.");
        }
        return inner;
    } catch (const YAML::Exception& e) {
        log.logInputError("CONFIG_REF_LOAD_FAILED",
                "could not load referenced config '" + path.string() + "': " + e.what());
        return YAML::Node(YAML::NodeType::Undefined);
    }
}

} // namespace

// Path string of a config entry as written in the composition (empty for an inline map entry).
[[nodiscard]] std::filesystem::path entryPath(const YAML::Node& node) {
    return node.IsScalar() ? std::filesystem::path{node.as<std::string>()} : std::filesystem::path{};
}

types::SimulationCompositionData loadComposition(const std::filesystem::path& file, ErrorLogger& log,
                                                 CompositionPaths* out_paths) {
    types::SimulationCompositionData composition{};
    composition.composition_file = file;
    // Base directory for resolving every relative path referenced by this composition.
    const std::filesystem::path base_dir = file.parent_path();

    // LoadFile throws on a missing/malformed file; main catches it so a bad composition never
    // crashes the program.
    const YAML::Node root = YAML::LoadFile(file.string());
    // Accept the keys either directly at the root or nested under `simulation_compositions`.
    const YAML::Node c = root["simulation_compositions"] ? root["simulation_compositions"] : root;

    const YAML::Node simulations = c["simulations"];
    if (simulations && simulations.IsSequence()) {
        for (const auto& sim : simulations) {
            // Each config entry is either an inline map (older layout) or a scalar path string to a
            // separate file (the mandated file-reference layout). IsScalar() distinguishes them.
            const YAML::Node sim_node = sim["simulation_config"];
            const std::filesystem::path sim_path = entryPath(sim_node);
            types::SimulationConfigData sim_config;
            if (sim_node.IsScalar()) {
                sim_config = parseSimulation(
                    loadReferencedConfig(base_dir, sim_node.as<std::string>(), "simulation_config", log),
                    log);
                // Rebase a relative map path onto the composition directory so map loading is
                // CWD-independent. Inline configs (else branch) keep map_filename verbatim.
                if (!sim_config.map_filename.empty()) {
                    sim_config.map_filename = resolveRefPath(base_dir, sim_config.map_filename);
                }
            } else {
                sim_config = parseSimulation(sim_node, log);
            }

            std::vector<types::MissionConfigData> missions;
            std::vector<std::filesystem::path> mission_path_list;
            const YAML::Node mission_nodes = sim["mission_configs"];
            if (mission_nodes && mission_nodes.IsSequence()) {
                for (const auto& mission : mission_nodes) {
                    mission_path_list.push_back(entryPath(mission));
                    missions.push_back(
                        mission.IsScalar()
                            ? parseMission(loadReferencedConfig(base_dir, mission.as<std::string>(),
                                                                "mission_config", log),
                                           log)
                            : parseMission(mission, log));
                }
            }
            if (missions.empty()) {
                log.logInputError("CONFIG_NO_MISSIONS",
                        "a simulation has no mission_configs; skipping that simulation.");
                continue;
            }
            composition.simulation_mission_groups.emplace_back(std::move(sim_config),
                                                               std::move(missions));
            // Record paths only for groups actually added, keeping them parallel to the composition.
            if (out_paths != nullptr) {
                out_paths->simulation_paths.push_back(sim_path);
                out_paths->mission_paths.push_back(std::move(mission_path_list));
            }
        }
    } else {
        log.logInputError("CONFIG_NO_SIMULATIONS", "no 'simulations' sequence found in composition file.");
    }

    const YAML::Node drones = c["drone_configs"];
    if (drones && drones.IsSequence()) {
        for (const auto& drone : drones) {
            if (out_paths != nullptr) {
                out_paths->drone_paths.push_back(entryPath(drone));
            }
            composition.drones.push_back(
                drone.IsScalar()
                    ? parseDrone(loadReferencedConfig(base_dir, drone.as<std::string>(),
                                                      "drone_config", log),
                                 log)
                    : parseDrone(drone, log));
        }
    } else {
        log.logInputError("CONFIG_NO_DRONES", "no 'drone_configs' sequence found in composition file.");
    }

    const YAML::Node lidars = c["lidar_configs"];
    if (lidars && lidars.IsSequence()) {
        for (const auto& lidar : lidars) {
            if (out_paths != nullptr) {
                out_paths->lidar_paths.push_back(entryPath(lidar));
            }
            composition.lidars.push_back(
                lidar.IsScalar()
                    ? parseLidar(loadReferencedConfig(base_dir, lidar.as<std::string>(),
                                                      "lidar_config", log),
                                 log)
                    : parseLidar(lidar, log));
        }
    } else {
        log.logInputError("CONFIG_NO_LIDARS", "no 'lidar_configs' sequence found in composition file.");
    }

    return composition;
}

} // namespace drone_mapper