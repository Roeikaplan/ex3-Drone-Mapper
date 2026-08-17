#include <drone_mapper/ComparisonConfig.h>

#include <drone_mapper/Units.h>

#include <yaml-cpp/yaml.h>

#include <stdexcept>
#include <string>

namespace drone_mapper {
namespace {

/**
 * @brief Read a required scalar from a YAML map, throwing a descriptive error when absent/invalid.
 * @param node Parent YAML map.
 * @param key Key to read.
 * @param ctx Dotted context path for the error message (e.g. "original.map_offset").
 * @return The parsed double.
 * @throws std::runtime_error if the key is missing or not convertible.
 * @note Unlike the composition loader (which recovers with defaults for the *simulation*), the
 *       standalone comparison utility treats a malformed comparison_config as an unrecoverable input
 *       and reports the -1 error contract, so a missing key is a hard error here.
 */
[[nodiscard]] double requireDouble(const YAML::Node& node, const std::string& key,
                                   const std::string& ctx) {
    if (!node || !node[key]) {
        throw std::runtime_error("comparison_config: missing '" + ctx + "." + key + "'");
    }
    try {
        return node[key].as<double>();
    } catch (const YAML::Exception&) {
        throw std::runtime_error("comparison_config: invalid '" + ctx + "." + key + "'");
    }
}

/**
 * @brief Parse a `map_offset{x_offset,y_offset,height_offset}` node into a `Position3D`.
 * @param node The `map_offset` map.
 * @param ctx Context path for errors.
 * @return The offset with per-axis quantity specs attached (cm).
 */
[[nodiscard]] Position3D parseOffset(const YAML::Node& node, const std::string& ctx) {
    return Position3D{
        requireDouble(node, "x_offset", ctx) * x_extent[cm],
        requireDouble(node, "y_offset", ctx) * y_extent[cm],
        requireDouble(node, "height_offset", ctx) * z_extent[cm],
    };
}

/**
 * @brief Parse a `map_boundaries` node (x/y/height _boundary with min_cm/max_cm) into MappingBounds.
 * @param node The `map_boundaries` map.
 * @param ctx Context path for errors.
 * @return The bounds in cm.
 */
[[nodiscard]] types::MappingBounds parseBounds(const YAML::Node& node, const std::string& ctx) {
    const YAML::Node xb = node["x_boundary"];
    const YAML::Node yb = node["y_boundary"];
    const YAML::Node hb = node["height_boundary"];
    types::MappingBounds bounds{};
    bounds.min_x = requireDouble(xb, "min_cm", ctx + ".x_boundary") * x_extent[cm];
    bounds.max_x = requireDouble(xb, "max_cm", ctx + ".x_boundary") * x_extent[cm];
    bounds.min_y = requireDouble(yb, "min_cm", ctx + ".y_boundary") * y_extent[cm];
    bounds.max_y = requireDouble(yb, "max_cm", ctx + ".y_boundary") * y_extent[cm];
    bounds.min_height = requireDouble(hb, "min_cm", ctx + ".height_boundary") * z_extent[cm];
    bounds.max_height = requireDouble(hb, "max_cm", ctx + ".height_boundary") * z_extent[cm];
    return bounds;
}

/**
 * @brief Parse one `original`/`target` section into a `MapConfig`.
 * @param node The section map.
 * @param ctx Context path for errors ("original" or "target").
 * @return The map geometry (boundaries, offset, resolution).
 */
[[nodiscard]] types::MapConfig parseSection(const YAML::Node& node, const std::string& ctx) {
    if (!node) {
        throw std::runtime_error("comparison_config: missing '" + ctx + "' section");
    }
    const double res_cm = requireDouble(node, "map_res_cm", ctx);
    return types::MapConfig{parseBounds(node["map_boundaries"], ctx + ".map_boundaries"),
                            parseOffset(node["map_offset"], ctx + ".map_offset"), res_cm * cm};
}

} // namespace

ComparisonMapConfigs loadComparisonConfig(const std::filesystem::path& file) {
    // LoadFile throws on a missing/malformed file; the caller reports it via the -1 contract.
    const YAML::Node root = YAML::LoadFile(file.string());
    const YAML::Node cc = root["comparison_config"] ? root["comparison_config"] : root;
    return ComparisonMapConfigs{parseSection(cc["original"], "original"),
                                parseSection(cc["target"], "target")};
}

} // namespace drone_mapper
