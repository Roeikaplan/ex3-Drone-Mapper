#pragma once

#include <drone_mapper/types/MapTypes.h>

#include <filesystem>

namespace drone_mapper {

/**
 * @brief The two map geometries a `comparison_config` describes: one for the origin, one for target.
 *
 * Each `MapConfig` carries the resolution, world offset, and boundaries the standalone
 * `maps_comparison` utility should use for that map when comparing two `.npy` files whose geometries
 * are not implied by their shapes alone.
 */
struct ComparisonMapConfigs {
    types::MapConfig original{};
    types::MapConfig target{};
};

/**
 * @brief Parse a `comparison_config` YAML into the origin/target map geometries.
 *
 * Expects `comparison_config: { original: {...}, target: {...} }` (or the two sections at the root),
 * where each section has `map_res_cm`, `map_offset{x_offset,y_offset,height_offset}`, and
 * `map_boundaries{x_boundary/y_boundary/height_boundary{min_cm,max_cm}}` — the format from the
 * assignment. File units are centimetres.
 *
 * @param file Path to the comparison-config YAML.
 * @return The parsed origin/target geometries.
 * @throws std::runtime_error / YAML::Exception if the file cannot be read or a required key is
 *         missing/invalid. The `maps_comparison` executable catches this and prints the -1 contract.
 * @note yaml-cpp is confined to the implementation so this header stays dependency-free, matching the
 *       `CompositionLoader` house style.
 */
[[nodiscard]] ComparisonMapConfigs loadComparisonConfig(const std::filesystem::path& file);

} // namespace drone_mapper
