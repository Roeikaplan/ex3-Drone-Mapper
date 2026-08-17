#include <drone_mapper/ComparisonConfig.h>
#include <drone_mapper/Map3DImpl.h>
#include <drone_mapper/MapsComparison.h>

#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <vector>

namespace {

using namespace drone_mapper;

/**
 * @brief Build a voxel-grid geometry for a raw `.npy` of the given shape.
 * @param shape 3-D array shape `(nx, ny, nz)` as loaded from disk.
 * @return A `MapConfig` with 1 cm voxels anchored at the world origin, so index `i` along an
 *         axis maps to world position `i` cm.
 * @note Used when no `comparison_config` is provided: the assignment says to then assume both maps
 *       share the same offset, boundaries, and resolution — which this identical geometry encodes.
 */
[[nodiscard]] types::MapConfig configForShape(const NpyArray::shape_t& shape) {
    const types::MappingBounds bounds{
        XLength{}, static_cast<double>(shape[0]) * x_extent[cm],
        YLength{}, static_cast<double>(shape[1]) * y_extent[cm],
        ZLength{}, static_cast<double>(shape[2]) * z_extent[cm],
    };
    return types::MapConfig{bounds, Position3D{}, 1.0 * cm};
}

/**
 * @brief Extract the path from a `comparison_config=<path>` argument.
 * @param arg The raw third CLI argument.
 * @return The `<path>` portion.
 * @throws std::runtime_error if @p arg is not of the form `comparison_config=<path>`.
 */
[[nodiscard]] std::string parseComparisonConfigArg(std::string_view arg) {
    constexpr std::string_view kPrefix = "comparison_config=";
    if (arg.substr(0, kPrefix.size()) != kPrefix) {
        throw std::runtime_error(
            "optional third argument must be of the form comparison_config=<path>");
    }
    return std::string{arg.substr(kPrefix.size())};
}

} // namespace

/**
 * @brief `maps_comparison <origin_map> <target_map> [comparison_config=<path>]`.
 *
 * Prints only the comparison score (0..100) to stdout. With a `comparison_config` the two maps are
 * compared using the per-map offset/boundaries/resolution from that file (which also enables the
 * differing-resolution bonus case, since `MapsComparison::compare` samples each map by world
 * position). Without it, both maps are assumed to share the same geometry, so their shapes must
 * match. On any error, prints `-1` to stdout and a descriptive message to stderr.
 */
int main(int argc, char** argv) {
    if (argc < 3 || argc > 4) {
        std::cout << "-1\n";
        std::cerr << "Usage: maps_comparison <origin_map> <target_map> [comparison_config=<path>]\n";
        return 1;
    }

    try {
        auto origin_arr = Map3DImpl::loadArray(argv[1]);
        auto target_arr = Map3DImpl::loadArray(argv[2]);

        types::MapConfig origin_cfg;
        types::MapConfig target_cfg;
        if (argc == 4) {
            // A comparison_config supplies each map's own geometry, so the shapes may legitimately
            // differ (e.g. same region at different resolutions).
            const ComparisonMapConfigs cfgs = loadComparisonConfig(parseComparisonConfigArg(argv[3]));
            origin_cfg = cfgs.original;
            target_cfg = cfgs.target;
        } else {
            // No config: assume identical geometry (same offset, boundaries, and resolution).
            if (origin_arr->Shape() != target_arr->Shape()) {
                throw std::runtime_error(
                    "origin and target maps have different shapes; pass a comparison_config to "
                    "compare maps with differing geometry.");
            }
            origin_cfg = configForShape(origin_arr->Shape());
            target_cfg = origin_cfg;
        }

        Map3DImpl origin{origin_arr, origin_cfg};
        Map3DImpl target{target_arr, target_cfg};

        const std::vector<IMap3D*> targets{&target};
        const std::vector<double> scores = MapsComparison::compare(origin, targets);

        // Print only the score (no surrounding text) so callers can parse stdout directly.
        std::cout << scores.front() << "\n";
        return 0;
    } catch (const std::exception& error) {
        // Contract: on any failure emit -1 on stdout and a human-readable reason on stderr.
        std::cout << "-1\n";
        std::cerr << error.what() << "\n";
        return 1;
    }
}
