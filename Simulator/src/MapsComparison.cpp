/**
 * @file MapsComparison.cpp
 * @brief Occupied-voxel intersection-over-union between two maps.
 */

#include <Simulator/MapsComparison.h>

#include <cmath>
#include <cstddef>

namespace simulator {
namespace {

using common::cm;
using common::x_extent;
using common::y_extent;
using common::z_extent;

/**
 * @brief Voxel count along one axis.
 * @param span_cm Axis length in centimetres.
 * @param res_cm Voxel edge length in centimetres; the caller guarantees it is positive.
 * @return Whole voxels spanning the axis, keeping a partial trailing cell; 0 for a non-positive span.
 * @note Mirrors `Map3DImpl::makeEmptyArray` exactly. The two formulas are coupled on purpose: if
 *       they diverge, this walks a grid one cell different from the one the map stores and every
 *       score is quietly wrong at the far edge.
 */
[[nodiscard]] std::size_t axisCount(double span_cm, double res_cm) {
    const double count = std::ceil(span_cm / res_cm);
    return count > 0.0 ? static_cast<std::size_t>(count) : std::size_t{0};
}

} // namespace

/**
 * @brief Compare a produced map against ground truth.
 * @param origin The reference map; its grid defines what gets sampled.
 * @param target The map being scored.
 * @return A score in [0, 100].
 * @note Sampling at the voxel *centre* - half a cell past the corner - is what makes the floor
 *       inside `atVoxel` land squarely on the intended cell rather than on a boundary, where
 *       rounding could pick either neighbour.
 */
double MapsComparison::compare(const common::IMap3D& origin, const common::IMap3D& target) {
    const common::types::MapConfig config = origin.getMapConfig();
    const double res_cm = config.resolution.force_numerical_value_in(cm);

    /**
     * @note Without a positive resolution there is no grid to walk. Reporting the floor score beats
     *       dividing by zero, and a map in that state is misconfigured rather than merely bad.
     */
    if (!(res_cm > 0.0)) {
        return 0.0;
    }

    const common::types::MappingBounds& bounds = config.boundaries;
    const std::size_t nx =
        axisCount((bounds.max_x - bounds.min_x).force_numerical_value_in(cm), res_cm);
    const std::size_t ny =
        axisCount((bounds.max_y - bounds.min_y).force_numerical_value_in(cm), res_cm);
    const std::size_t nz =
        axisCount((bounds.max_height - bounds.min_height).force_numerical_value_in(cm), res_cm);

    std::size_t intersection = 0;
    std::size_t union_count = 0;

    for (std::size_t i = 0; i < nx; ++i) {
        for (std::size_t j = 0; j < ny; ++j) {
            for (std::size_t k = 0; k < nz; ++k) {
                const common::Position3D centre{
                    config.offset.x + (static_cast<double>(i) + 0.5) * res_cm * x_extent[cm],
                    config.offset.y + (static_cast<double>(j) + 0.5) * res_cm * y_extent[cm],
                    config.offset.z + (static_cast<double>(k) + 0.5) * res_cm * z_extent[cm],
                };

                const bool origin_occupied =
                    origin.atVoxel(centre) == common::types::VoxelOccupancy::Occupied;
                const bool target_occupied =
                    target.atVoxel(centre) == common::types::VoxelOccupancy::Occupied;

                if (origin_occupied || target_occupied) {
                    ++union_count;
                    if (origin_occupied && target_occupied) {
                        ++intersection;
                    }
                }
            }
        }
    }

    /**
     * @note An empty union means neither map has a single occupied voxel, which makes them
     *       identical. Scoring that 100 rather than 0 avoids punishing a correct result on a world
     *       with nothing in it.
     */
    if (union_count == 0) {
        return 100.0;
    }
    return 100.0 * static_cast<double>(intersection) / static_cast<double>(union_count);
}

} // namespace simulator
