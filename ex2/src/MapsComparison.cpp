#include <drone_mapper/MapsComparison.h>

#include <cmath>
#include <cstddef>
#include <vector>

namespace drone_mapper {
namespace {

/**
 * @brief Voxel count along one axis: the mapped span divided by the voxel edge, rounded up.
 * @param span_cm Axis length (max - min) in cm.
 * @param res_cm Voxel edge length in cm; assumed positive (the caller guards this).
 * @return Number of whole voxels spanning the axis, keeping a partial trailing cell; 0 when the
 *         span is non-positive.
 * @note Mirrors the axis sizing in `Map3DImpl::makeEmptyArray` so `compare()` walks exactly the
 *       grid the map stores.
 */
[[nodiscard]] std::size_t axisCount(double span_cm, double res_cm) {
    const double count = std::ceil(span_cm / res_cm);
    return count > 0.0 ? static_cast<std::size_t>(count) : std::size_t{0};
}

} // namespace

std::vector<double> MapsComparison::compare(const IMap3D& origin,
                                            const std::vector<IMap3D*> targets) {
    const types::MapConfig cfg = origin.getMapConfig();
    const double res_cm = cfg.resolution.force_numerical_value_in(cm);

    std::vector<double> scores;
    scores.reserve(targets.size());
    // Without a positive resolution there is no grid to walk; report the floor score for
    // every target rather than dividing by zero.
    if (!(res_cm > 0.0)) {
        scores.assign(targets.size(), 0.0);
        return scores;
    }

    // The comparison is evaluated on the ORIGIN's grid: we walk the origin's voxels and sample each
    // map by world position (atVoxel converts world -> that map's own voxel index). This means a
    // TARGET at a different resolution or offset is supported positionally (the "different resolution
    // requests" bonus, see bonus.txt) — a finer target is just sampled at the origin's cell centres.
    const types::MappingBounds& b = cfg.boundaries;
    const std::size_t nx = axisCount((b.max_x - b.min_x).force_numerical_value_in(cm), res_cm);
    const std::size_t ny = axisCount((b.max_y - b.min_y).force_numerical_value_in(cm), res_cm);
    const std::size_t nz = axisCount((b.max_height - b.min_height).force_numerical_value_in(cm), res_cm);

    for (const IMap3D* target : targets) {
        // Occupied-voxel IoU (Jaccard): intersection / union over cells Occupied in either
        // map. Empty-vs-Empty agreement is deliberately excluded so a mostly-empty map cannot
        // inflate the score toward 100.
        std::size_t intersection = 0;
        std::size_t union_count = 0;

        for (std::size_t i = 0; i < nx; ++i) {
            for (std::size_t j = 0; j < ny; ++j) {
                for (std::size_t k = 0; k < nz; ++k) {
                    // Sample the voxel centre: (index + 0.5) cells past the offset so
                    // atVoxel's floor lands squarely on cell (i, j, k). Re-attach the X/Y/Z
                    // quantity specs after forming the plain cm scalar.
                    const Position3D pos{
                        cfg.offset.x + (static_cast<double>(i) + 0.5) * res_cm * x_extent[cm],
                        cfg.offset.y + (static_cast<double>(j) + 0.5) * res_cm * y_extent[cm],
                        cfg.offset.z + (static_cast<double>(k) + 0.5) * res_cm * z_extent[cm],
                    };

                    const bool origin_occ =
                        origin.atVoxel(pos) == types::VoxelOccupancy::Occupied;
                    const bool target_occ =
                        target != nullptr &&
                        target->atVoxel(pos) == types::VoxelOccupancy::Occupied;

                    if (origin_occ || target_occ) {
                        ++union_count;
                        if (origin_occ && target_occ) {
                            ++intersection;
                        }
                    }
                }
            }
        }

        // Two maps with no occupied voxels are trivially identical -> perfect score.
        const double score = union_count == 0
                                 ? 100.0
                                 : 100.0 * static_cast<double>(intersection) /
                                       static_cast<double>(union_count);
        scores.push_back(score);
    }

    return scores;
}

} // namespace drone_mapper
