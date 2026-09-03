/**
 * @file MapsComparison.cpp
 * @brief Occupied-voxel intersection-over-union between two maps.
 */

#include <Simulator/MapsComparison.h>

#include <UserCommon/VoxelGrid.h>

#include <cstddef>
#include <cstdint>

namespace simulator {

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
    const user_common_323998450_211633813::VoxelGrid grid = user_common_323998450_211633813::VoxelGrid::from(origin.getMapConfig());

    /**
     * @note Without a positive resolution there is no grid to walk. Reporting the floor score beats
     *       dividing by zero, and a map in that state is misconfigured rather than merely bad.
     */
    if (!grid.usable()) {
        return 0.0;
    }

    std::size_t intersection = 0;
    std::size_t union_count = 0;

    for (std::int64_t i = 0; i < grid.sizeX(); ++i) {
        for (std::int64_t j = 0; j < grid.sizeY(); ++j) {
            for (std::int64_t k = 0; k < grid.sizeZ(); ++k) {
                const common::Position3D centre = grid.centreOf(user_common_323998450_211633813::VoxelIndex{i, j, k});

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
