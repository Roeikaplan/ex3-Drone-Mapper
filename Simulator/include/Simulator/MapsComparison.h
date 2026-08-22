/**
 * @file MapsComparison.h
 * @brief Scoring of a produced map against ground truth.
 */

#pragma once

#include <Common/IMap3D.h>

namespace simulator {

/**
 * @brief Scores one map against another by occupied-voxel agreement.
 *
 * @note Architectural boundary: scoring is Simulator-side and runs only after a mission ends. It is
 *       the one place besides `MockLidar` that reads the hidden map, and neither ever exposes it to
 *       a plugin - which is what makes it impossible for a third-party MissionControl to write a
 *       perfect map without flying.
 */
class MapsComparison {
public:
    /**
     * @brief Compare a produced map against ground truth.
     * @param origin The reference map; its grid defines what gets sampled.
     * @param target The map being scored.
     * @return A score in [0, 100]; 0 when the origin has no usable resolution.
     *
     * @note Both maps are sampled **by world position** at the origin's voxel centres, not by
     *       matching indices. That is what lets a target at a different resolution or offset be
     *       scored correctly rather than compared cell-for-cell against a grid it does not share.
     * @note The metric is occupied-voxel IoU: intersection over union across cells `Occupied` in
     *       either map. Empty-to-empty agreement is deliberately excluded - most of a voxel world is
     *       empty, so counting it would push every score toward 100 and destroy the metric's ability
     *       to rank algorithms, which is the one thing competitive mode needs from it.
     * @note Two maps with no occupied voxels anywhere score 100: they are trivially identical, and
     *       reporting 0 would punish a correct result on an empty world.
     */
    [[nodiscard]] static double compare(const common::IMap3D& origin, const common::IMap3D& target);
};

} // namespace simulator
