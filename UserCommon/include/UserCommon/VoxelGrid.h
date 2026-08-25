/**
 * @file VoxelGrid.h
 * @brief The one definition of how a `MapConfig` becomes a grid of voxels.
 *
 * @note Architectural boundary: shared by all three projects, which is why it lives in
 *       `UserCommon/` rather than in any one of them. The `ceil(span / resolution)` axis count in
 *       particular had grown three separate copies - the map allocator, the scorer, and the
 *       planner - and a divergence between any two of them shifts the grid by one cell at the far
 *       edge of every map. That failure is silent: scores come out slightly wrong rather than
 *       obviously broken.
 * @note Header-only, so `UserCommon/` needs no build file - matching how the assignment describes
 *       the folder. Consumers add `UserCommon/include` to their include path and nothing else.
 * @note This describes a grid derived from a **config**. It deliberately says nothing about how any
 *       particular map stores its cells: `Map3DImpl` indexes its backing array by that array's own
 *       shape, which is a different question and stays where it is.
 */

#pragma once

#include <Common/Types.h>

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <optional>

namespace user_common {

/**
 * @brief A voxel's integer coordinate within a grid.
 */
struct VoxelIndex {
    std::int64_t x = 0;
    std::int64_t y = 0;
    std::int64_t z = 0;

    /**
     * @brief Whether two indices refer to the same voxel.
     * @param other Index to compare against.
     * @return True when every component matches.
     */
    [[nodiscard]] constexpr bool operator==(const VoxelIndex& other) const noexcept {
        return x == other.x && y == other.y && z == other.z;
    }
};

/**
 * @brief The grid of voxels a `MapConfig` describes.
 *
 * @note Every axis holds `ceil(span / resolution)` cells, so a partial trailing voxel is kept rather
 *       than dropped. Dropping it would make the far edge of a mission region unmappable and
 *       unscoreable.
 * @note A grid built from a config with a non-positive resolution is **unusable**: it reports zero
 *       cells and rejects every index, rather than dividing by zero. Callers check `usable()` when
 *       they need to distinguish "misconfigured" from "empty".
 */
class VoxelGrid {
public:
    /**
     * @brief Derive a grid from a map configuration.
     * @param config Boundaries, offset, and resolution.
     * @return The grid it describes; unusable when the resolution is not positive.
     */
    [[nodiscard]] static VoxelGrid from(const common::types::MapConfig& config) {
        VoxelGrid grid{};
        grid.resolution_cm_ = config.resolution.force_numerical_value_in(common::cm);
        if (!(grid.resolution_cm_ > 0.0)) {
            grid.resolution_cm_ = 0.0;
            return grid;
        }

        grid.origin_x_cm_ = config.offset.x.force_numerical_value_in(common::cm);
        grid.origin_y_cm_ = config.offset.y.force_numerical_value_in(common::cm);
        grid.origin_z_cm_ = config.offset.z.force_numerical_value_in(common::cm);

        const common::types::MappingBounds& bounds = config.boundaries;
        grid.nx_ = axisCount((bounds.max_x - bounds.min_x).force_numerical_value_in(common::cm),
                             grid.resolution_cm_);
        grid.ny_ = axisCount((bounds.max_y - bounds.min_y).force_numerical_value_in(common::cm),
                             grid.resolution_cm_);
        grid.nz_ = axisCount(
            (bounds.max_height - bounds.min_height).force_numerical_value_in(common::cm),
            grid.resolution_cm_);
        return grid;
    }

    /**
     * @brief Whether this grid has a usable resolution.
     * @return True when the resolution is positive.
     */
    [[nodiscard]] bool usable() const noexcept { return resolution_cm_ > 0.0; }

    /**
     * @brief Voxel edge length.
     * @return The resolution in centimetres, or 0 for an unusable grid.
     */
    [[nodiscard]] double resolutionCm() const noexcept { return resolution_cm_; }

    /**
     * @brief Cells along the X axis.
     * @return The count.
     */
    [[nodiscard]] std::int64_t sizeX() const noexcept { return nx_; }

    /**
     * @brief Cells along the Y axis.
     * @return The count.
     */
    [[nodiscard]] std::int64_t sizeY() const noexcept { return ny_; }

    /**
     * @brief Cells along the Z axis.
     * @return The count.
     */
    [[nodiscard]] std::int64_t sizeZ() const noexcept { return nz_; }

    /**
     * @brief Total cells in the grid.
     * @return The product of the three axis counts.
     * @note Used to size flat lookup arrays, which is why it is a `size_t` rather than signed.
     */
    [[nodiscard]] std::size_t cellCount() const noexcept {
        return static_cast<std::size_t>(nx_) * static_cast<std::size_t>(ny_) *
               static_cast<std::size_t>(nz_);
    }

    /**
     * @brief Whether an index lies inside the grid.
     * @param index Voxel coordinate to test.
     * @return True when every component is within its axis range.
     */
    [[nodiscard]] bool contains(const VoxelIndex& index) const noexcept {
        return index.x >= 0 && index.x < nx_ && index.y >= 0 && index.y < ny_ && index.z >= 0 &&
               index.z < nz_;
    }

    /**
     * @brief Flatten an index for use as an array subscript.
     * @param index Voxel coordinate; must be inside the grid.
     * @return The linear position, with X the slowest-varying axis.
     * @note Matches the C-order layout the `.npy` maps use, so a flat array indexed this way walks
     *       memory in the same direction the stored map does.
     */
    [[nodiscard]] std::size_t linearIndex(const VoxelIndex& index) const noexcept {
        return (static_cast<std::size_t>(index.x) * static_cast<std::size_t>(ny_) +
                static_cast<std::size_t>(index.y)) *
                   static_cast<std::size_t>(nz_) +
               static_cast<std::size_t>(index.z);
    }

    /**
     * @brief Recover an index from its flattened form.
     * @param linear A value previously produced by `linearIndex`.
     * @return The voxel coordinate it came from.
     * @note The inverse of `linearIndex`, and the reason a breadth-first search can store parents as
     *       plain integers rather than as three-component keys - a third of the memory and no
     *       comparator.
     */
    [[nodiscard]] VoxelIndex indexFromLinear(std::size_t linear) const noexcept {
        const auto nz = static_cast<std::size_t>(nz_);
        const auto ny = static_cast<std::size_t>(ny_);
        const std::size_t z = linear % nz;
        const std::size_t rest = linear / nz;
        return VoxelIndex{static_cast<std::int64_t>(rest / ny),
                          static_cast<std::int64_t>(rest % ny), static_cast<std::int64_t>(z)};
    }

    /**
     * @brief World position of a voxel's centre.
     * @param index Voxel coordinate.
     * @return The centre, with the per-axis quantity specs re-attached.
     * @note The centre rather than a corner, because sampling a map at a cell boundary can round
     *       into either neighbour. `indexOf(centreOf(v)) == v` for any in-range `v`.
     */
    [[nodiscard]] common::Position3D centreOf(const VoxelIndex& index) const {
        const double x = origin_x_cm_ + (static_cast<double>(index.x) + 0.5) * resolution_cm_;
        const double y = origin_y_cm_ + (static_cast<double>(index.y) + 0.5) * resolution_cm_;
        const double z = origin_z_cm_ + (static_cast<double>(index.z) + 0.5) * resolution_cm_;
        return common::Position3D{x * common::x_extent[common::cm],
                                  y * common::y_extent[common::cm],
                                  z * common::z_extent[common::cm]};
    }

    /**
     * @brief The voxel containing a world position.
     * @param position World position in centimetres.
     * @return Its index, or `nullopt` when the position lies outside the grid or the grid is
     *         unusable.
     * @note `floor` selects the cell that *contains* the point rather than the nearest cell centre.
     */
    [[nodiscard]] std::optional<VoxelIndex> indexOf(const common::Position3D& position) const {
        if (!usable()) {
            return std::nullopt;
        }

        const VoxelIndex index{
            static_cast<std::int64_t>(std::floor(
                (position.x.force_numerical_value_in(common::cm) - origin_x_cm_) / resolution_cm_)),
            static_cast<std::int64_t>(std::floor(
                (position.y.force_numerical_value_in(common::cm) - origin_y_cm_) / resolution_cm_)),
            static_cast<std::int64_t>(std::floor(
                (position.z.force_numerical_value_in(common::cm) - origin_z_cm_) / resolution_cm_)),
        };
        return contains(index) ? std::optional<VoxelIndex>{index} : std::nullopt;
    }

    /**
     * @brief Cells spanning one axis.
     * @param span_cm Axis length in centimetres.
     * @param resolution_cm Voxel edge length; the caller guarantees it is positive.
     * @return Whole cells covering the span, keeping a partial trailing one; 0 for a non-positive
     *         span.
     * @note **This is the formula that must not be duplicated.** Every grid in the project - the
     *       allocated output map, the scoring sweep, the planner's search space - derives its extent
     *       from here.
     */
    [[nodiscard]] static std::int64_t axisCount(double span_cm, double resolution_cm) {
        const double count = std::ceil(span_cm / resolution_cm);
        return count > 0.0 ? static_cast<std::int64_t>(count) : std::int64_t{0};
    }

private:
    double resolution_cm_ = 0.0;
    double origin_x_cm_ = 0.0;
    double origin_y_cm_ = 0.0;
    double origin_z_cm_ = 0.0;
    std::int64_t nx_ = 0;
    std::int64_t ny_ = 0;
    std::int64_t nz_ = 0;
};

} // namespace user_common
