/**
 * @file Map3DImpl.h
 * @brief The concrete voxel map, backed by a TinyNPY array.
 */

#pragma once

#include <TinyNPY.h>

#include <Common/IMutableMap3D.h>

#include <filesystem>
#include <memory>

namespace simulator {

/**
 * @brief A voxel map bridging world coordinates and a `.npy` grid.
 *
 * Serves both roles in a run: the read-only hidden ground truth and the writable output map. The
 * owned `MapConfig` (offset plus resolution) is what relates continuous centimetres to discrete
 * indices, so two instances over the same array can present different geometry.
 *
 * @note Storage contract: one byte per voxel, interpreted as **signed int8**. That is what lets the
 *       negative sentinels (`Unmapped = -1`, `OutOfBounds = -2`, `PotentiallyOccupied = -3`) coexist
 *       with occupancy in the same array. Any *positive* byte reads back as `Occupied`, which is how
 *       a hidden map full of Minecraft block ids is understood without a translation table.
 * @note Architectural boundary: this lives Simulator-side and never crosses the plugin boundary as a
 *       concrete type. A plugin sees only `IMap3D` / `IMutableMap3D`, and only ever the output map -
 *       the hidden map is reachable from `MockLidar` and `MapsComparison` alone.
 * @note This class throws. `IMutableMap3D::save` returns `void` in the frozen interface, so an
 *       exception is the only channel available for a write failure, and `loadArray` matches it for
 *       consistency. That fits the containment ladder rather than fighting it: the run factory wraps
 *       run creation, logs the failure, and scores the affected group -1.
 */
class Map3DImpl final : public common::IMutableMap3D {
public:
    /**
     * @brief Construct over an array with default (empty) geometry.
     * @param map_ptr Backing array; must not be null.
     * @throws std::invalid_argument when @p map_ptr is null.
     * @note Only useful for a map whose geometry is irrelevant. A hidden map constructed this way
     *       would report zero boundaries, and scoring walks the origin map's boundaries - so an
     *       empty grid there yields a false perfect score.
     */
    explicit Map3DImpl(std::unique_ptr<NpyArray> map_ptr);

    /**
     * @brief Construct over an array with explicit geometry.
     * @param map_ptr Backing array; must not be null.
     * @param map_config Boundaries, offset, and resolution relating world centimetres to indices.
     * @throws std::invalid_argument when @p map_ptr is null.
     */
    Map3DImpl(std::unique_ptr<NpyArray> map_ptr, common::types::MapConfig map_config);

    /**
     * @brief Occupancy of the voxel containing a world position.
     * @param pos World position in centimetres.
     * @return The stored occupancy, or `OutOfBounds` when @p pos falls outside the grid or the map
     *         has no usable resolution.
     */
    [[nodiscard]] common::types::VoxelOccupancy atVoxel(const common::Position3D& pos) const override;

    /**
     * @brief This map's geometry.
     * @return The owned `MapConfig`.
     */
    [[nodiscard]] common::types::MapConfig getMapConfig() const override;

    /**
     * @brief Whether a world position maps to a real cell.
     * @param pos World position in centimetres.
     * @return True when @p pos maps to an index inside the array and the resolution is positive.
     */
    [[nodiscard]] bool isInBounds(const common::Position3D& pos) const override;

    /**
     * @brief Store an occupancy value at a world position.
     * @param pos World position in centimetres.
     * @param value Occupancy to store.
     * @note Out-of-bounds writes are silently ignored: there is no cell to write to, and callers
     *       already guard with `isInBounds` where it matters.
     */
    void set(const common::Position3D& pos, common::types::VoxelOccupancy value) override;

    /**
     * @brief Serialise the map to a `.npy` file.
     * @param path Destination, overwritten if it exists.
     * @throws std::runtime_error when TinyNPY reports a write failure.
     */
    void save(const std::filesystem::path& path) const override;

    /**
     * @brief Allocate an empty output-map array sized from a config.
     * @param config Geometry whose boundaries and resolution set the per-axis voxel counts.
     * @return An owning, C-order, signed-int8 array pre-filled with `Unmapped`.
     * @throws std::invalid_argument when the resolution is not positive.
     * @note Each axis holds `ceil(span / resolution)` cells, keeping a partial trailing voxel.
     *       `MapsComparison` uses the identical formula, and the two must not drift: a mismatch
     *       shifts the scoring grid by one cell at the far edge of every map, which is wrong quietly
     *       rather than obviously.
     */
    [[nodiscard]] static std::unique_ptr<NpyArray> makeEmptyArray(
        const common::types::MapConfig& config);

    /**
     * @brief Read a `.npy` voxel grid from disk.
     * @param path Existing file to read.
     * @return An owning 3-D array ready to wrap.
     * @throws std::runtime_error when the read fails, the array is not 3-D, or its dtype is wider
     *         than one byte.
     * @note The one-byte and rank checks mirror the storage contract that `atVoxel` and `set` rely
     *       on. A wider dtype would be indexed as bytes and silently misread as garbage occupancy,
     *       so it is rejected up front.
     */
    [[nodiscard]] static std::unique_ptr<NpyArray> loadArray(const std::filesystem::path& path);

private:
    std::unique_ptr<NpyArray> map_;
    common::types::MapConfig config_;
};

} // namespace simulator
