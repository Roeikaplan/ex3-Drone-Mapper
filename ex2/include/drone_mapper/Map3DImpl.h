#pragma once

#include <TinyNPY.h>

#include <drone_mapper/IMutableMap3D.h>

#include <filesystem>
#include <memory>

namespace drone_mapper {

/**
 * @brief Concrete voxel map backed by a TinyNPY array; implements `IMutableMap3D`.
 *
 * Bridges continuous world coordinates (cm) and the discrete `.npy` voxel grid using
 * the owned `MapConfig` (offset + resolution). Serves both roles in the simulator:
 * the read-only hidden ground-truth map and the writable output map.
 *
 * Voxels are stored one-byte-per-cell and interpreted as **signed int8** so the
 * output map's negative sentinels (`Unmapped=-1`, `OutOfBounds=-2`,
 * `PotentiallyOccupied=-3`) coexist with positive occupancy values without clashing
 * with the hidden map's positive Minecraft block IDs (all read back as `Occupied`).
 */
class Map3DImpl final : public IMutableMap3D {
public:
    /**
     * @brief Construct with a default (empty) `MapConfig`.
     * @param map_ptr Non-null shared NPY array; throws `std::invalid_argument` if null.
     */
    Map3DImpl(std::shared_ptr<NpyArray> map_ptr);
    /**
     * @brief Construct with explicit map geometry.
     * @param map_ptr Non-null shared NPY array; throws `std::invalid_argument` if null.
     * @param map_config Offset, resolution, and boundaries relating world cm to voxel indices.
     */
    Map3DImpl(std::shared_ptr<NpyArray> map_ptr, const types::MapConfig map_config);

    /**
     * @brief Look up the occupancy of the voxel containing a world position.
     * @param pos World position in cm.
     * @return The stored `VoxelOccupancy`; `OutOfBounds` if `pos` maps outside the
     *         array (or the map has zero resolution). Any positive stored byte reads
     *         back as `Occupied`.
     */
    [[nodiscard]] types::VoxelOccupancy atVoxel(const Position3D& pos) const override;
    /**
     * @brief Access the map geometry (boundaries, offset, resolution).
     * @return The owned `MapConfig`.
     */
    [[nodiscard]] types::MapConfig getMapConfig() const override;
    /**
     * @brief Test whether a world position maps to a valid, writable voxel.
     * @param pos World position in cm.
     * @return `true` iff `pos` maps to an index inside the underlying array (and
     *         resolution is positive). Used by callers to guard `set`/reads.
     */
    [[nodiscard]] bool isInBounds(const Position3D& pos) const override;

    //Mutable map methods
    /**
     * @brief Write an occupancy value into the voxel containing a world position.
     * @param pos World position in cm; out-of-bounds writes are ignored (no-op).
     * @param value Occupancy to store (encoded as signed int8).
     */
    void set(const Position3D& pos, types::VoxelOccupancy value) override;
    /**
     * @brief Serialise the map to a `.npy` file.
     * @param output_path Destination path (overwritten if it exists).
     * @throws std::runtime_error if TinyNPY reports a write failure.
     */
    void save(const std::filesystem::path& output_path) const override;

    /**
     * @brief Allocate an empty output-map array sized from a `MapConfig`.
     * @param config Geometry whose boundaries + resolution set the per-axis voxel counts.
     * @return An owning, C-order, signed-int8 `NpyArray` pre-filled with `Unmapped` (-1),
     *         ready to wrap in a `Map3DImpl` for a writable output map.
     */
    [[nodiscard]] static std::shared_ptr<NpyArray> makeEmptyArray(const types::MapConfig& config);

    /**
     * @brief Load a `.npy` voxel grid from disk into an owning array.
     * @param path Existing `.npy` file to read.
     * @return A shared, 3-D `NpyArray` ready to wrap in a `Map3DImpl`.
     * @throws std::runtime_error if TinyNPY reports a read failure, the array is not 3-D, or
     *         its dtype is wider than one byte.
     * @note Inverse of `save()`. The one-byte constraint mirrors the storage contract that
     *       `atVoxel`/`set` rely on: voxels are indexed one byte per cell, so a wider dtype
     *       would be silently misread.
     */
    [[nodiscard]] static std::shared_ptr<NpyArray> loadArray(const std::filesystem::path& path);

private:
    // Changed: shared ownership supports the new pointer-based storage member.
    std::shared_ptr<NpyArray> map_;
    // Changed: replaces standalone resolution_ so all map geometry stays together.
    types::MapConfig config_;
};

} // namespace drone_mapper
