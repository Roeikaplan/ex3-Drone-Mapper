/**
 * @file Map3DImpl.cpp
 * @brief World-to-voxel geometry, signed-byte storage, and `.npy` serialisation.
 * @note The unit-stripping idiom appears at every geometry site: convert to a plain centimetre
 *       scalar with `force_numerical_value_in(cm)`, do the arithmetic in doubles, and re-attach the
 *       axis quantity spec on the way out. `force_` is what bypasses the strict X/Y/Z spec checks
 *       that otherwise make mixed-axis arithmetic a compile error.
 */

#include <Simulator/Map3DImpl.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <typeinfo>
#include <utility>

namespace simulator {
namespace {

using common::cm;

/**
 * @brief The outcome of mapping a world position onto the voxel grid.
 */
struct VoxelIndex {
    bool valid = false;
    std::size_t linear = 0;
};

/**
 * @brief Map a world position to a linear index into a C-order (X, Y, Z) array.
 * @param array Backing array, consulted for its shape.
 * @param config Geometry relating world centimetres to indices.
 * @param pos World position in centimetres.
 * @return A valid index, or `valid == false` when the resolution is non-positive, the array is not
 *         3-D, or the position falls outside the grid.
 * @note One function serves reads, writes, and the bounds check, so those three can never disagree
 *       about which cell a position belongs to.
 * @note `floor` rather than rounding: the cell that *contains* the point is wanted, not the nearest
 *       cell centre.
 * @note C-order with X slowest-varying, matching how the shipped `.npy` maps are authored and how
 *       `MockLidar` walks them.
 */
[[nodiscard]] VoxelIndex locate(const NpyArray& array, const common::types::MapConfig& config,
                                const common::Position3D& pos) {
    const double res_cm = config.resolution.force_numerical_value_in(cm);
    if (!(res_cm > 0.0)) {
        return {};
    }

    const NpyArray::shape_t& shape = array.Shape();
    if (shape.size() != 3) {
        return {};
    }

    const double gx = (pos.x - config.offset.x).force_numerical_value_in(cm) / res_cm;
    const double gy = (pos.y - config.offset.y).force_numerical_value_in(cm) / res_cm;
    const double gz = (pos.z - config.offset.z).force_numerical_value_in(cm) / res_cm;

    const double ix = std::floor(gx);
    const double iy = std::floor(gy);
    const double iz = std::floor(gz);

    const double nx = static_cast<double>(shape[0]);
    const double ny = static_cast<double>(shape[1]);
    const double nz = static_cast<double>(shape[2]);
    if (ix < 0.0 || iy < 0.0 || iz < 0.0 || ix >= nx || iy >= ny || iz >= nz) {
        return {};
    }

    const std::size_t linear =
        (static_cast<std::size_t>(ix) * shape[1] + static_cast<std::size_t>(iy)) * shape[2] +
        static_cast<std::size_t>(iz);
    return {true, linear};
}

} // namespace

/**
 * @brief Construct over an array with default (empty) geometry.
 * @param map_ptr Backing array; must not be null.
 */
Map3DImpl::Map3DImpl(std::shared_ptr<NpyArray> map_ptr)
    : Map3DImpl(std::move(map_ptr), common::types::MapConfig{}) {}

/**
 * @brief Construct over an array with explicit geometry.
 * @param map_ptr Backing array; must not be null.
 * @param map_config Boundaries, offset, and resolution.
 * @throws std::invalid_argument when @p map_ptr is null.
 * @note Rejecting null here rather than tolerating it keeps every other method free of a null check,
 *       and a null array is a wiring bug rather than a recoverable input problem.
 */
Map3DImpl::Map3DImpl(std::shared_ptr<NpyArray> map_ptr, common::types::MapConfig map_config)
    : map_(std::move(map_ptr)), config_(map_config) {
    if (!map_) {
        throw std::invalid_argument("Map3DImpl requires a non-null map array.");
    }
}

/**
 * @brief Occupancy of the voxel containing a world position.
 * @param pos World position in centimetres.
 * @return The stored occupancy, or `OutOfBounds` outside the grid.
 * @note Any positive byte means solid. That one rule serves both maps: the hidden map stores
 *       Minecraft block ids (1, 2, 3, 18, 45, ...) which must all read as `Occupied` for
 *       `MockLidar`'s ray march, while the output map only ever writes 1.
 * @note An unrecognised negative byte reads as `Unmapped` rather than being guessed at - claiming
 *       occupancy from a byte we do not understand would corrupt scoring.
 */
common::types::VoxelOccupancy Map3DImpl::atVoxel(const common::Position3D& pos) const {
    const VoxelIndex index = locate(*map_, config_, pos);
    if (!index.valid) {
        return common::types::VoxelOccupancy::OutOfBounds;
    }

    const auto raw = static_cast<std::int8_t>(map_->Data<std::uint8_t>()[index.linear]);
    if (raw > 0) {
        return common::types::VoxelOccupancy::Occupied;
    }
    switch (raw) {
    case 0:
        return common::types::VoxelOccupancy::Empty;
    case -1:
        return common::types::VoxelOccupancy::Unmapped;
    case -2:
        return common::types::VoxelOccupancy::OutOfBounds;
    case -3:
        return common::types::VoxelOccupancy::PotentiallyOccupied;
    default:
        return common::types::VoxelOccupancy::Unmapped;
    }
}

/**
 * @brief This map's geometry.
 * @return The owned `MapConfig`.
 */
common::types::MapConfig Map3DImpl::getMapConfig() const {
    return config_;
}

/**
 * @brief Whether a world position maps to a real cell.
 * @param pos World position in centimetres.
 * @return True when the position resolves to an index inside the array.
 */
bool Map3DImpl::isInBounds(const common::Position3D& pos) const {
    return locate(*map_, config_, pos).valid;
}

/**
 * @brief Store an occupancy value at a world position.
 * @param pos World position in centimetres.
 * @param value Occupancy to store.
 * @note The signed byte pattern is stored, so a later int8 read - here or in numpy on a signed
 *       dtype - recovers the exact value including negatives, without colliding with the positive
 *       block ids a hidden map uses.
 */
void Map3DImpl::set(const common::Position3D& pos, common::types::VoxelOccupancy value) {
    const VoxelIndex index = locate(*map_, config_, pos);
    if (!index.valid) {
        return;
    }

    map_->Data<std::uint8_t>()[index.linear] =
        static_cast<std::uint8_t>(static_cast<std::int8_t>(value));
}

/**
 * @brief Serialise the map to a `.npy` file.
 * @param path Destination, overwritten if it exists.
 * @throws std::runtime_error when TinyNPY reports a write failure.
 * @note TinyNPY signals success with a null pointer and failure with an error string, which reads
 *       backwards compared with most APIs.
 */
void Map3DImpl::save(const std::filesystem::path& path) const {
    if (const char* error = map_->SaveNPY(path.string())) {
        throw std::runtime_error(error);
    }
}

/**
 * @brief Allocate an empty output-map array sized from a config.
 * @param config Geometry whose boundaries and resolution set the per-axis voxel counts.
 * @return An owning, C-order, signed-int8 array pre-filled with `Unmapped`.
 * @throws std::invalid_argument when the resolution is not positive.
 * @note The `(shape, wordSize, type)` constructor plus `Allocate()` is used deliberately: the
 *       `T* data` constructor leaves `OwnData() == false`, so the array would not free its buffer -
 *       wrong for one it just allocated.
 * @note Every cell starts `Unmapped`. Scanning upgrades cells to `Empty` or `Occupied`, so a cell
 *       still reading `Unmapped` at the end genuinely was never observed.
 */
std::shared_ptr<NpyArray> Map3DImpl::makeEmptyArray(const common::types::MapConfig& config) {
    const double res_cm = config.resolution.force_numerical_value_in(cm);
    if (!(res_cm > 0.0)) {
        throw std::invalid_argument("Map3DImpl::makeEmptyArray requires a positive resolution.");
    }

    /**
     * @note `ceil` keeps a partial trailing voxel. `MapsComparison` repeats this formula exactly;
     *       if the two ever diverge the scoring grid shifts by a cell at the far edge of the map.
     */
    const auto axis_count = [res_cm](auto min, auto max) -> std::size_t {
        const double span_cm = (max - min).force_numerical_value_in(cm);
        const double count = std::ceil(span_cm / res_cm);
        return count > 0.0 ? static_cast<std::size_t>(count) : std::size_t{0};
    };

    const common::types::MappingBounds& bounds = config.boundaries;
    const NpyArray::shape_t shape{
        axis_count(bounds.min_x, bounds.max_x),
        axis_count(bounds.min_y, bounds.max_y),
        axis_count(bounds.min_height, bounds.max_height),
    };

    const char type_char = NpyArray::GetTypeChar(typeid(std::int8_t));
    auto array = std::make_shared<NpyArray>(shape, sizeof(std::int8_t), type_char);
    array->Allocate();

    std::int8_t* data = array->Data<std::int8_t>();
    std::fill(data, data + array->NumValue(),
              static_cast<std::int8_t>(common::types::VoxelOccupancy::Unmapped));
    return array;
}

/**
 * @brief Read a `.npy` voxel grid from disk.
 * @param path Existing file to read.
 * @return An owning 3-D array ready to wrap.
 * @throws std::runtime_error when the read fails, the array is not 3-D, or its dtype is wider than
 *         one byte.
 * @note The rank and width checks are up front rather than at first access. `locate` indexes the
 *       buffer as one byte per voxel over a 3-D grid, so a wider dtype would not fail - it would
 *       quietly produce garbage occupancy for every cell.
 */
std::shared_ptr<NpyArray> Map3DImpl::loadArray(const std::filesystem::path& path) {
    auto array = std::make_shared<NpyArray>();
    if (const char* error = array->LoadNPY(path.string())) {
        throw std::runtime_error(error);
    }

    if (array->Shape().size() != 3) {
        throw std::runtime_error("Map3DImpl::loadArray expects a 3-D .npy array: " + path.string());
    }
    if (array->SizeValueBytes() != 1) {
        throw std::runtime_error("Map3DImpl::loadArray expects a one-byte voxel dtype: " +
                                 path.string());
    }
    return array;
}

} // namespace simulator
