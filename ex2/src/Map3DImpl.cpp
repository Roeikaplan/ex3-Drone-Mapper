#include <drone_mapper/Map3DImpl.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <typeinfo>
#include <utility>

namespace drone_mapper {
namespace {

// Outcome of mapping a world position onto the voxel grid.
struct VoxelIndex {
    bool valid = false;
    std::size_t linear = 0;
};

/**
 * @brief Map a world position to a linear voxel index for a C-order (X, Y, Z) array.
 * @param array Backing NPY array.
 * @param config Map geometry (offset + resolution) relating world cm to voxel indices.
 * @param pos World position in cm.
 * @return A VoxelIndex with valid=true and the linear index, or valid=false when the resolution
 *         is non-positive, the array is not 3-D, or the position falls outside the grid.
 */
[[nodiscard]] VoxelIndex locate(const NpyArray& array,
                                const types::MapConfig& config,
                                const Position3D& pos) {
    const double res_cm = config.resolution.force_numerical_value_in(cm);
    // A zero/negative resolution has no valid grid; guard here so callers never divide by it.
    if (!(res_cm > 0.0)) {
        return {};
    }

    const NpyArray::shape_t& shape = array.Shape();
    if (shape.size() != 3) {
        return {};
    }

    // World -> grid: shift by the array origin (offset), then divide by the voxel edge
    // length. floor selects the cell that *contains* the point. Units are stripped to a
    // plain cm scalar first (force_ bypasses the strict X/Y/Z quantity-spec checks).
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

    // C-order (row-major), axes (X, Y, Z): X is the slowest-varying axis, matching how
    // the .npy maps are authored and how MockLidar samples them.
    const std::size_t linear =
        (static_cast<std::size_t>(ix) * shape[1] + static_cast<std::size_t>(iy)) * shape[2]
        + static_cast<std::size_t>(iz);
    return {true, linear};
}

} // namespace

Map3DImpl::Map3DImpl(std::shared_ptr<NpyArray> map_ptr)
    : Map3DImpl(std::move(map_ptr), types::MapConfig{}) {}

Map3DImpl::Map3DImpl(std::shared_ptr<NpyArray> map_ptr, const types::MapConfig map_config)
    : map_(std::move(map_ptr)),
      config_(map_config) {
    if (!map_) {
        throw std::invalid_argument("Map3DImpl requires a valid map pointer.");
    }
}

types::VoxelOccupancy Map3DImpl::atVoxel(const Position3D& pos) const {
    const VoxelIndex index = locate(*map_, config_, pos);
    if (!index.valid) {
        return types::VoxelOccupancy::OutOfBounds;
    }

    // Voxels are read as signed int8: negatives are our sentinels, while ANY positive
    // value counts as solid. That single rule serves both maps — the hidden map stores
    // Minecraft block ids (1, 2, 3, 18, 45, ...) that must all read as Occupied to match
    // MockLidar's `== Occupied` ray-march, and the output map only ever writes 1 for a hit.
    const auto raw = static_cast<std::int8_t>(map_->Data<std::uint8_t>()[index.linear]);
    if (raw > 0) {
        return types::VoxelOccupancy::Occupied;
    }
    switch (raw) {
    case 0:
        return types::VoxelOccupancy::Empty;
    case -1:
        return types::VoxelOccupancy::Unmapped;
    case -2:
        return types::VoxelOccupancy::OutOfBounds;
    case -3:
        return types::VoxelOccupancy::PotentiallyOccupied;
    default:
        // Unknown negative byte — treat as never-mapped rather than guessing occupancy.
        return types::VoxelOccupancy::Unmapped;
    }
}

types::MapConfig Map3DImpl::getMapConfig() const {
    return config_;
}

bool Map3DImpl::isInBounds(const Position3D& pos) const {
    return locate(*map_, config_, pos).valid;
}

void Map3DImpl::set(const Position3D& pos, types::VoxelOccupancy value) {
    const VoxelIndex index = locate(*map_, config_, pos);
    // Silently ignore out-of-bounds writes; callers (e.g. ScanResultToVoxels) already
    // guard with isInBounds, and there is no valid cell to store into.
    if (!index.valid) {
        return;
    }

    // VoxelOccupancy values live in [-3, 1]; store the signed byte pattern so a later
    // int8 read (in atVoxel, or numpy on a signed dtype) recovers the exact value,
    // negatives included, without colliding with positive block ids.
    map_->Data<std::uint8_t>()[index.linear] =
        static_cast<std::uint8_t>(static_cast<std::int8_t>(value));
}

void Map3DImpl::save(const std::filesystem::path& output_path) const {
    // TinyNPY returns nullptr on success, or an error string on failure.
    if (const char* error = map_->SaveNPY(output_path.string())) {
        throw std::runtime_error(error);
    }
}

std::shared_ptr<NpyArray> Map3DImpl::makeEmptyArray(const types::MapConfig& config) {
    const double res_cm = config.resolution.force_numerical_value_in(cm);
    if (!(res_cm > 0.0)) {
        throw std::invalid_argument("Map3DImpl::makeEmptyArray requires a positive resolution.");
    }

    // Voxel count per axis spans the mapping boundaries; ceil so a partial trailing
    // voxel is still represented. Generic lambda covers the distinct X/Y/Z length types.
    const auto axis_count = [res_cm](auto min, auto max) -> std::size_t {
        const double span_cm = (max - min).force_numerical_value_in(cm);
        const double count = std::ceil(span_cm / res_cm);
        return count > 0.0 ? static_cast<std::size_t>(count) : std::size_t{0};
    };

    const types::MappingBounds& b = config.boundaries;
    const NpyArray::shape_t shape{
        axis_count(b.min_x, b.max_x),
        axis_count(b.min_y, b.max_y),
        axis_count(b.min_height, b.max_height),
    };

    // Use the (shape, wordSize, type) constructor + Allocate() so the array OWNS and
    // frees its buffer. The T* data constructor would leave OwnData()==false (the raw
    // pointer stays unmanaged), which is wrong for a freshly allocated output map.
    const char type_char = NpyArray::GetTypeChar(typeid(std::int8_t));
    auto array = std::make_shared<NpyArray>(shape, sizeof(std::int8_t), type_char);
    array->Allocate();

    // Start every cell Unmapped(-1); scanning later upgrades cells to Empty/Occupied.
    std::int8_t* data = array->Data<std::int8_t>();
    std::fill(data,
              data + array->NumValue(),
              static_cast<std::int8_t>(types::VoxelOccupancy::Unmapped));
    return array;
}

std::shared_ptr<NpyArray> Map3DImpl::loadArray(const std::filesystem::path& path) {
    // Inverse of save(): LoadNPY returns nullptr on success, or an error string on failure.
    auto array = std::make_shared<NpyArray>();
    if (const char* error = array->LoadNPY(path.string())) {
        throw std::runtime_error(error);
    }

    // locate() indexes the buffer as one byte per voxel over a 3-D (X, Y, Z) grid; a
    // differently ranked or wider-dtype array would be silently misread, so reject it up front
    // rather than producing garbage occupancy.
    if (array->Shape().size() != 3) {
        throw std::runtime_error("Map3DImpl::loadArray expects a 3-D .npy array.");
    }
    if (array->SizeValueBytes() != 1) {
        throw std::runtime_error("Map3DImpl::loadArray expects a one-byte voxel dtype.");
    }
    return array;
}

} // namespace drone_mapper
