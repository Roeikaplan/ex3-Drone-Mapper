#pragma once

#include <drone_mapper/Types.h>

namespace drone_mapper {

// Read-only 3D occupancy map interface used by LiDAR implementations.
// **Do not change this interface.**
class IMap3D {
public:
    virtual ~IMap3D() = default;

    // Changed: renamed get() to atVoxel() to make it clear callers query a voxel by world position.
    [[nodiscard]] virtual types::VoxelOccupancy atVoxel(const Position3D& pos) const = 0;
    // Changed: map resolution/bounds/offset now travel together so offset-aware maps expose one config object.
    [[nodiscard]] virtual types::MapConfig getMapConfig() const = 0;
    [[nodiscard]] virtual bool isInBounds(const Position3D& pos) const = 0; 
};

} // namespace drone_mapper
