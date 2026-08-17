#pragma once

#include <drone_mapper/Units.h>

namespace drone_mapper::types {

enum class VoxelOccupancy {
    PotentiallyOccupied = -3,
    OutOfBounds = -2,
    Unmapped = -1,
    Empty = 0,
    Occupied = 1,
};

// Changed: moved from mission types because map bounds now belong to map configuration.
struct MappingBounds {
    XLength min_x{};
    XLength max_x{};
    YLength min_y{};
    YLength max_y{};
    ZLength min_height{};
    ZLength max_height{};
};

struct MappedVoxel {
    Position3D position{};
    VoxelOccupancy value = VoxelOccupancy::Unmapped;
};

// Changed: added to keep boundaries, offset, and resolution together on IMap3D.
// Note - IMap3D should be able to construct with a default MapConfig (aka no MapConfig. See Map3DImpl.h).
struct MapConfig {
    MappingBounds boundaries{};
    Position3D offset{};
    PhysicalLength resolution{};
};
} // namespace drone_mapper::types
