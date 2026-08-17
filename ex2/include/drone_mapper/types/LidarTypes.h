#pragma once

#include <drone_mapper/Units.h>

#include <cstddef>
#include <vector>

namespace drone_mapper::types {

struct LidarConfigData {
    PhysicalLength z_min{};
    PhysicalLength z_max{};
    PhysicalLength d{};
    std::size_t fov_circles = 0;
};

struct LidarHit {
    // Misses use max double centimeters;
    PhysicalLength distance{};
    Orientation angle{}; // relative angle 
};

using LidarScanResult = std::vector<LidarHit>;

} // namespace drone_mapper::types
