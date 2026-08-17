#pragma once

#include <drone_mapper/IMap3D.h>
#include <drone_mapper/Types.h>

namespace drone_mapper {

class MapsComparison {
public:
    [[nodiscard]] static std::vector<double> compare(const IMap3D& origin,
                                                     const std::vector<IMap3D*> targets); //currently should work with at least 1 target
};

} // namespace drone_mapper
