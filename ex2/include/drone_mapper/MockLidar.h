#pragma once

#include <drone_mapper/IGPS.h>
#include <drone_mapper/ILidar.h>
#include <drone_mapper/IMap3D.h>

namespace drone_mapper {

class MockLidar final : public ILidar {
public:
    MockLidar(types::LidarConfigData config, const IMap3D& map, const IGPS& gps);

    [[nodiscard]] types::LidarScanResult scan(Orientation scan_orientation) const override;
    // Change: 20.6 - added a config getter
    [[nodiscard]] types::LidarConfigData config() const override;

private:
    [[nodiscard]] PhysicalLength traceBeam(const Orientation& beam) const;

    types::LidarConfigData config_;
    const IMap3D& map_;
    const IGPS& gps_;
};

} // namespace drone_mapper
