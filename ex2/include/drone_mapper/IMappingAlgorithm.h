#pragma once

#include <drone_mapper/Types.h>

#include <vector>

namespace drone_mapper {

class IMap3D;

// **Do not change this interface.**
class IMappingAlgorithm {
public:
    virtual ~IMappingAlgorithm() = default;
    IMappingAlgorithm(const types::MissionConfigData& mission_config,
                    const types::LidarConfigData& lidar_config,
                    const types::DroneConfigData& drone_config,
                    const IMap3D& output_map)
        : mission_config_(mission_config),
          lidar_config_(lidar_config),
          drone_config_(drone_config),
          output_map_(output_map){}
    [[nodiscard]] virtual types::MappingStepCommand nextStep(const types::DroneState& state,
                                                             const types::LidarScanResult* latest_scan) = 0; // latest_scan can be a null pointer!
    // Signature changes 
    //virtual void applyVoxelUpdates(const std::vector<types::MappedVoxel> voxels) = 0; REMOVED in 12.6


// Added in 12.6
protected:
    const types::MissionConfigData mission_config_;
    const types::LidarConfigData lidar_config_;
    const types::DroneConfigData drone_config_;
    const IMap3D& output_map_;
};

} // namespace drone_mapper
