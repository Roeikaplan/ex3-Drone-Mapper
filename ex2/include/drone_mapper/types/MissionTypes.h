#pragma once

#include <drone_mapper/Units.h>

#include <cstddef>
#include <string>
#include <vector>

namespace drone_mapper::types {

// Changed: boundaries were removed because map bounds now live on MapConfig/IMap3D.
struct MissionConfigData {
    std::size_t max_steps = 0;
    PhysicalLength gps_resolution{};
    double output_mapping_resolution_factor = 0;
    // 20.6 - readded the map boundaries to the MissionConfig for outputting
    MappingBounds mission_bounds{};
};

enum class MissionRunStatus {
    Completed,
    MaxSteps,
    Error,
};

struct ErrorRef {
    std::string code{};
    std::string message{};
};

struct MissionRunResult {
    MissionRunStatus status = MissionRunStatus::Completed;
    std::size_t steps = 0;
    // Changed: a run can report multiple errors instead of a single ErrorRef.
    std::vector<ErrorRef> errors{}; // we may have multiple errors

    //Removed in 9.6
    // double score = 0.0; // moved to simulationResult
    // std::filesystem::path output_map_file{}; // moved to simulation Result
};

} // namespace drone_mapper::types
