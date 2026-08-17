#include <drone_mapper/MissionControlImpl.h>

#include <utility>

namespace drone_mapper {

MissionControlImpl::MissionControlImpl(types::MissionConfigData mission,
                                       types::DroneConfigData drone,
                                       const IMap3D& hidden_map,
                                       IMutableMap3D& output_map,
                                       IDroneControl& drone_control,
                                       std::filesystem::path output_map_file)
    : mission_(std::move(mission)),
      drone_(std::move(drone)),
      hidden_map_(hidden_map),
      output_map_(output_map),
      drone_control_(drone_control),
      output_map_file_(std::move(output_map_file)) {}

types::MissionRunResult MissionControlImpl::runMission() {
    types::MissionRunResult result{};
    // Budget exhausted without the algorithm reporting completion is the default outcome; the loop
    // overwrites it the instant a step reports Completed or Error.
    result.status = types::MissionRunStatus::MaxSteps;

    for (std::size_t step = 0; step < mission_.max_steps; ++step) {
        const types::DroneStepResult step_result = drone_control_.step();
        result.steps = step + 1;

        if (step_result.status == types::DroneStepStatus::Completed) {
            result.status = types::MissionRunStatus::Completed;
            break;
        }
        if (step_result.status == types::DroneStepStatus::Error) {
            // A drone-step Error is a fatal movement-validation/execution failure (a move that would
            // collide or leave the mission bounds). A collision ends the simulation with a failure
            // notice, so stop and surface it — the run is scored -1 upstream and the manager
            // proceeds to the next run, rather than replanning from a bad pose.
            result.status = types::MissionRunStatus::Error;
            result.errors.push_back(types::ErrorRef{"DRONE_STEP_ERROR", step_result.message});
            break;
        }
        // Continue: the algorithm is still working; keep stepping.
    }

    // Persist the accumulated output map once the loop settles: only the final map (every scan
    // applied) is meaningful for scoring, so it is saved here rather than per step.
    output_map_.save(output_map_file_);
    return result;
}

} // namespace drone_mapper
