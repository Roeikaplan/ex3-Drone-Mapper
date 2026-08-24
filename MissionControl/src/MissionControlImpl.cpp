/**
 * @file MissionControlImpl.cpp
 * @brief The step loop, the stop conditions, and the verbose trace.
 */

#include <MissionControl/MissionControlImpl.h>

#include <ostream>
#include <utility>

namespace mission_control {
namespace {

/**
 * @brief Short label for the movement a command carried.
 * @param command The command the algorithm produced.
 * @return A single CSV-safe token describing the movement, or "none".
 * @note Written for a human reading a trace, not for machine parsing: the point of the column is to
 *       answer "what did it try to do here" at a glance when a mission stalls.
 */
[[nodiscard]] std::string movementLabel(const types::MappingStepCommand& command) {
    if (!command.movement) {
        return "none";
    }

    const types::MovementCommand& move = *command.movement;
    switch (move.type) {
    case types::MovementCommandType::Hover:
        return "hover";
    case types::MovementCommandType::Rotate:
        return std::string{"rotate_"} +
               (move.rotation == types::RotationDirection::Left ? "left_" : "right_") +
               std::to_string(move.angle.force_numerical_value_in(deg));
    case types::MovementCommandType::Advance:
        return "advance_" + std::to_string(move.distance.force_numerical_value_in(cm));
    case types::MovementCommandType::Elevate:
        return "elevate_" + std::to_string(move.distance.force_numerical_value_in(cm));
    }
    return "unknown";
}

/**
 * @brief Short label for a step's outcome.
 * @param status The status the step returned.
 * @return A lowercase token.
 */
[[nodiscard]] std::string statusLabel(types::DroneStepStatus status) {
    switch (status) {
    case types::DroneStepStatus::Continue:
        return "continue";
    case types::DroneStepStatus::Completed:
        return "completed";
    case types::DroneStepStatus::Error:
        return "error";
    }
    return "unknown";
}

/**
 * @brief Where the verbose trace for a given output map belongs.
 * @param output_map_file The map this mission will produce.
 * @return The same path with `__steps.csv` in place of the extension.
 * @note Named from the output map rather than independently, so a results folder holding dozens of
 *       runs pairs each trace with its map by inspection.
 */
[[nodiscard]] std::filesystem::path traceFileFor(const std::filesystem::path& output_map_file) {
    std::filesystem::path trace = output_map_file;
    trace.replace_extension();
    trace += "__steps.csv";
    return trace;
}

} // namespace

/**
 * @brief Construct from the host-supplied dependencies.
 * @param dependencies Configs, sensors, output map, algorithm, output path, and verbose flag.
 * @note The drone controller is built here, in the initialiser list, from the same references the
 *       host handed over. Nothing is copied except the configs, which the controller keeps by value.
 * @note The trace file is opened eagerly when verbose is set rather than on the first write, so a
 *       mission that fails on step zero still leaves a header row explaining what the columns were.
 */
MissionControlImpl::MissionControlImpl(common::MissionControlDependencies dependencies)
    : mission_(dependencies.mission_config),
      output_map_(dependencies.output_map),
      output_map_file_(std::move(dependencies.output_map_file)),
      verbose_(dependencies.verbose),
      drone_control_(dependencies.drone_config, dependencies.mission_config, dependencies.lidar,
                     dependencies.gps, dependencies.movement, dependencies.output_map,
                     dependencies.mapping_algorithm) {
    if (verbose_ && !output_map_file_.empty()) {
        std::ofstream stream(traceFileFor(output_map_file_), std::ios::trunc);
        if (stream) {
            stream << "step,x_cm,y_cm,z_cm,heading_deg,command,status\n";
            trace_ = std::move(stream);
        }
    }
}

/**
 * @brief Run the mission to completion.
 * @return The outcome, the number of steps taken, and any error that ended it.
 * @note `MaxSteps` is the default outcome, overwritten the instant a step reports otherwise. Framing
 *       it that way means the budget-exhausted case needs no separate detection - falling out of the
 *       loop *is* the condition.
 * @note A drone-step error ends the mission rather than being retried. A refused move means the
 *       algorithm asked for something illegal from the pose it is in, and continuing would just
 *       replan from a state already known to be bad; the simulator scores the run -1 and moves on.
 */
types::MissionRunResult MissionControlImpl::runMission() {
    types::MissionRunResult result{};
    result.status = types::MissionRunStatus::MaxSteps;

    for (std::size_t step = 0; step < mission_.max_steps; ++step) {
        const types::DroneState before = drone_control_.state();
        const types::DroneStepResult step_result = drone_control_.step();
        result.steps = step + 1;

        traceStep(step, before, step_result);

        if (step_result.status == types::DroneStepStatus::Completed) {
            result.status = types::MissionRunStatus::Completed;
            break;
        }
        if (step_result.status == types::DroneStepStatus::Error) {
            result.status = types::MissionRunStatus::Error;
            result.errors.push_back(types::ErrorRef{"DRONE_STEP_ERROR", step_result.message});
            break;
        }
    }

    /**
     * @note Saved once the loop settles, on every outcome including failure. A partial map from a
     *       failed mission is still worth having on disk to look at, even though the simulator will
     *       score that run -1 rather than compare it.
     */
    if (!output_map_file_.empty()) {
        output_map_.save(output_map_file_);
    }

    return result;
}

/**
 * @brief Append one row to the verbose trace.
 * @param step Zero-based step index.
 * @param state The drone's pose before the step ran.
 * @param result What the step returned.
 * @note The pose recorded is the one the step *started* from, paired with the command issued from
 *       it. Reading the file top to bottom then shows the drone's path as a sequence of positions
 *       and the decision made at each.
 * @note Each row is flushed rather than buffered. A trace exists to explain a mission that went
 *       wrong, so it has to survive whatever went wrong - a buffered final few hundred rows are
 *       lost precisely when they matter. The cost is one flush per step, and only when verbose
 *       output was asked for.
 */
void MissionControlImpl::traceStep(std::size_t step, const types::DroneState& state,
                                   const types::DroneStepResult& result) {
    if (!trace_) {
        return;
    }

    *trace_ << step << ',' << state.position.x.force_numerical_value_in(cm) << ','
            << state.position.y.force_numerical_value_in(cm) << ','
            << state.position.z.force_numerical_value_in(cm) << ','
            << state.heading.horizontal.force_numerical_value_in(deg) << ','
            << movementLabel(drone_control_.lastCommand()) << ',' << statusLabel(result.status)
            << std::endl;
}

} // namespace mission_control
