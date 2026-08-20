/**
 * @file PluginSmokeCheck.cpp
 * @brief Exercises the whole plugin lifecycle against do-nothing fixtures.
 * @note This is the phase-01 deliverable: no drones, no maps, no YAML. Its value is that every step
 *       it performs fails at runtime rather than at compile time, so proving them here means later
 *       failures are unambiguously in simulation logic rather than in the loader.
 */

#include <Simulator/PluginSmokeCheck.h>

#include <Simulator/PluginLoader.h>
#include <Simulator/Registrar.h>

#include <Common/IMappingAlgorithm.h>
#include <Common/IMissionControl.h>
#include <Common/MappingAlgorithmFactory.h>
#include <Common/MissionControlFactory.h>

#include "NullSensors.h"

#include <exception>
#include <iostream>
#include <memory>
#include <vector>

namespace simulator {
namespace {

/**
 * @brief Name of an algorithm status, for the driver's output.
 * @param status Status reported by a plugin's `nextStep`.
 * @return A short human-readable label.
 */
[[nodiscard]] const char* algorithmStatusName(common::types::AlgorithmStatus status) {
    switch (status) {
    case common::types::AlgorithmStatus::Working:
        return "Working";
    case common::types::AlgorithmStatus::Finished:
        return "Finished";
    case common::types::AlgorithmStatus::FinishedWithUnmappableVoxels:
        return "FinishedWithUnmappableVoxels";
    }
    return "unknown";
}

/**
 * @brief Name of a mission status, for the driver's output.
 * @param status Status reported by a plugin's `runMission`.
 * @return A short human-readable label.
 */
[[nodiscard]] const char* missionStatusName(common::types::MissionRunStatus status) {
    switch (status) {
    case common::types::MissionRunStatus::Completed:
        return "Completed";
    case common::types::MissionRunStatus::MaxSteps:
        return "MaxSteps";
    case common::types::MissionRunStatus::Error:
        return "Error";
    }
    return "unknown";
}

/**
 * @brief Everything the plugins need in order to be constructed at all.
 *
 * @note Architectural boundary: `IMappingAlgorithm` copies the three configs into value members but
 *       holds `const IMap3D&` by reference, so the map must outlive every instance built from it.
 *       Bundling them here guarantees that, because this object is declared before the instances.
 */
struct SmokeCheckContext {
    common::types::MissionConfigData mission_config{};
    common::types::LidarConfigData lidar_config{};
    common::types::DroneConfigData drone_config{};
    NullMap3D map{};
    NullGPS gps{};
    NullMovement movement{};
    NullLidar lidar{};
};

/**
 * @brief Build and exercise one instance of every loaded plugin.
 * @param report The plugins claimed by the loader.
 * @param context Dependencies that must outlive every instance created here.
 * @return True when every factory produced a usable instance.
 * @note Instances live only for this function. Returning before the caller clears the registrar and
 *       releases the libraries is exactly the ordering the whole design depends on.
 * @note Each mission control is handed the *first* loaded algorithm, so `runMission` crossing into
 *       `nextStep` proves the full host to MissionControl `.so` to Algorithm `.so` call chain.
 */
[[nodiscard]] bool exercisePlugins(const PluginLoadReport& report, SmokeCheckContext& context) {
    bool all_ok = true;

    std::vector<std::unique_ptr<common::IMappingAlgorithm>> algorithms;
    for (const LoadedAlgorithm& loaded : report.algorithms) {
        std::unique_ptr<common::IMappingAlgorithm> instance =
            loaded.factory(common::MappingAlgorithmDependencies{
                context.mission_config, context.lidar_config, context.drone_config, context.map});
        if (!instance) {
            std::cout << "  " << loaded.file.filename().string() << " -> factory returned null\n";
            all_ok = false;
            continue;
        }
        const common::types::MappingStepCommand command =
            instance->nextStep(common::types::DroneState{}, nullptr);
        std::cout << "  " << loaded.file.filename().string()
                  << " -> nextStep status=" << algorithmStatusName(command.status) << "\n";
        algorithms.push_back(std::move(instance));
    }

    if (algorithms.empty() && !report.mission_controls.empty()) {
        std::cout << "  no algorithm instance available; mission controls cannot be exercised\n";
        return false;
    }

    for (const LoadedMissionControl& loaded : report.mission_controls) {
        std::unique_ptr<common::IMissionControl> instance =
            loaded.factory(common::MissionControlDependencies{context.mission_config,
                                                              context.drone_config,
                                                              context.lidar,
                                                              context.gps,
                                                              context.movement,
                                                              context.map,
                                                              *algorithms.front(),
                                                              std::filesystem::path{},
                                                              false});
        if (!instance) {
            std::cout << "  " << loaded.file.filename().string() << " -> factory returned null\n";
            all_ok = false;
            continue;
        }
        const common::types::MissionRunResult result = instance->runMission();
        std::cout << "  " << loaded.file.filename().string()
                  << " -> runMission status=" << missionStatusName(result.status)
                  << " steps=" << result.steps << "\n";
    }

    return all_ok;
}

} // namespace

int runPluginSmokeCheck(const std::filesystem::path& algorithms_dir,
                        const std::filesystem::path& mission_controls_dir) {
    PluginLoader loader;
    bool ok = true;

    {
        /**
         * @note The report owns the claimed factories, so it is scoped to die before the libraries
         *       are released. Its destructor is step two of the four-step teardown.
         */
        PluginLoadReport report;
        loader.load(algorithms_dir, PluginLoader::Kind::Algorithm, report);
        loader.load(mission_controls_dir, PluginLoader::Kind::MissionControl, report);

        std::cout << "algorithms=" << report.algorithms.size()
                  << " mission_controls=" << report.mission_controls.size()
                  << " failures=" << report.failures.size() << "\n";
        for (const PluginFailure& failure : report.failures) {
            std::cout << "  FAILED " << failure.file.string() << ": " << failure.reason << "\n";
            ok = false;
        }
        if (report.algorithms.empty() || report.mission_controls.empty()) {
            std::cout << "  expected at least one plugin of each kind\n";
            ok = false;
        }

        SmokeCheckContext context;
        try {
            ok = exercisePlugins(report, context) && ok;
        } catch (const std::exception& error) {
            /**
             * @note A plugin's own code runs inside `exercisePlugins`. An exception escaping it
             *       must not skip the teardown below, so it is caught here rather than at `main`.
             */
            std::cout << "  plugin threw: " << error.what() << "\n";
            ok = false;
        } catch (...) {
            std::cout << "  plugin threw a non-standard exception\n";
            ok = false;
        }
    }

    /**
     * @note Step three. Anything a library registered but the loader never claimed is dropped here,
     *       while the libraries are still mapped.
     */
    Registrar::instance().clear();

    /**
     * @note Step four, and only now. Every `std::function` and every plugin-derived object is gone,
     *       so unmapping the code they pointed into is finally safe.
     */
    loader.releaseAll();

    std::cout << (ok ? "plugin smoke check OK\n" : "plugin smoke check FAILED\n");
    return ok ? 0 : 1;
}

} // namespace simulator
