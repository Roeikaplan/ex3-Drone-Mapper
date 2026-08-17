#pragma once

#include <drone_mapper/CompositionPaths.h>
#include <drone_mapper/ErrorLogger.h>
#include <drone_mapper/types/SimulationTypes.h>

#include <filesystem>

namespace drone_mapper {

/**
 * @brief Parse a `simulation.yaml` composition file into a `SimulationCompositionData`.
 *
 * Reads the nested `simulations` (each a `simulation_config` + a list of `mission_configs`),
 * `drone_configs`, and `lidar_configs`, which the manager expands into the cartesian product of runs.
 * File units are centimetres and degrees; values are converted to the strong `mp-units` types.
 *
 * @param file Path to the YAML composition file.
 * @param log Sink for recoverable parse errors (missing/invalid keys), logged immediately.
 * @param out_paths Optional sink for each config's source path (as written in the composition),
 *        kept index-parallel to the returned composition so the score report can label runs by their
 *        source file. Filled only for the file-reference layout; entries are empty for inline configs.
 *        Pass nullptr when the paths are not needed (e.g. component tests).
 * @return The parsed composition, with `composition_file` set to @p file.
 * @throws YAML::Exception if the file cannot be opened or is not valid YAML (the caller in `main`
 *         catches this so the program never crashes). Individual missing/invalid scalars do **not**
 *         throw — they fall back to sensible defaults and are logged.
 * @note yaml-cpp is intentionally kept out of this header so it does not leak to consumers; only the
 *       implementation depends on it.
 */
[[nodiscard]] types::SimulationCompositionData loadComposition(const std::filesystem::path& file,
                                                               ErrorLogger& log,
                                                               CompositionPaths* out_paths = nullptr);

} // namespace drone_mapper