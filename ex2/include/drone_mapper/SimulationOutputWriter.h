#pragma once

#include <drone_mapper/CompositionPaths.h>
#include <drone_mapper/types/SimulationTypes.h>

#include <filesystem>

namespace drone_mapper {

/**
 * @brief Serialize a `SimulationManagerReport` to `<output_path>/simulation_output.yaml`.
 *
 * Emits the mandated hierarchical report nested under a top-level `score_report:` key: metadata
 * (`composition_file`, `generated_at_utc`, `metric`, `score_range`, `error_score`), a derived
 * `summary` (`total_runs`/`scored_runs`/`error_runs`/`average`/`min`/`max`), and the flat run list
 * re-grouped into nested `simulations → missions → runs`. `resolution_cm` and
 * `resolution_request_status` sit at the mission level; each run is labelled by its output-map file.
 *
 * @param report The aggregate produced by `SimulationManager::run()`.
 * @param output_path Directory to write into (created if missing); the file is overwritten.
 * @param composition_file Source composition path, echoed into the report.
 * @param paths Source config-file paths from `loadComposition`. When they fully describe the runs
 *        (the mandated file-reference layout), the report labels each simulation/mission/run by its
 *        `simulation_config`/`mission_config`/`drone_config`/`lidar_config` path (the PDF sample);
 *        otherwise (inline layout, or an empty `CompositionPaths{}`) it falls back to value labels.
 * @note yaml-cpp is confined to the implementation, so this header stays dependency-free. Keeping the
 *       writer separate from `SimulationManager` lets the manager's `run()` stay pure and file-free.
 */
void writeSimulationOutput(const types::SimulationManagerReport& report,
                           const std::filesystem::path& output_path,
                           const std::filesystem::path& composition_file,
                           const CompositionPaths& paths);

} // namespace drone_mapper