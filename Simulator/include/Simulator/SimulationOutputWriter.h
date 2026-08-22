/**
 * @file SimulationOutputWriter.h
 * @brief Serialisation of one plugin's results to the score report YAML.
 * @note yaml-cpp is confined to the implementation, so this header stays dependency-free.
 */

#pragma once

#include <Simulator/CompositionPaths.h>
#include <Simulator/SimulationTypes.h>

#include <filesystem>
#include <string>

namespace simulator {

/**
 * @brief Write one plugin's report to `<plugin_label>__simulation_output.yaml`.
 * @param report The aggregate produced by `SimulationManager::run`.
 * @param output_path Directory to write into; created if missing.
 * @param plugin_label Name of the plugin these results belong to; embedded in the filename.
 * @param paths Source config paths from `loadComposition`, used to label each run.
 *
 * @note One file per plugin, because a results folder holds several plugins' runs side by side and
 *       the assignment asks for the plugin's name to be part of the filename. The `__` separator
 *       matches the convention the output maps already use.
 * @note Architectural boundary: the writer is separate from `SimulationManager` so the manager's
 *       `run()` stays free of file I/O. That is what lets the manager be unit-tested without a
 *       filesystem, and it is what will let phase 08 run managers concurrently.
 * @note Run identity is inferred **positionally**: the writer walks @p paths in the same nested
 *       order the manager expands runs, consuming `report.runs` by index. `SimulationResult` stores
 *       its configs by value, so the report holds copies and `ConfigIdentityIndex` - which resolves
 *       by address - cannot help here. The expansion order is therefore a contract between this file
 *       and `SimulationManager::run`.
 * @note Never throws. A directory that cannot be created or a file that cannot be opened leaves no
 *       report rather than ending the program; the run itself has already succeeded by this point.
 */
void writeSimulationOutput(const types::SimulationManagerReport& report,
                           const std::filesystem::path& output_path,
                           const std::string& plugin_label, const CompositionPaths& paths);

} // namespace simulator
