/**
 * @file PluginSmokeCheck.h
 * @brief The phase-01 exercise of the complete plugin lifecycle.
 */

#pragma once

#include <Simulator/ErrorLogger.h>

#include <filesystem>

namespace simulator {

/**
 * @brief Load, instantiate, exercise, and tear down every plugin in two folders.
 * @param algorithms_dir Folder (or single `.so`) holding mapping-algorithm plugins.
 * @param mission_controls_dir Folder (or single `.so`) holding mission-control plugins.
 * @param logger Sink for every failure encountered; must outlive this call.
 * @return 0 when every plugin loaded, registered, and ran; 1 when anything failed.
 * @note Architectural boundary: this function owns the teardown ordering that the rest of the
 *       project inherits - destroy instances, destroy the load report, clear the registrar, and
 *       only then release the libraries. Getting that order wrong crashes during static
 *       destruction, after `main` has already returned successfully.
 * @note Failures go to @p logger and progress goes to `std::cout`. Keeping the two apart means the
 *       error log holds only things that went wrong, which is what makes an empty log meaningful.
 * @note Kept separate from `main` so it can be driven from a test without rewriting it.
 */
[[nodiscard]] int runPluginSmokeCheck(const std::filesystem::path& algorithms_dir,
                                      const std::filesystem::path& mission_controls_dir,
                                      ErrorLogger& logger);

} // namespace simulator
