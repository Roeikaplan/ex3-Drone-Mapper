/**
 * @file Placeholder.cpp
 * @brief Keeps the MissionControl plugin target buildable until it has a real implementation.
 * @note Replaced in phase 05 by `MissionControlImpl`, its own `DroneControlImpl`, and the
 *       registration entry point. Until then the do-nothing mission control used to drive the
 *       simulator lives in `Simulator/tests/fixtures/StubMissionControl.cpp`.
 */

namespace mission_control {

/**
 * @brief Gives the translation unit external-linkage content.
 * @note See the equivalent note in `Algorithm/src/Placeholder.cpp`.
 */
void placeholder() {}

} // namespace mission_control
