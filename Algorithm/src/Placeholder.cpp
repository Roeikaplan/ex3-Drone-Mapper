/**
 * @file Placeholder.cpp
 * @brief Keeps the Algorithm plugin target buildable until it has a real implementation.
 * @note Replaced in phase 06 by `MappingAlgorithmImpl` and its registration entry point. Until then
 *       the do-nothing algorithm used to drive the simulator lives in
 *       `Simulator/tests/fixtures/StubAlgorithm.cpp`.
 */

namespace algorithm {

/**
 * @brief Gives the translation unit external-linkage content.
 * @note An empty translation unit would build, but an unused internal-linkage symbol would trip
 *       `-Wunused` under `-Werror`; an exported function avoids the question entirely.
 */
void placeholder() {}

} // namespace algorithm
