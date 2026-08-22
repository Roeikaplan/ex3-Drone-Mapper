/**
 * @file CompositionLoader.h
 * @brief Parsing of the composition YAML into the simulator's configuration types.
 * @note yaml-cpp is deliberately absent from this header so it does not leak to consumers; only the
 *       implementation depends on it.
 */

#pragma once

#include <Simulator/CompositionPaths.h>
#include <Simulator/ErrorLogger.h>
#include <Simulator/SimulationTypes.h>

#include <filesystem>
#include <string>

namespace simulator {

/**
 * @brief The outcome of loading a composition file.
 * @note Carries its failure rather than throwing, matching `CommandLineParseResult` and
 *       `ResultsDirectory`. Nothing in this API can escape as an exception.
 */
struct CompositionLoadResult {
    /**
     * @brief The parsed composition.
     * @note Meaningful only when `ok()`; otherwise default-constructed.
     */
    types::SimulationCompositionData composition{};

    /**
     * @brief Why the file could not be loaded at all.
     * @note Empty on success. This is reserved for failures that leave nothing to run - an
     *       unreadable or malformed document. Every *recoverable* defect is logged instead and does
     *       not appear here.
     */
    std::string error{};

    /**
     * @brief Whether a usable composition was produced.
     * @return True when no fatal error was recorded.
     */
    [[nodiscard]] bool ok() const noexcept { return error.empty(); }
};

/**
 * @brief Parse a composition file into the simulations, missions, drones and lidars it names.
 * @param file Path to the composition YAML.
 * @param logger Sink for recoverable defects; every one is reported through `logInputError`.
 * @param out_paths Optional sink for each config's source path, kept index-parallel to the returned
 *        composition. Pass `nullptr` when the paths are not needed, as component tests do.
 * @return The parsed composition, or the reason the file could not be read.
 *
 * @note Architectural boundary: this is the only place in the program that knows what YAML looks
 *       like. Everything downstream works on typed structs with `mp-units` quantities already
 *       attached, so changing the input format touches exactly one translation unit.
 * @note Recovery is two-tiered and neither tier is fatal. A missing or invalid *field* falls back to
 *       a default; an unreadable or malformed *referenced file* degrades to an all-defaults entry.
 *       Only the composition file itself failing to parse produces an `error`.
 * @note Every relative path - the referenced configs and, transitively, the `map_filename` inside a
 *       referenced simulation config - resolves against the **composition file's** parent directory
 *       rather than the working directory, which is what makes a dataset runnable from anywhere.
 * @note Ex3 ships only the file-reference layout, so every config entry is expected to be a path
 *       string. The inline layout Assignment 2 also accepted has no Ex3 equivalent.
 */
[[nodiscard]] CompositionLoadResult loadComposition(const std::filesystem::path& file,
                                                    ErrorLogger& logger,
                                                    CompositionPaths* out_paths = nullptr);

} // namespace simulator
