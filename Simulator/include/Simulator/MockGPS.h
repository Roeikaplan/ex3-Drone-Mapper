/**
 * @file MockGPS.h
 * @brief The simulation's stand-in for the drone's positioning sensor.
 */

#pragma once

#include <Common/IGPS.h>

namespace simulator {

/**
 * @brief Holds the drone's true pose and lets the movement mock update it in place.
 *
 * @note Architectural boundary: passive state. It never validates a pose and never moves anything -
 *       `MockMovement` writes to it, and legality checking belongs to whoever drives the drone,
 *       which in Assignment 3 is `DroneControlImpl` inside the MissionControl plugin.
 * @note Unlike real hardware this reports the pose **exactly**: the configured resolution is stored
 *       but deliberately never applied. A quantized reading would leave the algorithm planning
 *       against a pose that disagrees with where the drone actually is, which would undermine the
 *       observed-`Empty`-only traversal that gives the mapping algorithm its collision guarantee.
 * @note A simulation fiction: a real deployment replaces this with a driver behind the same `IGPS`
 *       interface, and nothing above it changes.
 */
class MockGPS final : public common::IGPS {
public:
    /**
     * @brief Construct with the drone's initial pose.
     * @param position Initial world position of the drone centre, in centimetres.
     * @param heading Initial orientation; `0 deg` is +X east and `90 deg` is +Y south.
     * @param resolution Configured GPS precision in centimetres. Retained to honour the
     *        configuration contract but **not applied** - see the class note.
     */
    MockGPS(common::Position3D position, common::Orientation heading,
            common::PhysicalLength resolution);

    /**
     * @brief The drone's reported position.
     * @return The exact stored position, with no quantization.
     */
    [[nodiscard]] common::Position3D position() const override;

    /**
     * @brief The drone's reported orientation.
     * @return The exact stored heading.
     */
    [[nodiscard]] common::Orientation heading() const override;

    /**
     * @brief Overwrite the stored position.
     * @param position New world position of the drone centre.
     * @note For the movement mock's use. Nothing else should move the drone behind its back.
     */
    void setPosition(common::Position3D position);

    /**
     * @brief Overwrite the stored heading.
     * @param heading New orientation.
     * @note For the movement mock's use.
     */
    void setHeading(common::Orientation heading);

private:
    common::Position3D position_{};
    common::Orientation heading_{};

    /**
     * @brief Configured GPS precision.
     * @note Intentionally unused; kept so the constructor reflects the mission configuration and so
     *       a future variant could apply it without changing every call site.
     */
    common::PhysicalLength resolution_{};
};

} // namespace simulator
