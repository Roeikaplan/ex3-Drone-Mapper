#pragma once

#include <drone_mapper/IGPS.h>

namespace drone_mapper {

/**
 * @brief Simulation stand-in for the drone's positioning sensor (`IGPS`).
 *
 * Holds the drone's true pose and lets the movement mock update it in place.
 * Unlike real hardware, this mock reports the pose **exactly**: `position()` and
 * `heading()` return the stored values without quantizing to `resolution`.
 *
 * @note Architectural boundary: `MockGPS` is passive state — it never validates
 *       poses or motion. `MockMovement` writes to it via `setPosition`/`setHeading`,
 *       and higher-level logic (`DroneControl`) owns any legality checks.
 */
class MockGPS final : public IGPS {
public:
    /**
     * @brief Construct with the drone's initial pose.
     * @param position Initial world position of the drone centre (cm).
     * @param heading Initial orientation (degrees).
     * @param resolution Configured GPS precision (cm). Retained for interface
     *        compatibility but **deliberately not applied** — the hardware
     *        quantization step is intentionally bypassed in this mock.
     */
    MockGPS(Position3D position, Orientation heading, PhysicalLength resolution);

    /**
     * @brief Reported world position of the drone centre.
     * @return The exact stored position (no resolution quantization).
     */
    [[nodiscard]] Position3D position() const override;
    /**
     * @brief Reported orientation (heading) of the drone.
     * @return The exact stored heading.
     */
    [[nodiscard]] Orientation heading() const override;

    /**
     * @brief Overwrite the stored position (used by the movement mock).
     * @param position New world position of the drone centre (cm).
     */
    void setPosition(Position3D position);
    /**
     * @brief Overwrite the stored heading (used by the movement mock).
     * @param heading New orientation (degrees).
     */
    void setHeading(Orientation heading);

private:
    Position3D position_{};
    Orientation heading_{};
    // Kept to satisfy the constructor/config contract; intentionally unused because
    // this mock reports the exact pose rather than a quantized GPS reading.
    PhysicalLength resolution_{};
};

} // namespace drone_mapper
