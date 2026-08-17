#pragma once

#include <drone_mapper/IDroneMovement.h>
#include <drone_mapper/MockGPS.h>

namespace drone_mapper {

/**
 * @brief Simulation stand-in for the drone's motion actuator (`IDroneMovement`).
 *
 * Applies motion commands by mutating the shared `MockGPS` pose directly, modelling
 * a perfect actuator that always executes the requested motion.
 *
 * @note Architectural boundary: these methods only update the GPS pose and always
 *       report success. They do **not** enforce per-command drone limits
 *       (`max_advance`/`max_rotate`/`max_elevate`), map boundaries, or collisions —
 *       that validation is `DroneControl`'s responsibility (see `docs/HLD.md`).
 */
class MockMovement final : public IDroneMovement {
public:
    /**
     * @brief Construct bound to the GPS whose pose this actuator drives.
     * @param gps Shared pose the movement commands mutate; must outlive this object.
     */
    explicit MockMovement(MockGPS& gps);

    /**
     * @brief Rotate the heading in place about the vertical axis.
     * @param direction `Left` adds, `Right` subtracts the angle (angle convention:
     *        0=east/+X, 90=south/+Y).
     * @param angle Rotation magnitude in degrees.
     * @return Always success.
     */
    types::MovementResult rotate(types::RotationDirection direction, HorizontalAngle angle) override;
    /**
     * @brief Move horizontally along the current heading (XY plane; altitude unchanged).
     * @param distance Travel distance in cm along the heading direction.
     * @return Always success.
     */
    types::MovementResult advance(PhysicalLength distance) override;
    /**
     * @brief Change altitude only (Z axis).
     * @param distance Vertical travel in cm; **may be negative** to descend.
     * @return Always success.
     */
    types::MovementResult elevate(PhysicalLength distance) override;

private:
    MockGPS& gps_;
};

} // namespace drone_mapper
