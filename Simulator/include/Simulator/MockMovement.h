/**
 * @file MockMovement.h
 * @brief The simulation's stand-in for the drone's actuators.
 */

#pragma once

#include <Common/IDroneMovement.h>
#include <Simulator/MockGPS.h>

namespace simulator {

/**
 * @brief Applies movement commands by writing the resulting pose straight into the GPS.
 *
 * @note Architectural boundary: this **validates nothing** and always reports success - it will fly
 *       the drone through a wall or out of bounds without complaint. Every legality check belongs to
 *       whoever drives the drone, which in Assignment 3 is `DroneControlImpl` inside the
 *       MissionControl plugin. Adding checks here would move policy into the simulator and silently
 *       change what a third-party plugin is allowed to do.
 * @note Takes `MockGPS&` rather than `IGPS&` because it needs the setters. Widening it to the
 *       interface would leave it unable to move anything.
 */
class MockMovement final : public common::IDroneMovement {
public:
    /**
     * @brief Construct over the pose this actuator will update.
     * @param gps The run's GPS; must outlive this object.
     */
    explicit MockMovement(MockGPS& gps);

    /**
     * @brief Turn the drone in place.
     * @param direction Left or right.
     * @param angle Magnitude of the turn.
     * @return Always success.
     */
    common::types::MovementResult rotate(common::types::RotationDirection direction,
                                         common::HorizontalAngle angle) override;

    /**
     * @brief Move the drone horizontally along its current heading.
     * @param distance Distance to travel; may be negative to reverse.
     * @return Always success.
     */
    common::types::MovementResult advance(common::PhysicalLength distance) override;

    /**
     * @brief Change the drone's altitude.
     * @param distance Distance to climb; may be negative to descend.
     * @return Always success.
     */
    common::types::MovementResult elevate(common::PhysicalLength distance) override;

private:
    MockGPS& gps_;
};

} // namespace simulator
