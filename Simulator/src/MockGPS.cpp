/**
 * @file MockGPS.cpp
 * @brief Storage and retrieval of the drone's true pose.
 */

#include <Simulator/MockGPS.h>

namespace simulator {

/**
 * @brief Construct with the drone's initial pose.
 * @param position Initial world position of the drone centre.
 * @param heading Initial orientation.
 * @param resolution Configured GPS precision; stored but not applied.
 */
MockGPS::MockGPS(common::Position3D position, common::Orientation heading,
                 common::PhysicalLength resolution)
    : position_(position), heading_(heading), resolution_(resolution) {
    /**
     * @note `resolution_` is assigned and never read. That is the documented decision rather than an
     *       oversight: quantizing the reported pose would make it disagree with the drone's real
     *       position, and the mapping algorithm's safety argument depends on trusting it.
     */
    (void)resolution_;
}

/**
 * @brief The drone's reported position.
 * @return The exact stored position.
 */
common::Position3D MockGPS::position() const {
    return position_;
}

/**
 * @brief The drone's reported orientation.
 * @return The exact stored heading.
 */
common::Orientation MockGPS::heading() const {
    return heading_;
}

/**
 * @brief Overwrite the stored position.
 * @param position New world position of the drone centre.
 */
void MockGPS::setPosition(common::Position3D position) {
    position_ = position;
}

/**
 * @brief Overwrite the stored heading.
 * @param heading New orientation.
 */
void MockGPS::setHeading(common::Orientation heading) {
    heading_ = heading;
}

} // namespace simulator
