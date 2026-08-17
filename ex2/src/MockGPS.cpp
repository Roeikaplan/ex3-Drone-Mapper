#include <drone_mapper/MockGPS.h>

namespace drone_mapper {

MockGPS::MockGPS(Position3D position, Orientation heading, PhysicalLength resolution)
    : position_(position), heading_(heading), resolution_(resolution) {}

Position3D MockGPS::position() const {
    return position_;
}

Orientation MockGPS::heading() const {
    return heading_;
}

void MockGPS::setPosition(Position3D position) {
    position_ = position;
}

void MockGPS::setHeading(Orientation heading) {
    heading_ = heading;
}

} // namespace drone_mapper
