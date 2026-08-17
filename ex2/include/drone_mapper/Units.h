#pragma once

#include <mp-units/framework.h>
#include <mp-units/systems/isq.h>
#include <mp-units/systems/si/unit_symbols.h>

namespace drone_mapper {

namespace mp = mp_units;
namespace isq = mp_units::isq;
namespace si = mp_units::si;
using mp_units::si::unit_symbols::deg;
using si::unit_symbols::cm;

// required to uniquely define length width and height (see docs of mp-units - isq::length)
// Separate quantity specs keep X, Y, and Z coordinates from being mixed by accident.
QUANTITY_SPEC(x_extent, isq::length);
QUANTITY_SPEC(y_extent, isq::length);
QUANTITY_SPEC(z_extent, isq::length);

using PhysicalLength = mp::quantity<isq::length[cm], double>;
using XLength = mp::quantity<x_extent[cm], double>;
using YLength = mp::quantity<y_extent[cm], double>;
using ZLength = mp::quantity<z_extent[cm], double>;

struct Position3D {
    XLength x{};
    YLength y{};
    ZLength z{};
};

//Angles
// Separate quantity specs keep horizontal and altitude angles explicit.
QUANTITY_SPEC(horizontal_angle, isq::angular_measure);
QUANTITY_SPEC(altitude_angle,   isq::angular_measure);

using HorizontalAngle = mp::quantity<horizontal_angle[deg], double>;
using AltitudeAngle = mp::quantity<altitude_angle[deg], double>;

struct Orientation {
    HorizontalAngle horizontal{};
    AltitudeAngle altitude{};
};

} // namespace drone_mapper
