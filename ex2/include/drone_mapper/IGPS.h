#pragma once

#include <drone_mapper/Types.h>

namespace drone_mapper {

/// Read-only positioning sensor for the drone.
///
/// Provides the drone's current world pose. Implementations model a real GPS
/// and may report position at a limited precision (the GPS resolution) rather
/// than the exact underlying value.
///
/// **Do not change this interface.**
class IGPS {
public:
    virtual ~IGPS() = default;

    /// Reported world position of the drone. Implementations may quantize this
    /// to the configured GPS resolution, so it is not guaranteed to equal the
    /// exact underlying position.
    [[nodiscard]] virtual Position3D position() const = 0;

    /// Reported orientation (heading) of the drone.
    [[nodiscard]] virtual Orientation heading() const = 0;
};

} // namespace drone_mapper
