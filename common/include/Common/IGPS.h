#pragma once

#include <Common/Types.h>

namespace Common {

class IGPS {
public:
    virtual ~IGPS() = default;
    [[nodiscard]] virtual Position3D position() const = 0;
    [[nodiscard]] virtual Orientation heading() const = 0;
};

} // namespace Common
