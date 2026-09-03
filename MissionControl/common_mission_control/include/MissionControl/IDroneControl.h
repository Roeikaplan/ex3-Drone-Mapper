#pragma once

#include <Common/Types.h>

namespace mission_control_323998450_211633813 {

using namespace common;

class IDroneControl {
public:
    virtual ~IDroneControl() = default;
    [[nodiscard]] virtual types::DroneStepResult step() = 0;
    [[nodiscard]] virtual types::DroneState state() const = 0;
};

} // namespace mission_control_323998450_211633813
