#pragma once

#include <Common/Types.h>

namespace MissionControl {

using namespace Common;

class IDroneControl {
public:
    virtual ~IDroneControl() = default;
    [[nodiscard]] virtual types::DroneStepResult step() = 0;
    [[nodiscard]] virtual types::DroneState state() const = 0;
};

} // namespace MissionControl
