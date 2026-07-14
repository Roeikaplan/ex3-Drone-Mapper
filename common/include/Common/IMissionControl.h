#pragma once

#include <Common/Types.h>

namespace Common {

class IMissionControl {
public:
    virtual ~IMissionControl() = default;
    [[nodiscard]] virtual types::MissionRunResult runMission() = 0;
};

} // namespace Common
