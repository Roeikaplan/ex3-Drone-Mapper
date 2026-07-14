#pragma once

#include <Common/MissionControlFactory.h>

#include <utility>

namespace common {

struct MissionControlRegistration {
    explicit MissionControlRegistration(MissionControlFactory factory);
};

} // namespace Common

#define REGISTER_MISSION_CONTROL(class_name)                                      \
    [[maybe_unused]] ::Common::MissionControlRegistration register_me_##class_name{ \
        [](::Common::MissionControlDependencies dependencies)                     \
            -> std::unique_ptr<::Common::IMissionControl> {                       \
            return std::make_unique<class_name>(std::move(dependencies));          \
        }}
