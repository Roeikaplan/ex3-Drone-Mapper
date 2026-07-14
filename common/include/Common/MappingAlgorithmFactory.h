#pragma once

#include <Common/IMappingAlgorithm.h>

#include <functional>
#include <memory>

namespace Common {

using MappingAlgorithmFactory =
    std::function<std::unique_ptr<IMappingAlgorithm>(MappingAlgorithmDependencies)>;

} // namespace Common
