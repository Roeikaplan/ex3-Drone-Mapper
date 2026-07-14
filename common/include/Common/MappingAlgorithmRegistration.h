#pragma once

#include <Common/MappingAlgorithmFactory.h>

#include <utility>

namespace Common {

struct MappingAlgorithmRegistration {
    explicit MappingAlgorithmRegistration(MappingAlgorithmFactory factory);
};

} // namespace Common

#define REGISTER_MAPPING_ALGORITHM(class_name)                                      \
    [[maybe_unused]] ::Common::MappingAlgorithmRegistration register_me_##class_name{ \
        [](::Common::MappingAlgorithmDependencies dependencies)                     \
            -> std::unique_ptr<::Common::IMappingAlgorithm> {                       \
            return std::make_unique<class_name>(std::move(dependencies));            \
        }}
