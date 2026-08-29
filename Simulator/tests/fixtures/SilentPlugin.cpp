/**
 * @file SilentPlugin.cpp
 * @brief A shared library that loads cleanly and registers nothing, kept as a negative test double.
 *
 * The loader has to tell three outcomes apart: a plugin that registers usably, a file that cannot be
 * `dlopen`ed at all, and a library that opens perfectly but publishes no factory. This fixture is the
 * third case, which is the easiest one to get wrong - nothing fails, so a loader that only checked
 * `dlopen`'s return value would report success and hand back an empty factory.
 *
 * @note Deliberately contains **no** `REGISTER_*` macro. That absence is the entire point of the file,
 *       so do not "fix" it by adding one.
 * @note It still links `common::common`, so it is a genuine, well-formed library - the test would
 *       prove nothing if this were merely a corrupt file.
 */

#include <Common/IMappingAlgorithm.h>

namespace fixtures {

/**
 * @brief Present only so the translation unit has real content and a real dependency on `common/`.
 * @note Never instantiated: nothing registers it, and nothing can reach it from outside the `.so`.
 */
class UnregisteredAlgorithm final : public common::IMappingAlgorithm {
public:
    using common::IMappingAlgorithm::IMappingAlgorithm;

    /**
     * @brief Decide the next command.
     * @return A hover with status `Finished`.
     */
    [[nodiscard]] common::types::MappingStepCommand nextStep(
        const common::types::DroneState&, const common::types::LidarScanResult*) override {
        common::types::MappingStepCommand command{};
        command.status = common::types::AlgorithmStatus::Finished;
        return command;
    }
};

} // namespace fixtures
