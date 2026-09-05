/**
 * @file PluginLifecycleTest.cpp
 * @brief The plugin lifecycle end to end, and the teardown order that must follow it.
 * @note These are the only tests that actually `dlopen` a real `.so`. They exist because the failure
 *       they guard against is invisible everywhere else: getting the teardown order wrong does not
 *       fail an assertion, it segfaults in `__run_exit_handlers` *after* `main` has returned 0, with a
 *       stack naming nothing in this project. That exact crash was observed during the phase-01 spike.
 * @note This binary links `RegistrationEntryPoints.cpp` and is built with `ENABLE_EXPORTS`, without
 *       which every `dlopen` here fails with "undefined symbol" - the same trap the simulator target
 *       carries a comment about.
 * @note Since the lazy lifecycle landed, these cases also pin *when* the loads happen: discovery must
 *       map nothing, the first acquire must map exactly one library, and the last release must unmap
 *       it. `PluginRegistryTest` covers the counting rules; this file covers the `dlopen` boundary.
 */

#include <Simulator/PluginRegistry.h>
#include <Simulator/Registrar.h>

#include <gtest/gtest.h>

#include <filesystem>
#include <string>

namespace {

namespace fs = std::filesystem;

/**
 * @brief Folder holding the fixture plugins, injected by CMake.
 * @param kind Either "algorithms", "mission_controls", or "negative".
 * @return Path to that fixture folder.
 */
[[nodiscard]] fs::path fixtureFolder(const std::string& kind) {
    return fs::path{DRONE_PLUGIN_FIXTURES} / kind;
}

/**
 * @brief Gives each test a registrar and loader counters with nothing left over from the last one.
 * @note Both are process-wide singletons, so a test that left factories behind - or left the open
 *       count where it was - would hand them to the next test and make the suite order-dependent.
 */
class PluginLifecycleTest : public ::testing::Test {
protected:
    void SetUp() override {
        simulator::Registrar::instance().clear();
        simulator::resetPluginLibraryStats();
    }
    void TearDown() override { simulator::Registrar::instance().clear(); }

    simulator::ErrorLogger logger_{};
    simulator::PluginLifecycleLog lifecycle_{};
};

/**
 * @brief A fixture `.so` loads, runs its registration constructor, and yields a usable factory.
 */
TEST_F(PluginLifecycleTest, AFixturePluginLoadsAndSelfRegisters) {
    /**
     * @note The whole mechanism in one assertion: the `.so` carries an undefined symbol for its
     *       registration constructor, `dlopen` resolves it against this executable, the global
     *       object's constructor runs, and the factory lands in the registrar - which the registry
     *       then claims. If `ENABLE_EXPORTS` were ever dropped, this is what would fail.
     */
    simulator::PluginRegistry registry{logger_, lifecycle_};
    const simulator::PluginRegistry::Discovery discovery = registry.discover(
        fixtureFolder("algorithms") / "StubAlgorithm_A.so", simulator::PluginKind::Algorithm);

    ASSERT_TRUE(discovery.failures.empty());
    ASSERT_EQ(discovery.slots.size(), 1u);

    /**
     * @note Discovery on its own maps nothing. This is the property the whole lazy scheme rests on:
     *       a folder can be enumerated, and its entire task table built, with no library resident.
     */
    EXPECT_EQ(simulator::pluginLibraryStats().opens, 0u);

    simulator::PluginSlot& slot = *discovery.slots.front();
    registry.reserve(slot, 1);

    const common::MappingAlgorithmFactory* factory = registry.acquireAlgorithm(slot);
    ASSERT_NE(factory, nullptr) << "load failed: " << slot.failureReason();
    EXPECT_TRUE(static_cast<bool>(*factory));
    EXPECT_EQ(simulator::pluginLibraryStats().opens, 1u);

    registry.release(slot);
}

/**
 * @brief After a load, the registrar is empty because the registry has claimed everything.
 */
TEST_F(PluginLifecycleTest, TheRegistryClaimsWhatWasRegisteredSoNothingIsLeftBehind) {
    simulator::PluginRegistry registry{logger_, lifecycle_};
    const simulator::PluginRegistry::Discovery discovery =
        registry.discover(fixtureFolder("mission_controls"), simulator::PluginKind::MissionControl);

    ASSERT_GE(discovery.slots.size(), 1u);
    for (simulator::PluginSlot* slot : discovery.slots) {
        registry.reserve(*slot, 1);
        EXPECT_NE(registry.acquireMissionControl(*slot), nullptr) << slot->failureReason();
    }

    /**
     * @note Load-then-claim is the contract: whatever a library registered belongs to its slot, not
     *       to the registrar. Anything still sitting in the registrar afterwards is a factory nobody
     *       owns, and `Registrar::clear()` at teardown exists precisely to catch that case.
     */
    EXPECT_EQ(simulator::Registrar::instance().algorithmCount(), 0u);
    EXPECT_EQ(simulator::Registrar::instance().missionControlCount(), 0u);

    for (simulator::PluginSlot* slot : discovery.slots) {
        registry.release(*slot);
    }
}

/**
 * @brief The full load-claim-copy-destroy-unload sequence completes without crashing.
 */
TEST_F(PluginLifecycleTest, TheTeardownOrderSurvivesAFullLoadClaimDestroySequence) {
    /**
     * @note The regression test for the phase-01 crash, in the order the program performs it:
     *       1. workers joined     - nothing concurrent here, so this step is vacuous
     *       2. instances destroyed
     *       3. factories destroyed (every copy anyone took)
     *       4. the library unmapped
     *       Reversing 3 and 4 unmaps code that live `std::function` targets still point into. Under
     *       the lazy lifecycle step 4 happens inside `release`, on whichever thread finished last,
     *       which makes the ordering *more* load-bearing than it was, not less. The assertion is
     *       simply that the process reaches the end: this test crashing *is* the failure.
     */
    simulator::PluginRegistry registry{logger_, lifecycle_};

    simulator::PluginSlot& algorithm =
        *registry
             .discover(fixtureFolder("algorithms") / "StubAlgorithm_A.so",
                       simulator::PluginKind::Algorithm)
             .slots.front();
    simulator::PluginSlot& mission_control =
        *registry
             .discover(fixtureFolder("mission_controls") / "StubMissionControl_A.so",
                       simulator::PluginKind::MissionControl)
             .slots.front();

    registry.reserve(algorithm, 1);
    registry.reserve(mission_control, 1);

    {
        /**
         * @note Factory copies taken deliberately, because that is the dangerous case. Copying a
         *       plugin's `std::function` copies a callable whose target lives in the plugin's code
         *       segment, so these locals are exactly as unsafe to outlive the unload as an instance -
         *       and they are destroyed at the closing brace, before either release below.
         */
        const common::MappingAlgorithmFactory* algorithm_factory =
            registry.acquireAlgorithm(algorithm);
        const common::MissionControlFactory* mission_control_factory =
            registry.acquireMissionControl(mission_control);
        ASSERT_NE(algorithm_factory, nullptr);
        ASSERT_NE(mission_control_factory, nullptr);

        common::MappingAlgorithmFactory algorithm_copy = *algorithm_factory;
        common::MissionControlFactory mission_control_copy = *mission_control_factory;
        EXPECT_TRUE(static_cast<bool>(algorithm_copy));
        EXPECT_TRUE(static_cast<bool>(mission_control_copy));
        EXPECT_EQ(simulator::pluginLibraryStats().currently_open, 2u);
    }

    registry.release(algorithm);
    registry.release(mission_control);
    simulator::Registrar::instance().clear();

    EXPECT_EQ(simulator::pluginLibraryStats().currently_open, 0u);
    SUCCEED() << "load, claim, copy, destroy, unload completed without crashing";
}

/**
 * @brief `releaseAll` is safe to call with nothing loaded, and safe to call twice.
 * @note It sits on every exit path including the error ones, so idempotence is what lets `main` call
 *       it unconditionally rather than tracking whether a load ever succeeded.
 */
TEST_F(PluginLifecycleTest, ReleasingWithoutLoadingAnythingIsHarmless) {
    simulator::PluginRegistry registry{logger_, lifecycle_};
    registry.releaseAll();
    registry.releaseAll();
    EXPECT_EQ(simulator::pluginLibraryStats().opens, 0u);
    SUCCEED() << "releaseAll is idempotent, which is what makes it safe on every exit path";
}

/**
 * @brief A library that opens cleanly but registers nothing is reported as a load failure.
 */
TEST_F(PluginLifecycleTest, ALibraryThatRegistersNothingIsAFailureNotASuccess) {
    /**
     * @note The outcome that looks like success from `dlopen`'s point of view: the library opens
     *       cleanly and publishes no factory. A registry that only checked the handle would report
     *       this as loaded and hand back an empty `std::function`, which would then fail per run
     *       instead of once, at the moment of the load.
     */
    simulator::PluginRegistry registry{logger_, lifecycle_};
    simulator::PluginSlot& slot =
        *registry
             .discover(fixtureFolder("negative") / "SilentPlugin.so",
                       simulator::PluginKind::Algorithm)
             .slots.front();
    registry.reserve(slot, 1);

    EXPECT_EQ(registry.acquireAlgorithm(slot), nullptr);
    EXPECT_TRUE(slot.failed());
    EXPECT_NE(slot.failureReason().find("registered nothing"), std::string::npos)
        << "actual: " << slot.failureReason();

    /**
     * @note And it is unmapped again immediately. A library that turned out to be useless has no
     *       claim on memory for the rest of the batch - the eager loader kept such handles to the
     *       end, and that is exactly the behaviour this design replaces.
     */
    EXPECT_EQ(simulator::pluginLibraryStats().currently_open, 0u);
    EXPECT_EQ(simulator::pluginLibraryStats().closes, 1u);

    registry.release(slot);
}

/**
 * @brief A path that does not exist is reported as a failure and leaves the batch running.
 * @note One unreadable file in a folder of many must not end the mode - the other plugins still have
 *       runs to complete, and the report names this one under `errors:`.
 */
TEST_F(PluginLifecycleTest, AMissingFileIsReportedRatherThanFatal) {
    simulator::PluginRegistry registry{logger_, lifecycle_};
    const simulator::PluginRegistry::Discovery discovery = registry.discover(
        fixtureFolder("algorithms") / "DoesNotExist.so", simulator::PluginKind::Algorithm);

    /**
     * @note A missing path fails at *discovery*, before any slot exists, which is why this one is
     *       still detected up front rather than at first use.
     */
    EXPECT_TRUE(discovery.slots.empty());
    EXPECT_FALSE(discovery.failures.empty());
    EXPECT_EQ(simulator::pluginLibraryStats().opens, 0u);
}

/**
 * @brief A `.so` registering the wrong kind fails for that role and is unmapped again.
 */
TEST_F(PluginLifecycleTest, AlgorithmLoadedAsMissionControlIsRejected) {
    simulator::PluginRegistry registry{logger_, lifecycle_};
    simulator::PluginSlot& slot =
        *registry
             .discover(fixtureFolder("algorithms") / "StubAlgorithm_A.so",
                       simulator::PluginKind::MissionControl)
             .slots.front();
    registry.reserve(slot, 1);

    EXPECT_EQ(registry.acquireMissionControl(slot), nullptr);
    EXPECT_TRUE(slot.failed());
    EXPECT_NE(slot.failureReason().find("where a mission control was expected"), std::string::npos)
        << "actual: " << slot.failureReason();
    EXPECT_EQ(simulator::pluginLibraryStats().currently_open, 0u);

    registry.release(slot);
}

} // namespace
