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
 */

#include <Simulator/PluginLoader.h>
#include <Simulator/Registrar.h>
#include <Simulator/SimulationRunFactoryImpl.h>

#include <gtest/gtest.h>

#include <filesystem>
#include <string>

namespace {

namespace fs = std::filesystem;

/**
 * @brief Folder holding the fixture plugins, injected by CMake.
 * @param kind Either "algorithms" or "mission_controls".
 * @return Path to that fixture folder.
 */
[[nodiscard]] fs::path fixtureFolder(const std::string& kind) {
    return fs::path{DRONE_PLUGIN_FIXTURES} / kind;
}

/**
 * @brief Gives each test a registrar with nothing left over from the last one.
 * @note The registrar is a process-wide singleton, so a test that left factories behind would hand
 *       them to the next test and make the suite order-dependent.
 */
class PluginLifecycleTest : public ::testing::Test {
protected:
    void SetUp() override { simulator::Registrar::instance().clear(); }
    void TearDown() override { simulator::Registrar::instance().clear(); }
};

TEST_F(PluginLifecycleTest, AFixturePluginLoadsAndSelfRegisters) {
    /**
     * @note The whole mechanism in one assertion: the `.so` carries an undefined symbol for its
     *       registration constructor, `dlopen` resolves it against this executable, the global
     *       object's constructor runs, and the factory lands in the registrar - which the loader then
     *       claims. If `ENABLE_EXPORTS` were ever dropped, this is what would fail.
     */
    simulator::PluginLoader loader;
    simulator::PluginLoadReport report;
    loader.load(fixtureFolder("algorithms") / "StubAlgorithm_A.so",
                simulator::PluginLoader::Kind::Algorithm, report);

    ASSERT_TRUE(report.failures.empty())
        << "load failed: " << (report.failures.empty() ? "" : report.failures.front().reason);
    ASSERT_EQ(report.algorithms.size(), 1u);
    EXPECT_TRUE(static_cast<bool>(report.algorithms.front().factory));

    report.algorithms.clear();
    simulator::Registrar::instance().clear();
    loader.releaseAll();
}

TEST_F(PluginLifecycleTest, TheLoaderClaimsWhatWasRegisteredSoNothingIsLeftBehind) {
    simulator::PluginLoader loader;
    simulator::PluginLoadReport report;
    loader.load(fixtureFolder("mission_controls"), simulator::PluginLoader::Kind::MissionControl,
                report);

    ASSERT_GE(report.mission_controls.size(), 1u);

    /**
     * @note Load-then-claim is the contract: whatever a library registered belongs to the report, not
     *       to the registrar. Anything still sitting in the registrar afterwards is a factory nobody
     *       owns, and `Registrar::clear()` at teardown exists precisely to catch that case.
     */
    EXPECT_EQ(simulator::Registrar::instance().algorithmCount(), 0u);
    EXPECT_EQ(simulator::Registrar::instance().missionControlCount(), 0u);

    report.mission_controls.clear();
    loader.releaseAll();
}

TEST_F(PluginLifecycleTest, TheTeardownOrderSurvivesAFullLoadClaimDestroySequence) {
    /**
     * @note The regression test for the phase-01 crash, in the order `main` performs it:
     *       1. workers joined     - nothing concurrent here, so this step is vacuous
     *       2. instances destroyed
     *       3. factories destroyed (the report and anything holding copies)
     *       4. registrar cleared, then libraries released -> `dlclose`
     *       Reversing 3 and 4 unmaps code that live `std::function` targets still point into. The
     *       assertion is simply that the process reaches the end: this test crashing *is* the failure.
     */
    simulator::PluginLoader loader;
    simulator::PluginLoadReport report;

    loader.load(fixtureFolder("algorithms") / "StubAlgorithm_A.so",
                simulator::PluginLoader::Kind::Algorithm, report);
    loader.load(fixtureFolder("mission_controls") / "StubMissionControl_A.so",
                simulator::PluginLoader::Kind::MissionControl, report);

    ASSERT_EQ(report.algorithms.size(), 1u);
    ASSERT_EQ(report.mission_controls.size(), 1u);

    {
        /**
         * @note A factory copy taken deliberately, because that is the dangerous case. Copying a
         *       plugin's `std::function` copies a callable whose target lives in the plugin's code
         *       segment, so this local is exactly as unsafe to outlive `dlclose` as an instance.
         */
        common::MappingAlgorithmFactory algorithm_copy = report.algorithms.front().factory;
        common::MissionControlFactory mission_control_copy = report.mission_controls.front().factory;
        EXPECT_TRUE(static_cast<bool>(algorithm_copy));
        EXPECT_TRUE(static_cast<bool>(mission_control_copy));
    }

    report.algorithms.clear();
    report.mission_controls.clear();
    simulator::Registrar::instance().clear();
    loader.releaseAll();

    SUCCEED() << "load, claim, copy, destroy, clear, dlclose completed without crashing";
}

TEST_F(PluginLifecycleTest, ReleasingWithoutLoadingAnythingIsHarmless) {
    simulator::PluginLoader loader;
    loader.releaseAll();
    loader.releaseAll();
    SUCCEED() << "releaseAll is idempotent, which is what makes it safe on every exit path";
}

TEST_F(PluginLifecycleTest, ALibraryThatRegistersNothingIsAFailureNotASuccess) {
    /**
     * @note The outcome that looks like success from `dlopen`'s point of view: the library opens
     *       cleanly and publishes no factory. A loader that only checked the handle would report this
     *       as loaded and hand back an empty `std::function`, which would then fail per run instead of
     *       once at load time.
     */
    simulator::PluginLoader loader;
    simulator::PluginLoadReport report;
    loader.load(fixtureFolder("negative") / "SilentPlugin.so",
                simulator::PluginLoader::Kind::Algorithm, report);

    EXPECT_TRUE(report.algorithms.empty());
    ASSERT_EQ(report.failures.size(), 1u);
    EXPECT_NE(report.failures.front().reason.find("registered nothing"), std::string::npos)
        << "actual: " << report.failures.front().reason;

    loader.releaseAll();
}

TEST_F(PluginLifecycleTest, AMissingFileIsReportedRatherThanFatal) {
    simulator::PluginLoader loader;
    simulator::PluginLoadReport report;
    loader.load(fixtureFolder("algorithms") / "DoesNotExist.so",
                simulator::PluginLoader::Kind::Algorithm, report);

    EXPECT_TRUE(report.algorithms.empty());
    EXPECT_FALSE(report.failures.empty());

    loader.releaseAll();
}

} // namespace