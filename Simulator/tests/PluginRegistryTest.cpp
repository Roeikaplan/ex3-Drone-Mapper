/**
 * @file PluginRegistryTest.cpp
 * @brief The three properties the lazy plugin lifecycle claims, asserted directly.
 * @note The claims are: a library is mapped only when a run needs it, it is never mapped twice, and
 *       it is unmapped as soon as its last run finishes. None of them shows up in a simulation's
 *       results, so without this file they would be untestable assertions in a comment.
 * @note Every case here performs real `dlopen`/`dlclose` against the fixture plugins, for the same
 *       reason `PluginLifecycleTest` does: a mock loader would test the bookkeeping and miss the one
 *       thing that can actually crash.
 */

#include <Simulator/PluginRegistry.h>
#include <Simulator/Registrar.h>
#include <Simulator/TaskExecutor.h>

#include <gtest/gtest.h>

#include <atomic>
#include <cstddef>
#include <filesystem>
#include <string>
#include <vector>

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
 * @brief Resets the two process-wide singletons every case depends on.
 */
class PluginRegistryTest : public ::testing::Test {
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
 * @brief Discovering a whole folder maps none of it.
 * @note The first of the three properties, and the one the eager loader could not offer at all: it
 *       mapped every `.so` in the folder before deciding whether any of them was needed.
 */
TEST_F(PluginRegistryTest, DiscoveryLoadsNothing) {
    simulator::PluginRegistry registry{logger_, lifecycle_};
    const simulator::PluginRegistry::Discovery discovery =
        registry.discover(fixtureFolder("algorithms"), simulator::PluginKind::Algorithm);

    ASSERT_GE(discovery.slots.size(), 2u) << "the fixture folder should hold several plugins";
    EXPECT_EQ(registry.discoveredCount(), discovery.slots.size());
    EXPECT_EQ(registry.loadedCount(), 0u);
    EXPECT_EQ(simulator::pluginLibraryStats().opens, 0u);

    for (const simulator::PluginSlot* slot : discovery.slots) {
        EXPECT_EQ(slot->state(), simulator::PluginSlot::State::NotLoaded);
        EXPECT_EQ(slot->loadAttempts(), 0u);
    }
}

/**
 * @brief Many acquires of one plugin produce exactly one `dlopen`.
 * @note The second property. Every run of a plugin asks for its factory, and all but the first take
 *       the lock-free fast path.
 */
TEST_F(PluginRegistryTest, RepeatedAcquiresLoadTheLibraryOnlyOnce) {
    simulator::PluginRegistry registry{logger_, lifecycle_};
    simulator::PluginSlot& slot =
        *registry
             .discover(fixtureFolder("algorithms") / "StubAlgorithm_A.so",
                       simulator::PluginKind::Algorithm)
             .slots.front();

    constexpr std::size_t kUses = 8;
    registry.reserve(slot, kUses);

    for (std::size_t i = 0; i < kUses; ++i) {
        ASSERT_NE(registry.acquireAlgorithm(slot), nullptr) << slot.failureReason();
    }

    EXPECT_EQ(slot.loadAttempts(), 1u);
    EXPECT_EQ(simulator::pluginLibraryStats().opens, 1u);
    EXPECT_EQ(simulator::pluginLibraryStats().currently_open, 1u);

    for (std::size_t i = 0; i < kUses; ++i) {
        registry.release(slot);
    }
}

/**
 * @brief The library is unmapped by the release that takes the use count to zero, and not before.
 * @note The third property, and the one with a precise boundary: unmapping one release too early
 *       would pull the code out from under a run that is still holding the factory.
 */
TEST_F(PluginRegistryTest, TheLastReleaseUnloadsAndTheOnesBeforeItDoNot) {
    simulator::PluginRegistry registry{logger_, lifecycle_};
    simulator::PluginSlot& slot =
        *registry
             .discover(fixtureFolder("algorithms") / "StubAlgorithm_A.so",
                       simulator::PluginKind::Algorithm)
             .slots.front();

    registry.reserve(slot, 3);
    ASSERT_NE(registry.acquireAlgorithm(slot), nullptr);
    ASSERT_EQ(simulator::pluginLibraryStats().currently_open, 1u);

    registry.release(slot);
    EXPECT_EQ(simulator::pluginLibraryStats().currently_open, 1u) << "two uses still outstanding";
    EXPECT_EQ(slot.pendingUses(), 2u);

    registry.release(slot);
    EXPECT_EQ(simulator::pluginLibraryStats().currently_open, 1u) << "one use still outstanding";

    registry.release(slot);
    EXPECT_EQ(simulator::pluginLibraryStats().currently_open, 0u);
    EXPECT_EQ(simulator::pluginLibraryStats().closes, 1u);
    EXPECT_EQ(slot.state(), simulator::PluginSlot::State::Unloaded);
}

/**
 * @brief An unloaded plugin is never mapped a second time, even if something asks again.
 * @note The bonus is specifically "without loading them again", so the state machine has no edge
 *       back to `NotLoaded`. Nothing in the simulator can reach this - a released plugin has no runs
 *       left - but a future scheduling change might, and this is what would catch it.
 */
TEST_F(PluginRegistryTest, AnUnloadedPluginIsNeverReloaded) {
    simulator::PluginRegistry registry{logger_, lifecycle_};
    simulator::PluginSlot& slot =
        *registry
             .discover(fixtureFolder("algorithms") / "StubAlgorithm_A.so",
                       simulator::PluginKind::Algorithm)
             .slots.front();

    registry.reserve(slot, 1);
    ASSERT_NE(registry.acquireAlgorithm(slot), nullptr);
    registry.release(slot);
    ASSERT_EQ(slot.state(), simulator::PluginSlot::State::Unloaded);

    EXPECT_EQ(registry.acquireAlgorithm(slot), nullptr);
    EXPECT_EQ(slot.loadAttempts(), 1u);
    EXPECT_EQ(simulator::pluginLibraryStats().opens, 1u);
}

/**
 * @brief A plugin that failed to load is not retried either.
 * @note "Loaded once" has to cover failures too, or a folder with one broken `.so` would pay for a
 *       futile `dlopen` on every single run.
 */
TEST_F(PluginRegistryTest, AFailedLoadIsAttemptedOnceAndStaysFailed) {
    simulator::PluginRegistry registry{logger_, lifecycle_};
    simulator::PluginSlot& slot =
        *registry
             .discover(fixtureFolder("negative") / "SilentPlugin.so",
                       simulator::PluginKind::Algorithm)
             .slots.front();

    registry.reserve(slot, 5);
    for (std::size_t i = 0; i < 5; ++i) {
        EXPECT_EQ(registry.acquireAlgorithm(slot), nullptr);
    }

    EXPECT_TRUE(slot.failed());
    EXPECT_EQ(slot.loadAttempts(), 1u);
    EXPECT_EQ(simulator::pluginLibraryStats().opens, 1u);
    EXPECT_EQ(simulator::pluginLibraryStats().currently_open, 0u);

    for (std::size_t i = 0; i < 5; ++i) {
        registry.release(slot);
    }
}

/**
 * @brief Two plugins are mapped only while each is in use, never both for the whole run.
 * @note This is the shape a single-threaded batch takes: plugin A is mapped, used, unmapped, and
 *       only then is plugin B mapped. The peak is what the eager loader could never keep down.
 */
TEST_F(PluginRegistryTest, PluginsUsedInSequenceAreNeverMappedTogether) {
    simulator::PluginRegistry registry{logger_, lifecycle_};
    const simulator::PluginRegistry::Discovery discovery =
        registry.discover(fixtureFolder("algorithms"), simulator::PluginKind::Algorithm);
    ASSERT_GE(discovery.slots.size(), 2u);

    for (simulator::PluginSlot* slot : discovery.slots) {
        registry.reserve(*slot, 2);
    }

    for (simulator::PluginSlot* slot : discovery.slots) {
        ASSERT_NE(registry.acquireAlgorithm(*slot), nullptr) << slot->failureReason();
        EXPECT_EQ(simulator::pluginLibraryStats().currently_open, 1u)
            << "only the plugin currently in use should be mapped";
        registry.release(*slot);
        registry.release(*slot);
    }

    const simulator::PluginLibraryStats stats = simulator::pluginLibraryStats();
    EXPECT_EQ(stats.peak_open, 1u);
    EXPECT_EQ(stats.opens, discovery.slots.size());
    EXPECT_EQ(stats.closes, stats.opens);
    EXPECT_EQ(stats.currently_open, 0u);
}

/**
 * @brief Concurrent acquires of the same plugins still yield one load each.
 * @note The race the design has to survive: several workers reaching a plugin's first cell at once.
 *       The load path is serialised by the registry's mutex, so exactly one of them performs the
 *       `dlopen` and the rest see the finished state.
 */
TEST_F(PluginRegistryTest, ConcurrentAcquiresLoadEachLibraryExactlyOnce) {
    simulator::PluginRegistry registry{logger_, lifecycle_};
    const simulator::PluginRegistry::Discovery discovery =
        registry.discover(fixtureFolder("mission_controls"), simulator::PluginKind::MissionControl);
    ASSERT_GE(discovery.slots.size(), 2u);

    constexpr std::size_t kUsesPerSlot = 32;
    for (simulator::PluginSlot* slot : discovery.slots) {
        registry.reserve(*slot, kUsesPerSlot);
    }

    const std::size_t tasks = discovery.slots.size() * kUsesPerSlot;
    std::atomic<std::size_t> acquired{0};

    simulator::ThreadPoolExecutor executor{4};
    executor.forEach(tasks, [&](std::size_t index) {
        simulator::PluginSlot& slot = *discovery.slots[index % discovery.slots.size()];
        if (registry.acquireMissionControl(slot) != nullptr) {
            acquired.fetch_add(1, std::memory_order_relaxed);
        }
        registry.release(slot);
    });

    EXPECT_EQ(acquired.load(), tasks);
    for (const simulator::PluginSlot* slot : discovery.slots) {
        EXPECT_EQ(slot->loadAttempts(), 1u);
        EXPECT_EQ(slot->pendingUses(), 0u);
    }

    const simulator::PluginLibraryStats stats = simulator::pluginLibraryStats();
    EXPECT_EQ(stats.opens, discovery.slots.size());
    EXPECT_EQ(stats.closes, stats.opens);
    EXPECT_EQ(stats.currently_open, 0u);
}

/**
 * @brief One file named twice yields one slot, one load, and one shared use count.
 * @note Comparative mode hands the same algorithm to every pair, and a folder can hold a symlink
 *       beside its target. Both cases must resolve to a single library.
 */
TEST_F(PluginRegistryTest, TheSameFileDiscoveredTwiceSharesOneSlot) {
    simulator::PluginRegistry registry{logger_, lifecycle_};
    const fs::path file = fixtureFolder("algorithms") / "StubAlgorithm_A.so";

    simulator::PluginSlot& first =
        *registry.discover(file, simulator::PluginKind::Algorithm).slots.front();
    simulator::PluginSlot& second =
        *registry.discover(file, simulator::PluginKind::Algorithm).slots.front();

    EXPECT_EQ(&first, &second);
    EXPECT_EQ(registry.discoveredCount(), 1u);

    registry.reserve(first, 1);
    registry.reserve(second, 1);
    ASSERT_NE(registry.acquireAlgorithm(first), nullptr);
    ASSERT_NE(registry.acquireAlgorithm(second), nullptr);
    EXPECT_EQ(simulator::pluginLibraryStats().opens, 1u);

    registry.release(first);
    EXPECT_EQ(simulator::pluginLibraryStats().currently_open, 1u);
    registry.release(second);
    EXPECT_EQ(simulator::pluginLibraryStats().currently_open, 0u);
}

/**
 * @brief A plugin nobody ever runs is never mapped at all.
 * @note Zero reserved uses means zero loads. Under the eager scheme this plugin would have been
 *       mapped for the whole batch to no purpose.
 */
TEST_F(PluginRegistryTest, APluginWithNoReservedUsesIsNeverLoaded) {
    simulator::PluginRegistry registry{logger_, lifecycle_};
    const simulator::PluginRegistry::Discovery discovery =
        registry.discover(fixtureFolder("algorithms"), simulator::PluginKind::Algorithm);
    ASSERT_FALSE(discovery.slots.empty());

    registry.releaseAll();

    EXPECT_EQ(simulator::pluginLibraryStats().opens, 0u);
    EXPECT_EQ(registry.loadedCount(), 0u);
}

} // namespace
