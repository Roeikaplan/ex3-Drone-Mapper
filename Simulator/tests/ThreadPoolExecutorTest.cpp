/**
 * @file ThreadPoolExecutorTest.cpp
 * @brief The thread rule, asserted rather than assumed.
 * @note The rule is unusual enough that a plausible-looking implementation can satisfy every
 *       behavioural test and still be wrong about the thread *count*. So `workerCountFor` is checked
 *       directly as a table, and the behavioural cases then confirm the count is what the table says.
 */

#include <Simulator/ErrorLogger.h>
#include <Simulator/TaskExecutor.h>

#include <gtest/gtest.h>

#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <set>
#include <string>
#include <thread>
#include <vector>

namespace {

namespace fs = std::filesystem;

/**
 * @brief Run a body over `count` tasks and record which thread handled each index.
 * @param executor The executor under test.
 * @param count Number of tasks.
 * @return One thread id per index, in index order.
 * @note Recording per index rather than counting threads is what lets one run answer three separate
 *       questions: how many threads worked, whether the caller was one of them, and whether any index
 *       was visited twice.
 */
[[nodiscard]] std::vector<std::thread::id> recordThreads(simulator::ITaskExecutor& executor,
                                                         std::size_t count) {
    std::vector<std::thread::id> owners(count);
    executor.forEach(count, [&owners](std::size_t index) {
        owners[index] = std::this_thread::get_id();
    });
    return owners;
}

/**
 * @brief How many distinct threads appear in a recording.
 * @param owners Per-index thread ids.
 * @return The number of distinct ids.
 */
[[nodiscard]] std::size_t distinctThreads(const std::vector<std::thread::id>& owners) {
    return std::set<std::thread::id>(owners.begin(), owners.end()).size();
}

TEST(ThreadPoolExecutor, TheThreadRuleAsATable) {
    /**
     * @note Each row is a clause of the assignment's rule. A worker count of 0 means the calling
     *       thread does the work, so the live total is 1; otherwise it is 1 + the worker count.
     */
    EXPECT_EQ(simulator::ThreadPoolExecutor{0}.workerCountFor(100), 0u) << "absent means main works";
    EXPECT_EQ(simulator::ThreadPoolExecutor{1}.workerCountFor(100), 0u) << "1 means main works";
    EXPECT_EQ(simulator::ThreadPoolExecutor{2}.workerCountFor(100), 2u);
    EXPECT_EQ(simulator::ThreadPoolExecutor{4}.workerCountFor(100), 4u);
    EXPECT_EQ(simulator::ThreadPoolExecutor{8}.workerCountFor(3), 3u) << "capped at the task count";
    EXPECT_EQ(simulator::ThreadPoolExecutor{4}.workerCountFor(0), 0u) << "no tasks, no threads";
}

TEST(ThreadPoolExecutor, TheLiveThreadCountIsNeverExactlyTwo) {
    /**
     * @note The one place the assignment's two capping rules collide. Capping at `min(N, tasks)`
     *       alone would give a single worker plus a blocked main - a live total of exactly 2, which
     *       the rule forbids. Falling back to the calling thread satisfies both, and wastes nothing:
     *       a lone worker while main waits in `join()` does no more work than main would.
     */
    for (std::size_t requested = 2; requested <= 16; ++requested) {
        const simulator::ThreadPoolExecutor executor{requested};
        for (std::size_t tasks = 0; tasks <= 32; ++tasks) {
            const std::size_t workers = executor.workerCountFor(tasks);
            EXPECT_NE(workers, 1u) << "requested=" << requested << " tasks=" << tasks
                                   << ": one worker plus a blocked main is the forbidden total of 2";
            EXPECT_LE(workers, tasks) << "never spawn a thread with nothing to run";
            EXPECT_LE(workers, requested) << "never exceed what was asked for";
        }
    }
}

TEST(ThreadPoolExecutor, ASingleThreadRunsEverythingOnTheCaller) {
    simulator::ThreadPoolExecutor executor{1};
    const std::vector<std::thread::id> owners = recordThreads(executor, 10);

    ASSERT_EQ(owners.size(), 10u);
    for (const std::thread::id& owner : owners) {
        EXPECT_EQ(owner, std::this_thread::get_id());
    }
}

TEST(ThreadPoolExecutor, WorkersDoTheWorkAndTheCallerDoesNot) {
    simulator::ThreadPoolExecutor executor{3};
    const std::vector<std::thread::id> owners = recordThreads(executor, 200);

    EXPECT_LE(distinctThreads(owners), 3u) << "never more workers than were asked for";

    /**
     * @note The assignment says main blocks in `join()` while the workers run. If main also took
     *       tasks the results would still be correct, so nothing else in the suite would notice -
     *       this is the only assertion that pins the live thread count to 1 + N.
     */
    for (const std::thread::id& owner : owners) {
        EXPECT_NE(owner, std::this_thread::get_id()) << "the calling thread must not take tasks";
    }
}

TEST(ThreadPoolExecutor, MoreThreadsThanTasksSpawnsNoneIdle) {
    simulator::ThreadPoolExecutor executor{16};
    const std::vector<std::thread::id> owners = recordThreads(executor, 3);
    EXPECT_LE(distinctThreads(owners), 3u);
}

TEST(ThreadPoolExecutor, EveryIndexRunsExactlyOnce) {
    simulator::ThreadPoolExecutor executor{4};

    constexpr std::size_t kTasks = 500;
    std::vector<std::atomic<std::size_t>> visits(kTasks);
    for (std::atomic<std::size_t>& visit : visits) {
        visit.store(0);
    }

    executor.forEach(kTasks, [&visits](std::size_t index) {
        visits[index].fetch_add(1, std::memory_order_relaxed);
    });

    for (std::size_t index = 0; index < kTasks; ++index) {
        EXPECT_EQ(visits[index].load(), 1u) << "at index " << index;
    }
}

TEST(ThreadPoolExecutor, AnEmptyTableSpawnsNothing) {
    simulator::ThreadPoolExecutor executor{8};
    std::size_t calls = 0;
    executor.forEach(0, [&calls](std::size_t) { ++calls; });
    EXPECT_EQ(calls, 0u);
}

TEST(ThreadPoolExecutor, ResultsLandInIndexOrderWhateverTheSchedule) {
    /**
     * @note The property the whole report pipeline rests on. The work is jittered so that completion
     *       order is genuinely not index order - early indices are made slowest, so a scheme that
     *       recorded results as tasks finished would come out close to reversed.
     */
    simulator::ThreadPoolExecutor executor{4};

    constexpr std::size_t kTasks = 40;
    std::vector<std::size_t> slots(kTasks, 0);
    std::atomic<std::size_t> completion{0};
    std::vector<std::size_t> completion_order(kTasks, 0);

    executor.forEach(kTasks, [&](std::size_t index) {
        std::this_thread::sleep_for(std::chrono::microseconds((kTasks - index) * 200));
        slots[index] = index * 10;
        completion_order[index] = completion.fetch_add(1, std::memory_order_relaxed);
    });

    for (std::size_t index = 0; index < kTasks; ++index) {
        EXPECT_EQ(slots[index], index * 10) << "at index " << index;
    }

    std::vector<std::size_t> index_order(kTasks);
    for (std::size_t i = 0; i < kTasks; ++i) {
        index_order[i] = i;
    }
    EXPECT_NE(completion_order, index_order)
        << "the jitter failed to reorder anything, so this run proved nothing";
}

TEST(ErrorLogger, ConcurrentWritersProduceWholeLines) {
    /**
     * @note The logger is the only synchronised object in the design, so this is the one place a lock
     *       is actually load-bearing. A torn write would interleave two messages on one line, which
     *       would corrupt the error log exactly when it is most needed.
     */
    const fs::path log_file = fs::temp_directory_path() /
                              ("ex3_concurrent_log_" + std::to_string(::getpid()) + ".log");
    std::error_code ec;
    fs::remove(log_file, ec);

    constexpr std::size_t kWriters = 8;
    constexpr std::size_t kPerWriter = 200;

    {
        simulator::ErrorLogger logger{log_file};
        simulator::ThreadPoolExecutor executor{kWriters};
        executor.forEach(kWriters, [&logger](std::size_t writer) {
            for (std::size_t i = 0; i < kPerWriter; ++i) {
                logger.log("CONCURRENT", "writer " + std::to_string(writer) + " message " +
                                             std::to_string(i));
            }
        });

        EXPECT_EQ(logger.errorCount(), kWriters * kPerWriter);
    }

    std::ifstream stream(log_file);
    ASSERT_TRUE(stream.is_open());

    std::size_t lines = 0;
    std::string line;
    while (std::getline(stream, line)) {
        ++lines;
        EXPECT_EQ(line.find("[CONCURRENT]"), 0u) << "torn line: " << line;
        EXPECT_EQ(line.find("[CONCURRENT]", 1), std::string::npos)
            << "two messages landed on one line: " << line;
    }
    EXPECT_EQ(lines, kWriters * kPerWriter);

    fs::remove(log_file, ec);
}

} // namespace
