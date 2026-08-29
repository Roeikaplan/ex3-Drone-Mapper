/**
 * @file TaskExecutor.cpp
 * @brief The two execution strategies: everything on the caller, or a self-scheduling worker pool.
 */

#include <Simulator/TaskExecutor.h>

#include <algorithm>
#include <system_error>
#include <thread>
#include <vector>

namespace simulator {

/**
 * @brief Invoke the body for each index, in order, on this thread.
 * @param count Number of tasks.
 * @param body Work to perform for one index.
 * @note Deliberately does not catch anything. Whether a failing task should end the batch is the
 *       caller's decision, not the scheduler's, and the run body already wraps itself - which it
 *       must, because a concurrent executor cannot let an exception escape a worker at all.
 */
void InlineExecutor::forEach(std::size_t count, const std::function<void(std::size_t)>& body) {
    for (std::size_t index = 0; index < count; ++index) {
        body(index);
    }
}

/**
 * @brief Construct a pool for a requested thread count.
 * @param requested_threads Workers wanted in addition to the calling thread.
 */
ThreadPoolExecutor::ThreadPoolExecutor(std::size_t requested_threads) noexcept
    : requested_threads_(requested_threads) {}

/**
 * @brief How many workers this pool would spawn for a given amount of work.
 * @param task_count How many tasks there are.
 * @return The worker count; 0 means the calling thread works alone.
 * @note Both guards return 0, for different reasons. `requested_threads_ < 2` is the assignment's
 *       "missing or 1 means the main thread does the work". `task_count < 2` is the two capping rules
 *       reconciled: one task can only ever occupy one worker, and one worker plus a blocked main is
 *       the forbidden total of exactly 2.
 */
std::size_t ThreadPoolExecutor::workerCountFor(std::size_t task_count) const noexcept {
    if (requested_threads_ < 2 || task_count < 2) {
        return 0;
    }
    return std::min(requested_threads_, task_count);
}

/**
 * @brief Claim and run indices until the cursor passes the end.
 * @param count Number of tasks.
 * @param body Work to perform for one index.
 * @note The cursor deliberately overshoots `count` - every thread that leaves this loop has consumed
 *       one index beyond the end. That is what makes the exit condition a plain comparison with no
 *       compare-exchange retry.
 */
void ThreadPoolExecutor::drain(std::size_t count, const std::function<void(std::size_t)>& body) {
    for (std::size_t index = cursor_.fetch_add(1, std::memory_order_relaxed); index < count;
         index = cursor_.fetch_add(1, std::memory_order_relaxed)) {
        body(index);
    }
}

/**
 * @brief Run every task, spawning workers if the rule calls for any.
 * @param count Number of tasks.
 * @param body Work to perform for one index; must not throw.
 * @note The calling thread does **not** take tasks when workers exist: it blocks in `join()`, which
 *       is what the assignment describes and what makes the live thread count `1 + workers`.
 * @note The trailing `drain` is the spawn-failure path. `std::thread`'s constructor can throw when the
 *       system is out of resources; if that happens partway through, the workers that did start still
 *       drain the table, and anything left over is finished here rather than silently dropped. In the
 *       normal case the cursor is already past the end and this returns immediately.
 */
void ThreadPoolExecutor::forEach(std::size_t count, const std::function<void(std::size_t)>& body) {
    cursor_.store(0, std::memory_order_relaxed);

    const std::size_t workers = workerCountFor(count);
    if (workers == 0) {
        drain(count, body);
        return;
    }

    std::vector<std::thread> pool;
    pool.reserve(workers);
    try {
        for (std::size_t i = 0; i < workers; ++i) {
            pool.emplace_back([this, count, &body] { drain(count, body); });
        }
    } catch (const std::system_error&) {
        /**
         * @note Swallowed on purpose: the threads that did start are already working, and the join
         *       plus the final drain below complete the batch regardless of how many there are.
         */
    }

    for (std::thread& worker : pool) {
        worker.join();
    }

    drain(count, body);
}

} // namespace simulator
