/**
 * @file TaskExecutor.h
 * @brief The seam between deciding what work exists and deciding how it runs.
 */

#pragma once

#include <atomic>
#include <cstddef>
#include <functional>

namespace simulator {

/**
 * @brief Runs an indexed body over a known number of independent tasks.
 *
 * @note Architectural boundary: this exists so that *what* runs and *how it is scheduled* are
 *       separate decisions. The work set is fully known before execution starts, which is why the
 *       interface is an indexed loop rather than a queue - a queue exists to handle work that arrives
 *       dynamically, and here nothing arrives.
 * @note An implementation may invoke the body in **any order** and from any thread. Callers must
 *       therefore write results into a slot chosen by the index rather than by arrival, which is
 *       what keeps output identical however the work happens to be scheduled.
 * @note `forEach` returns only once every invocation has completed.
 */
class ITaskExecutor {
public:
    virtual ~ITaskExecutor() = default;

    /**
     * @brief Invoke a body once for each index in `[0, count)`.
     * @param count Number of tasks.
     * @param body Work to perform for one index; called exactly once per index.
     * @note **The body must not throw.** A concurrent implementation runs it on a worker thread,
     *       where an escaping exception calls `std::terminate` rather than unwinding to this caller.
     *       Containment therefore belongs inside the body - which is why `SimulationManager::runCell`
     *       catches everything itself rather than relying on a wrapper here.
     */
    virtual void forEach(std::size_t count, const std::function<void(std::size_t)>& body) = 0;
};

/**
 * @brief Runs every task on the calling thread, in index order.
 *
 * @note The single-threaded path, and the one used when the thread count is 1 - which the assignment
 *       defines as "the main thread does the work" rather than "one worker".
 * @note Also the executor the tests use, since it makes execution order deterministic without
 *       needing to reason about scheduling.
 */
class InlineExecutor final : public ITaskExecutor {
public:
    /**
     * @brief Invoke the body for each index, in order, on this thread.
     * @param count Number of tasks.
     * @param body Work to perform for one index.
     */
    void forEach(std::size_t count, const std::function<void(std::size_t)>& body) override;
};

/**
 * @brief Distributes tasks across a fixed pool of workers via a shared atomic cursor.
 *
 * @note **This class owns the assignment's thread rule in its entirety**, including the case where
 *       the answer is "no threads at all". `main` hands it `num_threads` and never branches on it;
 *       splitting one rule across two files is how half of it ends up violated later.
 * @note No queue, no condition variable, no result mutex. The work set is fully known before
 *       execution begins, so there is nothing for a queue to coordinate - workers simply
 *       fetch-and-increment an index until they run past the end.
 * @note Fetch-and-increment also load-balances for free, which matters here: run durations differ by
 *       orders of magnitude between a small map and a large one, or between an algorithm that
 *       finishes early and one that exhausts `max_steps`. A static range partition would leave
 *       workers idle through the tail.
 * @note Neither copyable nor movable: it holds an atomic, and it is always passed by reference.
 */
class ThreadPoolExecutor final : public ITaskExecutor {
public:
    /**
     * @brief Construct a pool for a requested thread count.
     * @param requested_threads The `num_threads` argument: workers wanted *in addition to* the
     *        calling thread. 0 and 1 both mean "the calling thread does the work".
     * @note The pool is not spawned here. The cap depends on how many tasks there turn out to be, and
     *       that is not known until the task table has been enumerated - which happens after this
     *       object is constructed.
     */
    explicit ThreadPoolExecutor(std::size_t requested_threads) noexcept;

    ThreadPoolExecutor(const ThreadPoolExecutor&) = delete;
    ThreadPoolExecutor& operator=(const ThreadPoolExecutor&) = delete;
    ThreadPoolExecutor(ThreadPoolExecutor&&) = delete;
    ThreadPoolExecutor& operator=(ThreadPoolExecutor&&) = delete;

    /**
     * @brief Run every task, spawning workers if the rule calls for any.
     * @param count Number of tasks.
     * @param body Work to perform for one index; must not throw.
     * @note Returns only once every index has been visited exactly once.
     * @note **Postcondition: no worker outlives this call.** The pool is a local of `forEach` and is
     *       joined before returning, so this object never owns a running thread between calls. That is
     *       what makes "join the workers" the *first* step of teardown without `main` having to ask for
     *       it: by the time the orchestrator is destroyed, no thread can still be inside plugin code.
     */
    void forEach(std::size_t count, const std::function<void(std::size_t)>& body) override;

    /**
     * @brief How many workers this pool would spawn for a given amount of work.
     * @param task_count How many tasks there are.
     * @return The worker count, where 0 means the calling thread does the work alone.
     *
     * @note The whole thread rule, as one testable expression. Two requirements meet here and the
     *       assignment does not say which wins:
     *       - cap at `min(N, task_count)`, so no thread is spawned with nothing to run;
     *       - the total live thread count is never exactly 2.
     *       They collide at one task with `N >= 2`, where the cap alone would give one worker plus a
     *       blocked main - exactly 2. Returning 0 there satisfies both, and is strictly better anyway:
     *       a lone worker while main waits in `join()` is a thread hand-off doing no useful work.
     * @note The invariant that falls out: **the worker count is never exactly 1**, so the total is
     *       never exactly 2.
     */
    [[nodiscard]] std::size_t workerCountFor(std::size_t task_count) const noexcept;

private:
    /**
     * @brief Claim and run indices until the cursor passes the end.
     * @param count Number of tasks.
     * @param body Work to perform for one index.
     * @note `memory_order_relaxed` is sufficient because the cursor only hands out indices; it
     *       publishes nothing. The happens-before edge that makes each task's writes visible to the
     *       calling thread is the `join()` in `forEach`.
     */
    void drain(std::size_t count, const std::function<void(std::size_t)>& body);

    std::size_t requested_threads_;
    std::atomic<std::size_t> cursor_{0};
};

} // namespace simulator
