/**
 * @file TaskExecutor.h
 * @brief The seam between deciding what work exists and deciding how it runs.
 */

#pragma once

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

} // namespace simulator
