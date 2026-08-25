/**
 * @file TaskExecutor.cpp
 * @brief The single-threaded executor.
 */

#include <Simulator/TaskExecutor.h>

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

} // namespace simulator
