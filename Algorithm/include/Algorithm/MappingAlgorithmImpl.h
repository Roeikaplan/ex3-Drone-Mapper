/**
 * @file MappingAlgorithmImpl.h
 * @brief Deterministic frontier-based exploration of a bounded voxel region.
 */

#pragma once

#include <Common/IMappingAlgorithm.h>

#include <UserCommon/VoxelGrid.h>

#include <cstddef>
#include <cstdint>
#include <deque>
#include <vector>

namespace algorithm {

/**
 * @brief Reusable working memory for one frontier search.
 *
 * @note Exists purely so the search stops allocating. It is re-run on most steps of a mission, and on
 *       the large scenarios the grid reaches ~216,000 cells - so a locally-declared `visited` and
 *       `parent` meant allocating and zero-filling about 2 MB *per call*, thousands of times per run.
 *       Serially that is nearly free, because the allocator hands back the same warm block every time.
 *       Under a thread pool it is not: the block is large enough to be served by `mmap`, so four
 *       workers turn it into a stream of map/unmap syscalls and page faults, and the run gets *slower*
 *       with more threads. Reusing one buffer per algorithm instance removes the allocation entirely.
 * @note **Per algorithm instance, and therefore per run.** One instance is created per simulation run
 *       and never shared between them, so this buffer is touched by exactly one thread for its whole
 *       life and needs no synchronisation.
 * @note Visits are recorded with a **generation stamp** rather than a boolean, so consecutive searches
 *       do not have to re-zero the array: a cell counts as visited only when its stamp equals the
 *       current generation, and bumping the generation invalidates every entry at once.
 * @note The stamp is deliberately **one byte**, not four. It is the array the search probes for every
 *       neighbour of every expanded node, so its width is the dominant memory traffic in the hot loop -
 *       a 32-bit stamp measured 47% slower on the large scenarios despite eliminating the same
 *       allocations. One byte only allows 255 live generations, so the array is cleared when the
 *       counter wraps; that costs one `memset` per 255 searches instead of one per search.
 * @note The frontier queue lives here too, as a vector plus a head index rather than a `std::deque`.
 *       A deque allocates in fixed-size chunks, so a search that enqueues most of a 216,000-cell grid
 *       made *thousands of small allocations* - and many small allocations contend far worse across
 *       threads than one large one, because every thread is hitting the allocator's free lists rather
 *       than making one `mmap`. Reusing one vector removes them all; `clear()` keeps the capacity, so
 *       only the first search of a run ever grows it.
 */
struct FrontierSearchScratch {
    /**
     * @brief Generation in which each cell was last visited; 0 means never.
     */
    std::vector<std::uint8_t> visit_stamp{};

    /**
     * @brief Predecessor of each cell in the current search, as a linear index.
     * @note Meaningful only for cells whose stamp matches the current generation. Stale entries from
     *       an earlier generation are never read, because a cell is only ever walked back from after
     *       being reached in this one.
     */
    std::vector<std::uint32_t> parent{};

    /**
     * @brief Cells discovered but not yet expanded, in discovery order.
     * @note A vector used as a FIFO rather than a `std::deque`, with `queue_head` marking the front.
     *       Entries before the head are spent and simply left in place - nothing is ever erased from
     *       the middle, so no element is moved and the order the search sees is exactly a deque's.
     */
    std::vector<user_common::VoxelIndex> queue{};

    /**
     * @brief Index of the next cell to expand.
     * @note The queue is empty when this reaches `queue.size()`. Advancing an index rather than
     *       erasing the front is what makes dequeuing O(1) with no moves.
     */
    std::size_t queue_head = 0;

    /**
     * @brief The current search's generation, always in `[1, kMaxGeneration]`.
     * @note 0 is reserved for "never visited", so it is never a live generation.
     */
    std::uint8_t generation = 0;

    /**
     * @brief Highest stamp value before the counter has to wrap and the array be cleared.
     */
    static constexpr std::uint8_t kMaxGeneration = 255;

    /**
     * @brief Marker for "no predecessor", used for the search's start cell.
     * @note A distinct sentinel rather than a signed -1 so both arrays stay 32-bit: halving the
     *       predecessor array against a 64-bit one halves the memory traffic of the hot loop, and a
     *       grid large enough to reach this value could not be allocated in the first place.
     */
    static constexpr std::uint32_t kNoParent = 0xFFFFFFFFu;

    /**
     * @brief Size the buffers for a grid and begin a new search.
     * @param cells How many voxels the grid holds.
     * @note Resizes only when the grid's size changes, which in practice means once per run.
     */
    void beginSearch(std::size_t cells);

    /**
     * @brief Whether a cell has been reached in the current search.
     * @param linear_index The cell's linear index.
     * @return True when it was already visited this generation.
     */
    [[nodiscard]] bool visited(std::size_t linear_index) const {
        return visit_stamp[linear_index] == generation;
    }

    /**
     * @brief Record a cell as reached, and from where.
     * @param linear_index The cell's linear index.
     * @param from The predecessor's linear index, or `kNoParent` for the start cell.
     */
    void visit(std::size_t linear_index, std::uint32_t from) {
        visit_stamp[linear_index] = generation;
        parent[linear_index] = from;
    }

    /**
     * @brief Whether every discovered cell has been expanded.
     * @return True when nothing is left to expand.
     */
    [[nodiscard]] bool queueEmpty() const noexcept { return queue_head >= queue.size(); }

    /**
     * @brief Add a cell to the back of the queue.
     * @param cell The cell to expand later.
     * @note After the first search of a run this never allocates: `beginSearch` clears the vector
     *       without releasing its capacity, and a cell can be enqueued at most once per search.
     */
    void enqueue(const user_common::VoxelIndex& cell) { queue.push_back(cell); }

    /**
     * @brief Take the cell at the front of the queue.
     * @return A copy of the oldest cell not yet expanded.
     * @note Only valid when `queueEmpty()` is false.
     * @note **By value, deliberately.** The caller expands this cell and enqueues its neighbours in
     *       the same iteration, and a `push_back` that grows the vector would leave any reference
     *       into it dangling. Three integers are far cheaper than that hazard.
     */
    [[nodiscard]] user_common::VoxelIndex dequeue() { return queue[queue_head++]; }
};

/**
 * @brief Explores a region by repeatedly surveying a cell and travelling to the nearest frontier.
 *
 * Each cycle: scan the current voxel in all six axis directions, then breadth-first search the
 * already-observed `Empty` space for the closest cell that borders something still `Unmapped`, and
 * compile the route there into per-step movement commands.
 *
 * @note **This class carries the run's collision safety, and that is not obvious from where it
 *       sits.** The mission control can only refuse a move into a cell *already observed*
 *       `Occupied` - it has no ground truth. Because the search traverses only cells proven `Empty`
 *       by an actual scan, every path emitted here is through space the drone has already seen to be
 *       free. Relaxing that to include `Unmapped` or `PotentiallyOccupied` cells would fly the drone
 *       into walls, and nothing downstream would catch it.
 * @note Planning reads the output map and **never writes it** - the base class holds it as
 *       `const IMap3D&`, so this is compiler-enforced. Writing would also mean reasoning about its
 *       own guesses rather than about measurements.
 * @note Keeps no pose of its own. Every decision comes from the `DroneState` it is handed plus the
 *       map, so a drone that gets moved unexpectedly is planned for correctly on the next call.
 * @note Termination is guaranteed by never choosing an already-surveyed cell as a destination: the
 *       set of candidate targets strictly shrinks, so the search runs out rather than cycling.
 */
class MappingAlgorithmImpl final : public common::IMappingAlgorithm {
public:
    /**
     * @brief Inherit the dependencies constructor from the interface.
     * @note `IMappingAlgorithm` supplies one, unlike `IMissionControl`, so nothing more is needed.
     */
    using common::IMappingAlgorithm::IMappingAlgorithm;

    /**
     * @brief Decide the next command.
     * @param state The drone's current pose and step index.
     * @param latest_scan The previous scan, or null on the first call.
     * @return A scan while surveying, a movement micro-step while travelling, or a terminal status.
     * @note @p latest_scan is deliberately unused. The mission control has already applied it to the
     *       output map before this call, and the map is the accumulated picture of everything seen
     *       so far; re-reading the raw scan would double-count the most recent one.
     */
    [[nodiscard]] common::types::MappingStepCommand nextStep(
        const common::types::DroneState& state,
        const common::types::LidarScanResult* latest_scan) override;

private:
    /**
     * @brief Queued micro-steps for the current sweep or route.
     * @note Drained one per call. A whole route is compiled up front because the map cannot change
     *       while it executes - nothing scans mid-route - so re-planning each step would produce the
     *       same answer at a cost.
     */
    std::deque<common::types::MappingStepCommand> plan_{};

    /**
     * @brief Which cells have already had their scan sweep issued, indexed by linear voxel id.
     * @note Sized lazily on the first call, once the map's geometry is known. A flat array rather
     *       than a set: it is consulted for every node the search expands.
     */
    std::vector<std::uint8_t> surveyed_{};

    /**
     * @brief Working memory for the frontier search, reused across every call.
     * @note A member rather than a local purely for performance; it carries no state between searches
     *       beyond the generation counter that lets the visited marks be discarded for free.
     */
    FrontierSearchScratch scratch_{};

    /**
     * @brief Latches once exploration is complete.
     * @note Without it, a finished run would re-search the whole grid on every remaining step of the
     *       mission's budget.
     */
    bool finished_ = false;
};

} // namespace algorithm
