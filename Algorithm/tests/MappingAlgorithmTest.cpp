/**
 * @file MappingAlgorithmTest.cpp
 * @brief Coverage of the bootstrap, the safety invariant, route compilation, and termination.
 * @note The safety cases are the ones that matter most. Nothing downstream can catch a planner that
 *       routes through unobserved space - the mission control has no ground truth - so a regression
 *       there would surface as a collision in a full run rather than as a failing assertion.
 */

#include <Algorithm/MappingAlgorithmImpl.h>

#include <UserCommon/VoxelGrid.h>

#include <gtest/gtest.h>

#include <cstddef>
#include <map>
#include <vector>

namespace {

using namespace common;
using algorithm::MappingAlgorithmImpl;
using user_common::VoxelGrid;
using user_common::VoxelIndex;

/**
 * @brief Ordering so voxel indices can key a map.
 * @note An explicit comparator rather than an `operator<`. A free operator declared here would not
 *       be found by `std::less`, whose lookup reaches `user_common` by argument-dependent lookup and
 *       never sees this file's anonymous namespace - and putting an ordering into the shared header
 *       to satisfy one test would be the tail wagging the dog. The planner itself needs no ordering
 *       at all; it indexes flat arrays.
 */
struct VoxelIndexLess {
    /**
     * @brief Compare two indices lexicographically.
     * @param lhs Left index.
     * @param rhs Right index.
     * @return True when @p lhs sorts before @p rhs.
     */
    [[nodiscard]] bool operator()(const VoxelIndex& lhs, const VoxelIndex& rhs) const {
        if (lhs.x != rhs.x) {
            return lhs.x < rhs.x;
        }
        if (lhs.y != rhs.y) {
            return lhs.y < rhs.y;
        }
        return lhs.z < rhs.z;
    }
};

/**
 * @brief A map the test fills cell by cell.
 * @note Sparse and read-only from the planner's point of view, which is all `IMap3D` offers it.
 *       Everything not written reads `Unmapped`, exactly like a real output map before it is
 *       scanned.
 */
class HandBuiltMap final : public IMap3D {
public:
    /**
     * @brief Construct a cubic region anchored at the origin.
     * @param cells_per_axis Number of voxels along each axis.
     * @param resolution_cm Voxel edge length.
     */
    HandBuiltMap(std::int64_t cells_per_axis, double resolution_cm) {
        config_.resolution = resolution_cm * cm;
        const double span = static_cast<double>(cells_per_axis) * resolution_cm;
        config_.boundaries.max_x = span * x_extent[cm];
        config_.boundaries.max_y = span * y_extent[cm];
        config_.boundaries.max_height = span * z_extent[cm];
        grid_ = VoxelGrid::from(config_);
    }

    /**
     * @brief Record a cell's occupancy.
     * @param index Cell to write.
     * @param value Occupancy to store.
     */
    void put(const VoxelIndex& index, types::VoxelOccupancy value) { cells_[index] = value; }

    /**
     * @brief Mark a run of cells `Empty`.
     * @param from First cell.
     * @param count How many cells.
     * @param delta Direction to step in.
     */
    void putEmptyRun(const VoxelIndex& from, std::int64_t count, const VoxelIndex& delta) {
        VoxelIndex at = from;
        for (std::int64_t i = 0; i < count; ++i) {
            put(at, types::VoxelOccupancy::Empty);
            at = VoxelIndex{at.x + delta.x, at.y + delta.y, at.z + delta.z};
        }
    }

    /**
     * @brief The grid this map describes.
     * @return Its geometry.
     */
    [[nodiscard]] const VoxelGrid& grid() const noexcept { return grid_; }

    /**
     * @brief Occupancy at a world position.
     * @param pos Position to sample.
     * @return The stored value, or `Unmapped` when never written.
     */
    [[nodiscard]] types::VoxelOccupancy atVoxel(const Position3D& pos) const override {
        const std::optional<VoxelIndex> index = grid_.indexOf(pos);
        if (!index) {
            return types::VoxelOccupancy::OutOfBounds;
        }
        const auto found = cells_.find(*index);
        return found == cells_.end() ? types::VoxelOccupancy::Unmapped : found->second;
    }

    /**
     * @brief This map's geometry.
     * @return The configuration.
     */
    [[nodiscard]] types::MapConfig getMapConfig() const override { return config_; }

    /**
     * @brief Whether a position lies inside the region.
     * @param pos Position to test.
     * @return True when it maps to a cell.
     */
    [[nodiscard]] bool isInBounds(const Position3D& pos) const override {
        return grid_.indexOf(pos).has_value();
    }

private:
    types::MapConfig config_{};
    VoxelGrid grid_{};
    std::map<VoxelIndex, types::VoxelOccupancy, VoxelIndexLess> cells_{};
};

/**
 * @brief A drone whose limits permit a full hop in one command.
 * @param resolution_cm Voxel edge length the drone must be able to cross.
 * @return The configuration.
 */
[[nodiscard]] types::DroneConfigData permissiveDrone(double resolution_cm) {
    types::DroneConfigData drone{};
    drone.radius = 1.0 * cm;
    drone.max_rotate = 90.0 * horizontal_angle[deg];
    drone.max_advance = resolution_cm * cm;
    drone.max_elevate = resolution_cm * cm;
    return drone;
}

/**
 * @brief Build a planner over a map.
 * @param map The map to plan against.
 * @param drone The drone's limits.
 * @return The planner.
 */
[[nodiscard]] MappingAlgorithmImpl makePlanner(const HandBuiltMap& map,
                                               const types::DroneConfigData& drone) {
    static types::MissionConfigData mission{};
    static types::LidarConfigData lidar{};
    return MappingAlgorithmImpl{MappingAlgorithmDependencies{mission, lidar, drone, map}};
}

/**
 * @brief A drone state at a cell's centre, facing east.
 * @param map The map supplying the geometry.
 * @param index Cell the drone occupies.
 * @param step Step index to report.
 * @return The state.
 */
[[nodiscard]] types::DroneState stateAt(const HandBuiltMap& map, const VoxelIndex& index,
                                        std::size_t step = 0) {
    types::DroneState state{};
    state.position = map.grid().centreOf(index);
    state.heading = Orientation{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]};
    state.step_index = step;
    return state;
}

/**
 * @brief The mission opens with a survey rather than a move.
 */
TEST(MappingAlgorithm, TheFirstCommandIsAScanNotAMove) {
    /**
     * @note The drone's own cell starts `Unmapped`, so there is nowhere the search could legally
     *       travel. Surveying first is what makes the cell `Empty` and unlocks everything after.
     */
    HandBuiltMap map{10, 5.0};
    MappingAlgorithmImpl planner = makePlanner(map, permissiveDrone(5.0));

    const types::MappingStepCommand command = planner.nextStep(stateAt(map, {5, 5, 5}), nullptr);

    EXPECT_TRUE(command.scan_orientation.has_value());
    EXPECT_FALSE(command.movement.has_value());
    EXPECT_EQ(command.status, types::AlgorithmStatus::Working);
}

/**
 * @brief The opening survey is six scans, one per axis direction.
 * @note All six are issued unconditionally, before anything is known. Scanning only where something
 *       is already suspected would make the first cycle depend on a picture that does not exist yet.
 */
TEST(MappingAlgorithm, TheSurveyIsSixScansCoveringEveryAxis) {
    HandBuiltMap map{10, 5.0};
    MappingAlgorithmImpl planner = makePlanner(map, permissiveDrone(5.0));

    for (int i = 0; i < 6; ++i) {
        const types::MappingStepCommand command =
            planner.nextStep(stateAt(map, {5, 5, 5}, static_cast<std::size_t>(i)), nullptr);
        EXPECT_TRUE(command.scan_orientation.has_value()) << "command " << i;
        EXPECT_FALSE(command.movement.has_value()) << "command " << i;
    }
}

/**
 * @brief A route through observed `Empty` space is compiled into movement commands.
 * @note The survey is what unlocks travel at all: only cells an actual scan proved `Empty` are
 *       traversable, so this is the first call that can legally produce a movement.
 * @note The emitted command is the advance itself rather than a turn, because the drone already
 *       faces along the route. `TurnsBeforeAdvancingWhenTheRouteChangesAxis` covers the other case.
 */
TEST(MappingAlgorithm, TravelsToAFrontierThroughObservedEmptySpace) {
    HandBuiltMap map{10, 5.0};
    /**
     * @note A corridor east from the drone. Its far end borders unmapped space, so it is a frontier
     *       worth reaching, and every cell between is `Empty`.
     */
    map.putEmptyRun({5, 5, 5}, 4, {1, 0, 0});

    MappingAlgorithmImpl planner = makePlanner(map, permissiveDrone(5.0));
    for (int i = 0; i < 6; ++i) {
        (void)planner.nextStep(stateAt(map, {5, 5, 5}, static_cast<std::size_t>(i)), nullptr);
    }

    const types::MappingStepCommand command = planner.nextStep(stateAt(map, {5, 5, 5}, 6), nullptr);
    ASSERT_TRUE(command.movement.has_value());
    EXPECT_EQ(command.movement->type, types::MovementCommandType::Advance)
        << "already facing east, so no turn is needed first";
}

/**
 * @brief **The safety invariant**: a frontier reachable only through `Unmapped` cells is refused.
 * @note The most important case in the file. A violation cannot be caught anywhere downstream, so it
 *       would surface as a collision in a full run rather than as a failing assertion here.
 */
TEST(MappingAlgorithm, NeverRoutesThroughUnobservedSpace) {
    /**
     * @note **The safety invariant.** The only frontier sits beyond a gap of `Unmapped` cells. The
     *       planner must decline rather than cross it - nothing downstream can catch a route through
     *       space the drone has never seen, because the mission control has no ground truth either.
     */
    HandBuiltMap map{10, 5.0};
    map.put({5, 5, 5}, types::VoxelOccupancy::Empty);
    map.putEmptyRun({8, 5, 5}, 2, {1, 0, 0});

    MappingAlgorithmImpl planner = makePlanner(map, permissiveDrone(5.0));
    for (int i = 0; i < 6; ++i) {
        (void)planner.nextStep(stateAt(map, {5, 5, 5}, static_cast<std::size_t>(i)), nullptr);
    }

    const types::MappingStepCommand command = planner.nextStep(stateAt(map, {5, 5, 5}, 6), nullptr);
    EXPECT_FALSE(command.movement.has_value())
        << "the isolated region is unreachable; refusing to move is the only safe answer";
    EXPECT_NE(command.status, types::AlgorithmStatus::Working);
}

/**
 * @brief `PotentiallyOccupied` is impassable, exactly like `Occupied`.
 * @note The same invariant as the previous test, for the other non-`Empty` value a scan can produce.
 *       Only cells proven `Empty` may be traversed - "not known to be a wall" is not good enough.
 */
TEST(MappingAlgorithm, NeverRoutesThroughPotentiallyOccupiedSpace) {
    /**
     * @note `PotentiallyOccupied` exists precisely because the sensor detected something it could not
     *       place. Treating it as passable would fly the drone at whatever that was.
     */
    HandBuiltMap map{10, 5.0};
    map.put({5, 5, 5}, types::VoxelOccupancy::Empty);
    map.put({6, 5, 5}, types::VoxelOccupancy::PotentiallyOccupied);
    map.putEmptyRun({7, 5, 5}, 2, {1, 0, 0});

    MappingAlgorithmImpl planner = makePlanner(map, permissiveDrone(5.0));
    for (int i = 0; i < 6; ++i) {
        (void)planner.nextStep(stateAt(map, {5, 5, 5}, static_cast<std::size_t>(i)), nullptr);
    }

    const types::MappingStepCommand command = planner.nextStep(stateAt(map, {5, 5, 5}, 6), nullptr);
    EXPECT_FALSE(command.movement.has_value());
}

/**
 * @brief A route that changes axis emits the rotation before the advance.
 * @note The drone advances along its heading, so the two commands issued in the wrong order would
 *       travel along the *old* axis. The mission control would accept that - the command is legal
 *       and within limits - and the drone would simply end up somewhere else.
 */
TEST(MappingAlgorithm, TurnsBeforeAdvancingWhenTheRouteChangesAxis) {
    HandBuiltMap map{10, 5.0};
    map.putEmptyRun({5, 5, 5}, 4, {0, 1, 0});

    MappingAlgorithmImpl planner = makePlanner(map, permissiveDrone(5.0));
    for (int i = 0; i < 6; ++i) {
        (void)planner.nextStep(stateAt(map, {5, 5, 5}, static_cast<std::size_t>(i)), nullptr);
    }

    const types::MappingStepCommand command = planner.nextStep(stateAt(map, {5, 5, 5}, 6), nullptr);
    ASSERT_TRUE(command.movement.has_value());
    EXPECT_EQ(command.movement->type, types::MovementCommandType::Rotate)
        << "the route runs south while the drone faces east";
}

/**
 * @brief A vertical route elevates directly, with no rotation first.
 * @note The counterpart to the axis-change case above: altitude is not a bearing, so `elevate` is
 *       independent of heading and a turn inserted here would be a step spent achieving nothing.
 */
TEST(MappingAlgorithm, AVerticalHopElevatesWithoutTurning) {
    HandBuiltMap map{10, 5.0};
    map.putEmptyRun({5, 5, 5}, 4, {0, 0, 1});

    MappingAlgorithmImpl planner = makePlanner(map, permissiveDrone(5.0));
    for (int i = 0; i < 6; ++i) {
        (void)planner.nextStep(stateAt(map, {5, 5, 5}, static_cast<std::size_t>(i)), nullptr);
    }

    const types::MappingStepCommand command = planner.nextStep(stateAt(map, {5, 5, 5}, 6), nullptr);
    ASSERT_TRUE(command.movement.has_value());
    EXPECT_EQ(command.movement->type, types::MovementCommandType::Elevate)
        << "altitude is not a bearing, so there is nothing to turn toward";
}

/**
 * @brief Every emitted command fits inside the drone's per-command limits.
 * @note The planner must split a whole-voxel hop into micro-steps rather than assume the drone can
 *       cross a cell in one command. All three primitives are checked, since each has its own limit.
 */
TEST(MappingAlgorithm, EveryEmittedCommandRespectsTheDronesLimits) {
    /**
     * @note A drone that can only manage a fifth of a voxel per command. Emitting a whole hop would
     *       have the mission control refuse the very first move and end the run.
     */
    HandBuiltMap map{10, 10.0};
    map.putEmptyRun({5, 5, 5}, 4, {1, 0, 0});

    types::DroneConfigData tiny{};
    tiny.radius = 1.0 * cm;
    tiny.max_rotate = 15.0 * horizontal_angle[deg];
    tiny.max_advance = 2.0 * cm;
    tiny.max_elevate = 2.0 * cm;

    MappingAlgorithmImpl planner = makePlanner(map, tiny);
    for (int i = 0; i < 6; ++i) {
        (void)planner.nextStep(stateAt(map, {5, 5, 5}, static_cast<std::size_t>(i)), nullptr);
    }

    for (int i = 0; i < 20; ++i) {
        const types::MappingStepCommand command =
            planner.nextStep(stateAt(map, {5, 5, 5}, static_cast<std::size_t>(6 + i)), nullptr);
        if (!command.movement.has_value()) {
            break;
        }
        const types::MovementCommand& move = *command.movement;
        if (move.type == types::MovementCommandType::Advance) {
            EXPECT_LE(move.distance.force_numerical_value_in(cm), 2.0 + 1e-9);
        } else if (move.type == types::MovementCommandType::Rotate) {
            EXPECT_LE(move.angle.force_numerical_value_in(deg), 15.0 + 1e-9);
        } else if (move.type == types::MovementCommandType::Elevate) {
            EXPECT_LE(std::abs(move.distance.force_numerical_value_in(cm)), 2.0 + 1e-9);
        }
    }
}

/**
 * @brief A region with no frontier left terminates as `Finished`.
 * @note Every cell is `Empty` and none borders `Unmapped` space, so the search finds no target at
 *       all. This is the clean ending, and the one the next test is deliberately distinguished from.
 */
TEST(MappingAlgorithm, AFullyObservedRegionFinishesCleanly) {
    HandBuiltMap map{3, 5.0};
    for (std::int64_t i = 0; i < 3; ++i) {
        for (std::int64_t j = 0; j < 3; ++j) {
            for (std::int64_t k = 0; k < 3; ++k) {
                map.put({i, j, k}, types::VoxelOccupancy::Empty);
            }
        }
    }

    MappingAlgorithmImpl planner = makePlanner(map, permissiveDrone(5.0));
    for (int i = 0; i < 6; ++i) {
        (void)planner.nextStep(stateAt(map, {1, 1, 1}, static_cast<std::size_t>(i)), nullptr);
    }

    const types::MappingStepCommand command = planner.nextStep(stateAt(map, {1, 1, 1}, 6), nullptr);
    EXPECT_EQ(command.status, types::AlgorithmStatus::Finished)
        << "nothing borders unmapped space, so there is no frontier left";
}

/**
 * @brief A sealed pocket terminates as `FinishedWithUnmappableVoxels`, not `Finished`.
 * @note The distinction is real information for the report: a region sealed behind `Occupied` cells
 *       was never observable at all, which is a different outcome from having covered everything.
 *       Collapsing the two statuses would make a walled-off room indistinguishable from success.
 */
TEST(MappingAlgorithm, AnUnreachablePocketIsReportedAsUnmappable) {
    HandBuiltMap map{4, 5.0};
    for (std::int64_t i = 0; i < 4; ++i) {
        for (std::int64_t j = 0; j < 4; ++j) {
            for (std::int64_t k = 0; k < 4; ++k) {
                map.put({i, j, k}, types::VoxelOccupancy::Occupied);
            }
        }
    }
    map.put({1, 1, 1}, types::VoxelOccupancy::Empty);
    /**
     * @note One `Unmapped` cell walled in by `Occupied` on every side. The drone can neither reach it
     *       nor see into it, which is a different outcome from having covered everything.
     */
    map.put({3, 3, 3}, types::VoxelOccupancy::Unmapped);

    MappingAlgorithmImpl planner = makePlanner(map, permissiveDrone(5.0));
    for (int i = 0; i < 6; ++i) {
        (void)planner.nextStep(stateAt(map, {1, 1, 1}, static_cast<std::size_t>(i)), nullptr);
    }

    const types::MappingStepCommand command = planner.nextStep(stateAt(map, {1, 1, 1}, 6), nullptr);
    EXPECT_EQ(command.status, types::AlgorithmStatus::FinishedWithUnmappableVoxels);
}

/**
 * @brief Once exploration ends the planner stays finished and stops searching.
 * @note The latch is what stops a completed run re-searching the whole grid on every remaining step
 *       of the mission's budget. On the large scenarios that would be a ~216,000-cell search per
 *       step, for however many thousand steps `max_steps` still allows.
 */
TEST(MappingAlgorithm, FinishingLatchesSoLaterCallsAreCheap) {
    HandBuiltMap map{3, 5.0};
    for (std::int64_t i = 0; i < 3; ++i) {
        for (std::int64_t j = 0; j < 3; ++j) {
            for (std::int64_t k = 0; k < 3; ++k) {
                map.put({i, j, k}, types::VoxelOccupancy::Empty);
            }
        }
    }

    MappingAlgorithmImpl planner = makePlanner(map, permissiveDrone(5.0));
    for (int i = 0; i < 7; ++i) {
        (void)planner.nextStep(stateAt(map, {1, 1, 1}, static_cast<std::size_t>(i)), nullptr);
    }

    for (int i = 0; i < 3; ++i) {
        const types::MappingStepCommand command =
            planner.nextStep(stateAt(map, {1, 1, 1}, static_cast<std::size_t>(10 + i)), nullptr);
        EXPECT_EQ(command.status, types::AlgorithmStatus::Finished);
        EXPECT_FALSE(command.movement.has_value());
    }
}

/**
 * @brief A drone outside the mapped region terminates instead of guessing a move.
 * @note There is no cell to plan from, so any movement would be invented rather than derived from
 *       the map - the one path by which a bad configuration could turn into a collision. Stopping
 *       also covers a degenerate grid, which fails the same lookup.
 */
TEST(MappingAlgorithm, ADroneOutsideTheRegionStopsRatherThanGuessing) {
    HandBuiltMap map{5, 5.0};

    MappingAlgorithmImpl planner = makePlanner(map, permissiveDrone(5.0));

    types::DroneState outside{};
    outside.position = Position3D{500.0 * x_extent[cm], 500.0 * y_extent[cm], 500.0 * z_extent[cm]};
    outside.heading = Orientation{0.0 * horizontal_angle[deg], 0.0 * altitude_angle[deg]};

    const types::MappingStepCommand command = planner.nextStep(outside, nullptr);
    EXPECT_FALSE(command.movement.has_value())
        << "guessing a move from an unknown position is the one thing that could cause a collision";
    EXPECT_EQ(command.status, types::AlgorithmStatus::Finished);
}

/**
 * @brief Axis counts round up, so a partial trailing voxel is still a voxel.
 * @note `ceil` rather than truncation: 10 cm at 3 cm resolution needs four cells, and truncating to
 *       three would leave the final centimetre outside the grid - unmappable by construction rather
 *       than by obstruction. Non-positive spans give 0 instead of a negative count.
 */
TEST(VoxelGridTest, AxisCountKeepsAPartialTrailingVoxel) {
    EXPECT_EQ(VoxelGrid::axisCount(10.0, 3.0), 4) << "ceil(10/3), not 3";
    EXPECT_EQ(VoxelGrid::axisCount(9.0, 3.0), 3);
    EXPECT_EQ(VoxelGrid::axisCount(0.0, 3.0), 0);
    EXPECT_EQ(VoxelGrid::axisCount(-5.0, 3.0), 0);
}

/**
 * @brief A cell's centre maps back to the cell it came from.
 * @note The planner converts both ways on every step - index to world to emit a move, world to index
 *       to read the map - so a round trip that landed one cell over would have it plan for one voxel
 *       while reading the occupancy of another.
 */
TEST(VoxelGridTest, CentreRoundTripsThroughIndexOf) {
    const HandBuiltMap map{7, 5.0};
    const VoxelGrid& grid = map.grid();

    for (std::int64_t i = 0; i < grid.sizeX(); ++i) {
        for (std::int64_t j = 0; j < grid.sizeY(); ++j) {
            const VoxelIndex index{i, j, 3};
            const std::optional<VoxelIndex> back = grid.indexOf(grid.centreOf(index));
            ASSERT_TRUE(back.has_value());
            EXPECT_TRUE(*back == index);
        }
    }
}

/**
 * @brief Linear and 3-D indices convert to each other without loss, across the whole grid.
 * @note The frontier search works in 3-D indices while its scratch buffers are flat arrays keyed by
 *       linear index, so every visit stamp and parent link depends on this pairing being exact. A
 *       mismatch would corrupt the reconstructed route rather than fail loudly.
 */
TEST(VoxelGridTest, LinearIndexRoundTrips) {
    const HandBuiltMap map{6, 5.0};
    const VoxelGrid& grid = map.grid();

    for (std::size_t linear = 0; linear < grid.cellCount(); ++linear) {
        EXPECT_EQ(grid.linearIndex(grid.indexFromLinear(linear)), linear);
    }
}

/**
 * @brief A grid with no usable resolution reports itself unusable and maps nothing.
 * @note A zero resolution gives no cells to divide the region into. Reporting that up front is what
 *       lets the planner bail out via `indexOf` rather than dividing by it - the degenerate-geometry
 *       half of `ADroneOutsideTheRegionStopsRatherThanGuessing`.
 */
TEST(VoxelGridTest, AnUnusableGridRejectsEverything) {
    types::MapConfig config{};
    config.boundaries.max_x = 10.0 * x_extent[cm];
    config.resolution = 0.0 * cm;

    const VoxelGrid grid = VoxelGrid::from(config);
    EXPECT_FALSE(grid.usable());
    EXPECT_EQ(grid.cellCount(), 0u);
    EXPECT_FALSE(grid.indexOf(Position3D{}).has_value());
}

} // namespace
