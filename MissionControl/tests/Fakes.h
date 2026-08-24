/**
 * @file Fakes.h
 * @brief Hand-written stand-ins for the interfaces the mission control is given.
 * @note The MissionControl project links only `common::common`, so nothing Simulator-side - not
 *       `Map3DImpl`, not the mocks - is reachable from here. These fakes exist because of that
 *       boundary, not in spite of it: a plugin that could reach into the host's implementation would
 *       not be a plugin.
 */

#pragma once

#include <Common/IDroneMovement.h>
#include <Common/IGPS.h>
#include <Common/ILidar.h>
#include <Common/IMappingAlgorithm.h>
#include <Common/IMutableMap3D.h>

#include <cmath>
#include <cstddef>
#include <filesystem>
#include <map>
#include <vector>

namespace mission_control::testing {

using namespace common;

/**
 * @brief A sparse map over a fixed cubic region.
 *
 * @note Sparse rather than dense so a test can declare a 200 cm world without allocating it. Only
 *       cells that were written are stored; everything else reads `Unmapped`, exactly as a real
 *       output map behaves before it is scanned.
 */
class FakeMap final : public IMutableMap3D {
public:
    /**
     * @brief Construct a cubic map anchored at the origin.
     * @param span_cm Extent of each axis in centimetres.
     * @param resolution_cm Voxel edge length in centimetres.
     */
    FakeMap(double span_cm, double resolution_cm) {
        config_.resolution = resolution_cm * cm;
        config_.boundaries.max_x = span_cm * x_extent[cm];
        config_.boundaries.max_y = span_cm * y_extent[cm];
        config_.boundaries.max_height = span_cm * z_extent[cm];
        span_cm_ = span_cm;
        resolution_cm_ = resolution_cm;
    }

    /**
     * @brief Occupancy of the cell containing a position.
     * @param pos World position.
     * @return The stored value, `Unmapped` if never written, `OutOfBounds` outside the region.
     */
    [[nodiscard]] types::VoxelOccupancy atVoxel(const Position3D& pos) const override {
        if (!isInBounds(pos)) {
            return types::VoxelOccupancy::OutOfBounds;
        }
        const auto found = cells_.find(key(pos));
        return found == cells_.end() ? types::VoxelOccupancy::Unmapped : found->second;
    }

    /**
     * @brief This map's geometry.
     * @return The configured bounds, offset, and resolution.
     */
    [[nodiscard]] types::MapConfig getMapConfig() const override { return config_; }

    /**
     * @brief Whether a position lies inside the region.
     * @param pos World position.
     * @return True when every axis is within `[0, span)`.
     */
    [[nodiscard]] bool isInBounds(const Position3D& pos) const override {
        const double x = pos.x.force_numerical_value_in(cm);
        const double y = pos.y.force_numerical_value_in(cm);
        const double z = pos.z.force_numerical_value_in(cm);
        return x >= 0.0 && x < span_cm_ && y >= 0.0 && y < span_cm_ && z >= 0.0 && z < span_cm_;
    }

    /**
     * @brief Store a value at a position.
     * @param pos World position.
     * @param value Occupancy to store.
     * @note Out-of-bounds writes are dropped, matching the real map.
     */
    void set(const Position3D& pos, types::VoxelOccupancy value) override {
        if (isInBounds(pos)) {
            cells_[key(pos)] = value;
        }
    }

    /**
     * @brief Record that a save was requested.
     * @param path Where the map would have been written.
     * @note Nothing is written to disk. What the tests care about is *how many times* this is
     *       called - saving per step rather than once is the mistake worth catching.
     */
    void save(const std::filesystem::path& path) const override {
        ++saves_;
        last_save_path_ = path;
    }

    /**
     * @brief How many times `save` was called.
     * @return The count.
     */
    [[nodiscard]] std::size_t saveCount() const noexcept { return saves_; }

    /**
     * @brief The path of the most recent save request.
     * @return That path, or empty when none was made.
     */
    [[nodiscard]] const std::filesystem::path& lastSavePath() const noexcept {
        return last_save_path_;
    }

    /**
     * @brief How many cells hold a given value.
     * @param value Occupancy to count.
     * @return The number of written cells matching it.
     */
    [[nodiscard]] std::size_t countOf(types::VoxelOccupancy value) const {
        std::size_t total = 0;
        for (const auto& entry : cells_) {
            if (entry.second == value) {
                ++total;
            }
        }
        return total;
    }

private:
    /**
     * @brief Integer cell coordinate for a position.
     * @param pos World position.
     * @return The `{i, j, k}` index of the cell containing it.
     */
    [[nodiscard]] std::array<long, 3> key(const Position3D& pos) const {
        return {static_cast<long>(std::floor(pos.x.force_numerical_value_in(cm) / resolution_cm_)),
                static_cast<long>(std::floor(pos.y.force_numerical_value_in(cm) / resolution_cm_)),
                static_cast<long>(std::floor(pos.z.force_numerical_value_in(cm) / resolution_cm_))};
    }

    types::MapConfig config_{};
    double span_cm_ = 0.0;
    double resolution_cm_ = 1.0;
    std::map<std::array<long, 3>, types::VoxelOccupancy> cells_{};
    mutable std::size_t saves_ = 0;
    mutable std::filesystem::path last_save_path_{};
};

/**
 * @brief A pose that the fake actuator writes into.
 */
class FakeGPS final : public IGPS {
public:
    /**
     * @brief Construct at a starting pose.
     * @param position Where the drone begins.
     * @param heading Which way it faces.
     */
    FakeGPS(Position3D position, Orientation heading)
        : position_(position), heading_(heading) {}

    /**
     * @brief The drone's position.
     * @return The stored position.
     */
    [[nodiscard]] Position3D position() const override { return position_; }

    /**
     * @brief The drone's orientation.
     * @return The stored heading.
     */
    [[nodiscard]] Orientation heading() const override { return heading_; }

    /**
     * @brief Overwrite the position.
     * @param position New position.
     */
    void setPosition(Position3D position) { position_ = position; }

    /**
     * @brief Overwrite the heading.
     * @param heading New orientation.
     */
    void setHeading(Orientation heading) { heading_ = heading; }

private:
    Position3D position_{};
    Orientation heading_{};
};

/**
 * @brief An actuator that moves the fake GPS and counts what it was asked to do.
 * @note Mirrors the simulator's own actuator: it validates nothing and always succeeds, so a test
 *       asserting that an illegal move was *refused* is asserting about the drone controller rather
 *       than about this.
 */
class FakeMovement final : public IDroneMovement {
public:
    /**
     * @brief Construct over the pose to update.
     * @param gps The pose this actuator writes into.
     */
    explicit FakeMovement(FakeGPS& gps) : gps_(gps) {}

    /**
     * @brief Turn in place.
     * @param direction Left or right.
     * @param angle How far to turn.
     * @return Always success.
     */
    types::MovementResult rotate(types::RotationDirection direction,
                                 HorizontalAngle angle) override {
        ++calls_;
        const Orientation current = gps_.heading();
        const HorizontalAngle signed_angle =
            direction == types::RotationDirection::Left ? angle : -angle;
        gps_.setHeading(Orientation{current.horizontal + signed_angle, current.altitude});
        return types::MovementResult{true, {}};
    }

    /**
     * @brief Travel along the current heading.
     * @param distance How far to travel.
     * @return Always success.
     */
    types::MovementResult advance(PhysicalLength distance) override {
        ++calls_;
        const Orientation heading = gps_.heading();
        gps_.setPosition(user_common_advance(gps_.position(), heading, distance));
        return types::MovementResult{true, {}};
    }

    /**
     * @brief Change altitude.
     * @param distance How far to climb; negative descends.
     * @return Always success.
     */
    types::MovementResult elevate(PhysicalLength distance) override {
        ++calls_;
        const Position3D position = gps_.position();
        gps_.setPosition(Position3D{
            position.x, position.y,
            position.z + distance.force_numerical_value_in(cm) * z_extent[cm]});
        return types::MovementResult{true, {}};
    }

    /**
     * @brief How many movement commands were executed.
     * @return The count.
     * @note A test that expects a move to be refused asserts this stayed at zero: the drone
     *       controller must reject *before* commanding, not after.
     */
    [[nodiscard]] std::size_t callCount() const noexcept { return calls_; }

private:
    /**
     * @brief Horizontal travel along a heading.
     * @param from Starting position.
     * @param heading Current orientation.
     * @param distance How far to travel.
     * @return The resulting position.
     * @note Declared here rather than calling the shared helper so this fake stays a plain,
     *       independent reference implementation - a bug in the shared geometry should make the
     *       tests disagree with it, not agree silently.
     */
    [[nodiscard]] static Position3D user_common_advance(const Position3D& from,
                                                        const Orientation& heading,
                                                        PhysicalLength distance) {
        const double radians = heading.horizontal.force_numerical_value_in(deg) * M_PI / 180.0;
        const double travel = distance.force_numerical_value_in(cm);
        return Position3D{from.x + std::cos(radians) * travel * x_extent[cm],
                          from.y + std::sin(radians) * travel * y_extent[cm], from.z};
    }

    FakeGPS& gps_;
    std::size_t calls_ = 0;
};

/**
 * @brief A lidar that returns a scan the test supplies.
 */
class FakeLidar final : public ILidar {
public:
    /**
     * @brief Construct with a configuration and a canned result.
     * @param config The geometry to report.
     * @param result What every scan returns.
     */
    FakeLidar(types::LidarConfigData config, types::LidarScanResult result)
        : config_(config), result_(std::move(result)) {}

    /**
     * @brief Take a scan.
     * @return The canned result, regardless of direction.
     */
    [[nodiscard]] types::LidarScanResult scan(Orientation) const override {
        ++scans_;
        return result_;
    }

    /**
     * @brief This sensor's configuration.
     * @return The configured geometry.
     */
    [[nodiscard]] types::LidarConfigData config() const override { return config_; }

    /**
     * @brief How many scans were taken.
     * @return The count.
     */
    [[nodiscard]] std::size_t scanCount() const noexcept { return scans_; }

private:
    types::LidarConfigData config_{};
    types::LidarScanResult result_{};
    mutable std::size_t scans_ = 0;
};

/**
 * @brief An algorithm that replays a scripted list of commands.
 *
 * @note This is what makes the mission loop testable: the drone's whole behaviour becomes an input
 *       to the test rather than something to be coaxed out of a real planner.
 * @note It also records whether the *first* call received a null scan, which is the one ordering
 *       rule that cannot be observed any other way.
 */
class ScriptedAlgorithm final : public IMappingAlgorithm {
public:
    /**
     * @brief Construct with the commands to issue, in order.
     * @param dependencies What the host supplies.
     * @param script Commands to return, one per step; the last repeats once exhausted.
     */
    ScriptedAlgorithm(MappingAlgorithmDependencies dependencies,
                      std::vector<types::MappingStepCommand> script)
        : IMappingAlgorithm(std::move(dependencies)), script_(std::move(script)) {}

    /**
     * @brief Return the next scripted command.
     * @param state The drone's current state.
     * @param latest_scan The previous scan, or null on the first call.
     * @return The scripted command for this step.
     */
    [[nodiscard]] types::MappingStepCommand nextStep(
        const types::DroneState& state, const types::LidarScanResult* latest_scan) override {
        if (calls_ == 0) {
            first_scan_was_null_ = latest_scan == nullptr;
        }
        observed_states_.push_back(state);
        const types::MappingStepCommand command =
            script_.empty() ? types::MappingStepCommand{}
                            : script_[std::min(calls_, script_.size() - 1)];
        ++calls_;
        return command;
    }

    /**
     * @brief Whether the first call received a null scan pointer.
     * @return True when it did.
     */
    [[nodiscard]] bool firstScanWasNull() const noexcept { return first_scan_was_null_; }

    /**
     * @brief How many times the algorithm was consulted.
     * @return The count.
     */
    [[nodiscard]] std::size_t callCount() const noexcept { return calls_; }

    /**
     * @brief The drone states the algorithm was shown, in order.
     * @return One entry per call.
     */
    [[nodiscard]] const std::vector<types::DroneState>& observedStates() const noexcept {
        return observed_states_;
    }

private:
    std::vector<types::MappingStepCommand> script_{};
    std::vector<types::DroneState> observed_states_{};
    std::size_t calls_ = 0;
    bool first_scan_was_null_ = false;
};

} // namespace mission_control::testing
