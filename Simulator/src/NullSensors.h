/**
 * @file NullSensors.h
 * @brief Null-object stand-ins for the simulator-facing interfaces, used by the phase-01 smoke check.
 *
 * The plugin dependency structs demand live references to a map and three sensors before a factory
 * can be invoked at all. These fill that requirement with the least possible behaviour, so the
 * smoke check exercises the plugin lifecycle and nothing else.
 *
 * @note Temporary. Deleted once `Map3DImpl`, `MockGPS`, `MockMovement`, and `MockLidar` land, at
 *       which point the real mocks fill the same slots.
 * @note Header-only and private to `Simulator/src/` on purpose: nothing outside the smoke check may
 *       depend on these, and they must never reach a plugin.
 */

#pragma once

#include <Common/IDroneMovement.h>
#include <Common/IGPS.h>
#include <Common/ILidar.h>
#include <Common/IMutableMap3D.h>

#include <filesystem>

namespace simulator {

/**
 * @brief A map that stores nothing and reports every voxel as unmapped.
 *
 * @note Implements `IMutableMap3D` rather than `IMap3D` so one instance satisfies both slots: the
 *       algorithm's read-only `const IMap3D&` and the mission control's writable `IMutableMap3D&`.
 * @note `isInBounds` returns false, so a plugin that respects bounds will decline to write - which
 *       is the desired behaviour for a smoke check that must not depend on map semantics.
 */
class NullMap3D final : public common::IMutableMap3D {
public:
    /**
     * @brief Occupancy at a world position.
     * @return Always `Unmapped`.
     */
    [[nodiscard]] common::types::VoxelOccupancy atVoxel(const common::Position3D&) const override {
        return common::types::VoxelOccupancy::Unmapped;
    }

    /**
     * @brief Geometry of this map.
     * @return A default-constructed `MapConfig` (zero bounds, zero offset, zero resolution).
     */
    [[nodiscard]] common::types::MapConfig getMapConfig() const override { return {}; }

    /**
     * @brief Whether a position lies inside the map.
     * @return Always false; the null map has no extent.
     */
    [[nodiscard]] bool isInBounds(const common::Position3D&) const override { return false; }

    /**
     * @brief Write a voxel.
     * @note Discards the write. Nothing in the smoke check reads the map back.
     */
    void set(const common::Position3D&, common::types::VoxelOccupancy) override {}

    /**
     * @brief Persist the map.
     * @note Deliberately writes no file - the smoke check must leave no artefacts on disk.
     */
    void save(const std::filesystem::path&) const override {}
};

/**
 * @brief A positioning sensor permanently reporting the origin.
 * @note Passive state with no setters, unlike the eventual `MockGPS`: nothing in the smoke check
 *       moves the drone.
 */
class NullGPS final : public common::IGPS {
public:
    /**
     * @brief Reported world position.
     * @return The origin.
     */
    [[nodiscard]] common::Position3D position() const override { return {}; }

    /**
     * @brief Reported orientation.
     * @return Zero heading and zero altitude angle (`0 deg` = +X east).
     */
    [[nodiscard]] common::Orientation heading() const override { return {}; }
};

/**
 * @brief An actuator that accepts every command and moves nothing.
 * @note Reports success so a plugin under test follows its normal path rather than an error branch.
 */
class NullMovement final : public common::IDroneMovement {
public:
    /**
     * @brief Rotate the drone.
     * @return Success.
     */
    common::types::MovementResult rotate(common::types::RotationDirection,
                                         common::HorizontalAngle) override {
        return {};
    }

    /**
     * @brief Advance along the current heading.
     * @return Success.
     */
    common::types::MovementResult advance(common::PhysicalLength) override { return {}; }

    /**
     * @brief Change altitude.
     * @return Success.
     */
    common::types::MovementResult elevate(common::PhysicalLength) override { return {}; }
};

/**
 * @brief A lidar that returns no hits.
 * @note Zero `fov_circles` in the returned config, so a plugin computing beam counts from it gets a
 *       consistent, empty picture rather than a mismatch between config and scan.
 */
class NullLidar final : public common::ILidar {
public:
    /**
     * @brief Take a scan.
     * @return An empty hit list.
     */
    [[nodiscard]] common::types::LidarScanResult scan(common::Orientation) const override {
        return {};
    }

    /**
     * @brief The sensor's configuration.
     * @return A default-constructed `LidarConfigData`.
     */
    [[nodiscard]] common::types::LidarConfigData config() const override { return {}; }
};

} // namespace simulator
