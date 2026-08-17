#pragma once

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

// Shared fixture for the whole-flow integration suite. Every integration test must be a
// TEST_F(Integration, ...) — gtest forbids mixing TEST and TEST_F within one suite, and the suite
// name must stay exactly `Integration` so the graded --gtest_filter=Integration.* keeps matching.

#ifndef DRONE_MAPPER_SOURCE_DIR
#define DRONE_MAPPER_SOURCE_DIR "."
#endif

namespace drone_mapper_test {

/**
 * @brief Base fixture for integration tests: repository/data paths and small file helpers.
 *
 * Holds the helpers every whole-flow test needs (locating repo data via the CMake-injected source
 * dir, writing YAML fixtures, reading outputs back), so individual test files do not re-declare
 * them.
 *
 * @note Named `Integration` (via the alias below) because a gtest fixture's class name IS the suite
 *       name; the assignment mandates the `Integration.*` filter.
 */
class IntegrationFixture : public ::testing::Test {
public:
    /// Repository source root, injected by CMake so data maps are found regardless of the CWD.
    [[nodiscard]] static std::filesystem::path sourceDir() {
        return std::filesystem::path{DRONE_MAPPER_SOURCE_DIR};
    }

    /**
     * @brief Write text to a file, creating parent directories as needed.
     * @param path Destination file.
     * @param content File body.
     */
    static void writeFile(const std::filesystem::path& path, const std::string& content) {
        std::filesystem::create_directories(path.parent_path());
        std::ofstream out(path, std::ios::trunc);
        out << content;
    }

    /**
     * @brief Read a whole text file into a string.
     * @param path File to read.
     * @return Its full contents (empty if absent).
     */
    [[nodiscard]] static std::string readAll(const std::filesystem::path& path) {
        std::ifstream in(path);
        std::ostringstream os;
        os << in.rdbuf();
        return os.str();
    }

    /**
     * @brief A fresh (removed if pre-existing) directory under the gtest temp dir.
     * @param name Directory name distinguishing the test.
     * @return The (now non-existent) path, ready for a run's output.
     */
    [[nodiscard]] static std::filesystem::path freshOutDir(const std::string& name) {
        const std::filesystem::path dir = std::filesystem::path{::testing::TempDir()} / name;
        std::filesystem::remove_all(dir);
        return dir;
    }
};

} // namespace drone_mapper_test

/// The fixture under the mandated suite name (class name == gtest suite name).
using Integration = drone_mapper_test::IntegrationFixture;
