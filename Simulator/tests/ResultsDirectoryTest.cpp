/**
 * @file ResultsDirectoryTest.cpp
 * @brief Coverage of results-directory naming, placement, and collision avoidance.
 * @note The back-to-back creation case is the important one: it is the only test that would fail if
 *       the name were a bare second-resolution timestamp, which the assignment forbids.
 */

#include <Simulator/ResultsDirectory.h>

#include <gtest/gtest.h>

#include <cstdlib>
#include <filesystem>
#include <string>

namespace {

namespace fs = std::filesystem;

/**
 * @brief Gives each test its own scratch directory to stand in for a plugin folder.
 */
class ResultsDirectoryTest : public ::testing::Test {
protected:
    /**
     * @brief Create a uniquely named scratch directory under the system temp folder.
     */
    void SetUp() override {
        const ::testing::TestInfo* const info =
            ::testing::UnitTest::GetInstance()->current_test_info();
        dir_ = fs::temp_directory_path() /
               ("ex3_results_" + std::string{info->name()} + "_" + std::to_string(::getpid()));
        std::error_code ec;
        fs::remove_all(dir_, ec);
        fs::create_directories(dir_, ec);
    }

    /**
     * @brief Remove the scratch directory and everything in it.
     */
    void TearDown() override {
        std::error_code ec;
        fs::remove_all(dir_, ec);
    }

    /**
     * @brief Build an accepted command line pointing at the scratch directory.
     * @param mode The run mode to request.
     * @return Arguments with `varied_plugin_folder` set and nothing else that matters here.
     */
    [[nodiscard]] simulator::CommandLineArgs argsFor(simulator::RunMode mode) const {
        simulator::CommandLineArgs args{};
        args.mode = mode;
        args.varied_plugin_folder = dir_;
        return args;
    }

    fs::path dir_{};
};

/**
 * @brief Comparative mode creates its results directory under the varied plugin folder.
 * @note Both halves are checked - the parent and the prefix - because the assignment fixes where the
 *       directory goes as firmly as what it is called.
 */
TEST_F(ResultsDirectoryTest, ComparativeUsesItsOwnPrefix) {
    const simulator::ResultsDirectory result =
        simulator::createResultsDirectory(argsFor(simulator::RunMode::Comparative));

    ASSERT_TRUE(result.ok()) << result.error;
    EXPECT_EQ(result.path.parent_path(), dir_);
    EXPECT_EQ(result.path.filename().string().rfind("comparative_results_", 0), 0u);
    EXPECT_TRUE(fs::is_directory(result.path));
}

/**
 * @brief Competition mode uses the other prefix, which is not simply the first one renamed.
 */
TEST_F(ResultsDirectoryTest, CompetitionUsesItsOwnPrefix) {
    const simulator::ResultsDirectory result =
        simulator::createResultsDirectory(argsFor(simulator::RunMode::Competition));

    ASSERT_TRUE(result.ok()) << result.error;
    EXPECT_EQ(result.path.filename().string().rfind("competition_", 0), 0u)
        << "the assignment's two prefixes are deliberately asymmetric";
    EXPECT_TRUE(fs::is_directory(result.path));
}

/**
 * @brief The directory name carries a `YYYYMMDD_HHMMSS` stamp in a fixed shape.
 * @note The stamp is what makes a fresh run land somewhere new, so its length and separator are
 *       pinned rather than left to whatever the formatter happens to emit.
 */
TEST_F(ResultsDirectoryTest, NameCarriesATimestamp) {
    const simulator::ResultsDirectory result =
        simulator::createResultsDirectory(argsFor(simulator::RunMode::Comparative));

    ASSERT_TRUE(result.ok()) << result.error;
    const std::string stamp =
        result.path.filename().string().substr(std::string{"comparative_results_"}.size());
    ASSERT_EQ(stamp.size(), std::string{"YYYYMMDD_HHMMSS"}.size());
    EXPECT_EQ(stamp[8], '_');
}

/**
 * @brief Three runs inside one second still get three distinct directories.
 */
TEST_F(ResultsDirectoryTest, BackToBackRunsNeverCollide) {
    /**
     * @note These three calls land in the same wall-clock second, which is exactly the case a bare
     *       timestamp cannot survive and the assignment explicitly rules out.
     */
    const simulator::ResultsDirectory first =
        simulator::createResultsDirectory(argsFor(simulator::RunMode::Comparative));
    const simulator::ResultsDirectory second =
        simulator::createResultsDirectory(argsFor(simulator::RunMode::Comparative));
    const simulator::ResultsDirectory third =
        simulator::createResultsDirectory(argsFor(simulator::RunMode::Comparative));

    ASSERT_TRUE(first.ok()) << first.error;
    ASSERT_TRUE(second.ok()) << second.error;
    ASSERT_TRUE(third.ok()) << third.error;

    EXPECT_NE(first.path, second.path);
    EXPECT_NE(second.path, third.path);
    EXPECT_TRUE(fs::is_directory(first.path));
    EXPECT_TRUE(fs::is_directory(second.path));
    EXPECT_TRUE(fs::is_directory(third.path));
}

/**
 * @brief An uncreatable directory is reported through the result, never by throwing.
 * @note This runs before any mission does, so the failure is recoverable - `main` prints it and
 *       returns normally. Throwing would turn a bad path into a crash with no report at all.
 */
TEST_F(ResultsDirectoryTest, MissingParentIsReportedNotThrown) {
    simulator::CommandLineArgs args = argsFor(simulator::RunMode::Comparative);
    args.varied_plugin_folder = dir_ / "does" / "not" / "exist";

    simulator::ResultsDirectory result{};
    EXPECT_NO_THROW({ result = simulator::createResultsDirectory(args); });

    EXPECT_FALSE(result.ok());
    EXPECT_FALSE(result.error.empty());
    EXPECT_TRUE(result.path.empty());
}

} // namespace
