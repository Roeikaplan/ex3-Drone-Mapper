/**
 * @file ErrorLoggerTest.cpp
 * @brief Coverage of the error sink's formatting, counting, degradation, and thread safety.
 * @note The concurrency case is the one that earns its keep: everything else here would pass just as
 *       happily without a mutex.
 */

#include <Simulator/ErrorLogger.h>

#include <gtest/gtest.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <regex>
#include <string>
#include <thread>
#include <vector>

namespace {

namespace fs = std::filesystem;

/**
 * @brief Gives each test its own scratch directory and removes it afterwards.
 * @note A leaked directory would eventually be enumerated by a plugin-folder scan in an end-to-end
 *       run, so cleanup is not merely tidiness.
 */
class ErrorLoggerTest : public ::testing::Test {
protected:
    /**
     * @brief Create a uniquely named scratch directory under the system temp folder.
     */
    void SetUp() override {
        const ::testing::TestInfo* const info =
            ::testing::UnitTest::GetInstance()->current_test_info();
        dir_ = fs::temp_directory_path() /
               ("ex3_errorlogger_" + std::string{info->name()} + "_" +
                std::to_string(::getpid()));
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
     * @brief Read a file into a list of lines.
     * @param file File to read.
     * @return One entry per line; empty when the file does not exist.
     */
    [[nodiscard]] static std::vector<std::string> readLines(const fs::path& file) {
        std::vector<std::string> lines;
        std::ifstream stream(file);
        std::string line;
        while (std::getline(stream, line)) {
            lines.push_back(line);
        }
        return lines;
    }

    fs::path dir_{};
};

TEST_F(ErrorLoggerTest, DefaultConstructedWritesNoFile) {
    simulator::ErrorLogger logger;
    logger.log("CODE", "message");

    EXPECT_TRUE(logger.file().empty());
    EXPECT_EQ(logger.errorCount(), 1u);
}

TEST_F(ErrorLoggerTest, WritesCodeAndMessage) {
    const fs::path file = dir_ / "errors.log";
    {
        simulator::ErrorLogger logger{file};
        logger.log("PLUGIN_LOAD_FAILED", "libfoo.so: undefined symbol");
    }

    const std::vector<std::string> lines = readLines(file);
    ASSERT_EQ(lines.size(), 1u);
    EXPECT_EQ(lines.front(), "[PLUGIN_LOAD_FAILED] libfoo.so: undefined symbol");
}

TEST_F(ErrorLoggerTest, AppendsAcrossCalls) {
    const fs::path file = dir_ / "errors.log";
    {
        simulator::ErrorLogger logger{file};
        logger.log("A", "first");
        logger.log("B", "second");
        logger.logInputError("C", "third");
    }

    const std::vector<std::string> lines = readLines(file);
    ASSERT_EQ(lines.size(), 3u);
    EXPECT_EQ(lines[0], "[A] first");
    EXPECT_EQ(lines[2], "[C] third");
}

TEST_F(ErrorLoggerTest, CreatesMissingParentDirectories) {
    const fs::path file = dir_ / "nested" / "deeper" / "errors.log";
    {
        simulator::ErrorLogger logger{file};
        logger.log("CODE", "message");
    }

    EXPECT_TRUE(fs::exists(file));
}

TEST_F(ErrorLoggerTest, CountsTotalAndInputErrorsSeparately) {
    simulator::ErrorLogger logger{dir_ / "errors.log"};
    logger.log("A", "one");
    logger.logInputError("B", "two");
    logger.logInputError("C", "three");

    EXPECT_EQ(logger.errorCount(), 3u) << "input errors are errors too";
    EXPECT_EQ(logger.inputErrorCount(), 2u);
}

TEST_F(ErrorLoggerTest, UnopenablePathDegradesInsteadOfThrowing) {
    /**
     * @note The existing *directory* is handed over as if it were a log file, so the open must fail.
     *       The logger has to keep working as a stderr-only sink rather than throwing from its
     *       constructor at the exact moment something needs reporting.
     */
    EXPECT_NO_THROW({
        simulator::ErrorLogger logger{dir_};
        logger.log("CODE", "message");
        EXPECT_EQ(logger.errorCount(), 1u);
    });
}

TEST_F(ErrorLoggerTest, ConcurrentWritesProduceWholeLines) {
    constexpr int kThreads = 8;
    constexpr int kPerThread = 200;
    const fs::path file = dir_ / "errors.log";

    {
        simulator::ErrorLogger logger{file};

        std::vector<std::thread> workers;
        workers.reserve(kThreads);
        for (int t = 0; t < kThreads; ++t) {
            workers.emplace_back([&logger, t] {
                for (int i = 0; i < kPerThread; ++i) {
                    logger.log("THREAD" + std::to_string(t),
                               "message " + std::to_string(i) + " padded to a useful length so a "
                                                                "torn write would be visible");
                }
            });
        }
        for (std::thread& worker : workers) {
            worker.join();
        }

        EXPECT_EQ(logger.errorCount(), static_cast<std::size_t>(kThreads * kPerThread));
    }

    const std::vector<std::string> lines = readLines(file);
    ASSERT_EQ(lines.size(), static_cast<std::size_t>(kThreads * kPerThread))
        << "a torn or interleaved write would change the line count";

    /**
     * @note Counting lines alone is not enough: two half-lines still add up to two lines. Every line
     *       must match the full expected shape for the lock to be doing its job.
     */
    const std::regex expected{R"(^\[THREAD[0-7]\] message \d+ padded to a useful length so a torn write would be visible$)"};
    for (const std::string& line : lines) {
        EXPECT_TRUE(std::regex_match(line, expected)) << "malformed line: " << line;
    }
}

} // namespace
