/**
 * @file ErrorLogger.cpp
 * @brief Immediate, synchronised error reporting to `std::cerr` and an optional log file.
 * @note Every write path here flushes. Buffering would be faster and would also mean a later hard
 *       failure silently discards the diagnostics explaining it.
 */

#include <Simulator/ErrorLogger.h>

#include <iostream>
#include <system_error>
#include <utility>

namespace simulator {

/**
 * @brief Construct a logger that mirrors to `std::cerr` and appends to a file.
 * @param errors_log_file Destination log; missing parent directories are created.
 * @note Opened in append mode so a results directory reused across runs accumulates rather than
 *       truncating an earlier transcript.
 * @note Every failure here is swallowed on purpose. A logger that throws from its constructor would
 *       take down the program at exactly the moment something already needs reporting; degrading to
 *       stderr-only keeps the diagnostics flowing.
 */
ErrorLogger::ErrorLogger(std::filesystem::path errors_log_file)
    : file_path_(std::move(errors_log_file)) {
    std::error_code ec;
    if (file_path_.has_parent_path()) {
        std::filesystem::create_directories(file_path_.parent_path(), ec);
    }

    std::ofstream stream(file_path_, std::ios::app);
    if (stream) {
        file_ = std::move(stream);
    }
}

/**
 * @brief Record one error immediately.
 * @param code Short machine-readable code.
 * @param message Human-readable detail.
 * @note The lock spans both sinks so a line cannot be split across threads on either one, and the
 *       terminal and file transcripts stay in the same order.
 */
void ErrorLogger::log(std::string_view code, std::string_view message) {
    const std::lock_guard<std::mutex> guard(mutex_);
    ++error_count_;
    writeLocked(code, message);
}

/**
 * @brief Record a recovered input-file error.
 * @param code Short machine-readable code.
 * @param message Human-readable detail.
 * @note Counted twice on purpose: once as an error like any other, and once as an input problem, so
 *       a caller can distinguish "the input was malformed but usable" from "the run failed".
 */
void ErrorLogger::logInputError(std::string_view code, std::string_view message) {
    const std::lock_guard<std::mutex> guard(mutex_);
    ++error_count_;
    ++input_error_count_;
    writeLocked(code, message);
}

/**
 * @brief How many errors have been recorded in total.
 * @return The count, including those recorded through `logInputError`.
 * @note Locks because the counter is written from worker threads; reading it unsynchronised would be
 *       a data race even though the value is only ever used for a summary line.
 */
std::size_t ErrorLogger::errorCount() const {
    const std::lock_guard<std::mutex> guard(mutex_);
    return error_count_;
}

/**
 * @brief How many of the recorded errors were recovered input problems.
 * @return The count of `logInputError` calls.
 */
std::size_t ErrorLogger::inputErrorCount() const {
    const std::lock_guard<std::mutex> guard(mutex_);
    return input_error_count_;
}

/**
 * @brief Write one formatted line to both sinks.
 * @param code Short machine-readable code.
 * @param message Human-readable detail.
 * @note `std::endl` rather than `'\n'` is deliberate: it flushes, which is what makes "logged
 *       immediately" true rather than merely intended.
 * @note The file sink is optional; a stderr-only logger simply skips it.
 */
void ErrorLogger::writeLocked(std::string_view code, std::string_view message) {
    std::cerr << "[" << code << "] " << message << std::endl;
    if (file_) {
        *file_ << "[" << code << "] " << message << std::endl;
    }
}

} // namespace simulator
