#include <drone_mapper/ErrorLogger.h>

#include <iostream>
#include <system_error>
#include <utility>

namespace drone_mapper {

ErrorLogger::ErrorLogger(std::filesystem::path errors_log_file,
                         std::filesystem::path input_errors_file)
    : input_errors_path_(std::move(input_errors_file)) {
    // Best-effort error-log sink: create the parent directory, then open in append mode. Any failure
    // degrades to stderr-only (file_ stays empty) rather than throwing — the logger must never be
    // the thing that crashes the program. input_errors.txt is intentionally NOT opened here; it is
    // created lazily in logInputError so it appears only when the input actually had errors.
    std::error_code ec;
    if (errors_log_file.has_parent_path()) {
        std::filesystem::create_directories(errors_log_file.parent_path(), ec);
    }
    std::ofstream stream(errors_log_file, std::ios::app);
    if (stream) {
        file_ = std::move(stream);
    }
}

void ErrorLogger::log(std::string_view code, std::string_view message) {
    // Flush each line immediately (std::endl) so errors are never lost to buffering if a later stage
    // aborts — the "log when it occurs, never deferred" contract.
    std::cerr << "[" << code << "] " << message << std::endl;
    if (file_) {
        *file_ << "[" << code << "] " << message << std::endl;
    }
}

void ErrorLogger::logInputError(std::string_view code, std::string_view message) {
    // Every input error is still an error: it goes to the error log (and stderr) immediately.
    log(code, message);
    if (input_errors_path_.empty()) {
        return;
    }
    // Lazily create input_errors.txt on the first recovered input error, so the file exists only when
    // the input had errors (per the assignment). Best-effort: a failure to open degrades silently.
    if (!input_file_) {
        std::error_code ec;
        if (input_errors_path_.has_parent_path()) {
            std::filesystem::create_directories(input_errors_path_.parent_path(), ec);
        }
        std::ofstream stream(input_errors_path_, std::ios::app);
        if (stream) {
            input_file_ = std::move(stream);
        }
    }
    if (input_file_) {
        *input_file_ << "[" << code << "] " << message << std::endl;
    }
}

} // namespace drone_mapper