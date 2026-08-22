/**
 * @file UtcTime.cpp
 * @brief UTC formatting shared by the results directory and the reports.
 * @note Factored out of `ResultsDirectory.cpp` once the reports needed a second format. Keeping one
 *       implementation means the directory name and the report inside it are always read from the
 *       clock the same way.
 */

#include <Simulator/UtcTime.h>

#include <ctime>

namespace simulator {
namespace {

/**
 * @brief Format the current UTC time with a `strftime` pattern.
 * @param pattern A `strftime` format string.
 * @return The formatted time, or an empty string when the conversion or formatting failed.
 * @note `gmtime_r` is POSIX. That is fine for this project, which targets the course's Linux
 *       container; a portable build would need `gmtime_s` on Windows.
 * @note Failure returns empty rather than a placeholder, so a caller can decide whether a missing
 *       timestamp is fatal for it. The results directory treats it as fatal; a report does not.
 */
[[nodiscard]] std::string formatUtc(const char* pattern) {
    const std::time_t now = std::time(nullptr);
    std::tm utc{};
    if (::gmtime_r(&now, &utc) == nullptr) {
        return {};
    }

    char buffer[64] = {};
    const std::size_t written = std::strftime(buffer, sizeof(buffer), pattern, &utc);
    return written == 0 ? std::string{} : std::string(buffer, written);
}

} // namespace

/**
 * @brief The current UTC time as a compact stamp suitable for a filename.
 * @return `YYYYMMDD_HHMMSS`, or an empty string if formatting failed.
 */
std::string utcCompactStamp() {
    return formatUtc("%Y%m%d_%H%M%S");
}

/**
 * @brief The current UTC time in ISO-8601.
 * @return `YYYY-MM-DDTHH:MM:SSZ`, or an empty string if formatting failed.
 * @note The trailing `Z` is written literally rather than via `%z`, which would emit `+0000`.
 */
std::string utcIso8601() {
    return formatUtc("%Y-%m-%dT%H:%M:%SZ");
}

} // namespace simulator
