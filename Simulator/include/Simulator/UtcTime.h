/**
 * @file UtcTime.h
 * @brief The two UTC timestamp formats the simulator emits.
 */

#pragma once

#include <string>

namespace simulator {

/**
 * @brief The current UTC time as a compact stamp suitable for a filename.
 * @return `YYYYMMDD_HHMMSS`, or an empty string if formatting failed.
 * @note Used for the results directory name, where colons and spaces would be awkward.
 */
[[nodiscard]] std::string utcCompactStamp();

/**
 * @brief The current UTC time in ISO-8601.
 * @return `YYYY-MM-DDTHH:MM:SSZ`, or an empty string if formatting failed.
 * @note Used for the reports' `generated_at_utc` field, whose format the assignment fixes.
 * @note Both formats come from the same clock reading style deliberately: a directory name and the
 *       report inside it must never appear to disagree about when a run happened.
 */
[[nodiscard]] std::string utcIso8601();

} // namespace simulator
