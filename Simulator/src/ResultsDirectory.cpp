/**
 * @file ResultsDirectory.cpp
 * @brief Timestamped, collision-free creation of a run's output folder.
 */

#include <Simulator/ResultsDirectory.h>

#include <ctime>
#include <system_error>

namespace simulator {
namespace {

/**
 * @brief How many suffixed names to try before giving up.
 * @note A bound rather than an unbounded loop: if something is systematically preventing creation,
 *       failing with a message beats spinning forever inside a program that must always finish.
 */
constexpr int kMaxAttempts = 1000;

/**
 * @brief The current UTC time as a compact stamp.
 * @return The time formatted `YYYYMMDD_HHMMSS`, or an empty string if formatting failed.
 * @note UTC rather than local time, to match the `generated_at_utc` field the reports carry, so a
 *       directory name and the report inside it never appear to disagree about when a run happened.
 * @note `gmtime_r` is POSIX. That is fine for this project, which targets the course's Linux
 *       container; a portable build would need `gmtime_s` on Windows.
 * @note Phase 07's report writers need an ISO-8601 variant of this. That is the point at which the
 *       two should be factored into a shared time helper rather than duplicated.
 */
[[nodiscard]] std::string utcStamp() {
    const std::time_t now = std::time(nullptr);
    std::tm utc{};
    if (::gmtime_r(&now, &utc) == nullptr) {
        return {};
    }

    char buffer[32] = {};
    const std::size_t written = std::strftime(buffer, sizeof(buffer), "%Y%m%d_%H%M%S", &utc);
    return written == 0 ? std::string{} : std::string(buffer, written);
}

/**
 * @brief The directory-name prefix for a run mode.
 * @param mode The mode that was requested.
 * @return "comparative_results_" or "competition_", exactly as the assignment names them.
 * @note The two prefixes are deliberately not symmetric - one carries "_results" and the other does
 *       not. That asymmetry is the assignment's, not a slip, so it must be preserved verbatim.
 */
[[nodiscard]] const char* directoryPrefix(RunMode mode) {
    return mode == RunMode::Comparative ? "comparative_results_" : "competition_";
}

/**
 * @brief Build the candidate name for a given attempt.
 * @param prefix Mode-specific name prefix.
 * @param stamp UTC time stamp.
 * @param attempt Zero-based attempt number.
 * @return The bare directory name; the first attempt carries no suffix.
 * @note Suffixing starts at `_2` so the common case reads as a plain timestamp and only a genuine
 *       collision produces a number.
 */
[[nodiscard]] std::string candidateName(const char* prefix, const std::string& stamp, int attempt) {
    std::string name = prefix + stamp;
    if (attempt > 0) {
        name += "_" + std::to_string(attempt + 1);
    }
    return name;
}

} // namespace

/**
 * @brief Create the results directory for one run.
 * @param args An accepted command line; its paths must already have been validated.
 * @return The created directory, or the reason it could not be created.
 * @note `create_directory` rather than `create_directories` is the whole collision test: the plural
 *       form succeeds silently when the path already exists, while the singular form returns false,
 *       which is how a taken name is detected and retried. The parent folder is known to exist
 *       because `validateCommandLinePaths` already required it.
 * @note A false return with an error code set is a real failure - permissions, a read-only mount -
 *       and is reported immediately rather than retried, since suffixing would not help.
 */
ResultsDirectory createResultsDirectory(const CommandLineArgs& args) {
    ResultsDirectory result{};

    const std::string stamp = utcStamp();
    if (stamp.empty()) {
        result.error = "could not format a timestamp for the results directory name";
        return result;
    }

    const char* const prefix = directoryPrefix(args.mode);

    for (int attempt = 0; attempt < kMaxAttempts; ++attempt) {
        const std::filesystem::path candidate =
            args.varied_plugin_folder / candidateName(prefix, stamp, attempt);

        std::error_code ec;
        if (std::filesystem::create_directory(candidate, ec)) {
            result.path = candidate;
            return result;
        }
        if (ec) {
            result.error = "could not create results directory " + candidate.string() + ": " +
                           ec.message();
            return result;
        }
    }

    result.error = "could not find an unused results directory name under " +
                   args.varied_plugin_folder.string() + " after " + std::to_string(kMaxAttempts) +
                   " attempts";
    return result;
}

} // namespace simulator
