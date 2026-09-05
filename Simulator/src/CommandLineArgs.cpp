/**
 * @file CommandLineArgs.cpp
 * @brief Tokenising, classifying, and validating the simulator's command line.
 * @note Errors are aggregated per category rather than emitted one per offending token, because the
 *       assignment asks for *an* error naming every unsupported argument and *an* error detailing
 *       every missing one - not a wall of one-line complaints.
 */

#include <Simulator/CommandLineArgs.h>

#include <Simulator/PluginDiscovery.h>

#include <charconv>
#include <fstream>
#include <map>
#include <string_view>
#include <system_error>

namespace simulator {
namespace {

constexpr std::string_view kComparativeFlag = "-comparative";
constexpr std::string_view kCompetitionFlag = "-competition";
constexpr std::string_view kVerboseFlag = "-verbose";

constexpr std::string_view kSimulation = "simulation";
constexpr std::string_view kMissionControlFolder = "mission_control_folder";
constexpr std::string_view kAlgorithm = "algorithm";
constexpr std::string_view kMissionControl = "mission_control";
constexpr std::string_view kAlgorithmsFolder = "algorithms_folder";
constexpr std::string_view kNumThreads = "num_threads";

/**
 * @brief The raw shape of a command line, before any mode-dependent meaning is applied.
 * @note Keys land in an ordered map so error text is reproducible run to run, which matters because
 *       the tests compare messages.
 */
struct RawArguments {
    std::map<std::string, std::string> values{};
    bool comparative = false;
    bool competition = false;
    bool verbose = false;
    std::vector<std::string> unsupported{};
    std::vector<std::string> duplicated{};
    std::vector<std::string> empty_valued{};
};

/**
 * @brief Join strings for inclusion in an aggregated error message.
 * @param items Strings to join; may be empty.
 * @return The items separated by ", ".
 */
[[nodiscard]] std::string join(const std::vector<std::string>& items) {
    std::string joined;
    for (const std::string& item : items) {
        if (!joined.empty()) {
            joined += ", ";
        }
        joined += item;
    }
    return joined;
}

/**
 * @brief Whether a key is one the simulator understands in either mode.
 * @param key Key text, without its value.
 * @return True for any of the six documented keys.
 * @note Deliberately mode-blind. Recognising a key here and rejecting it later as wrong-for-the-mode
 *       produces a far more useful message than calling it "unsupported".
 */
[[nodiscard]] bool isKnownKey(std::string_view key) {
    return key == kSimulation || key == kMissionControlFolder || key == kAlgorithm ||
           key == kMissionControl || key == kAlgorithmsFolder || key == kNumThreads;
}

/**
 * @brief Whether a known key belongs to a particular mode.
 * @param key Key text.
 * @param mode The mode that was requested.
 * @return True when the key is meaningful in @p mode.
 * @note `simulation` and `num_threads` belong to both; the remaining four are mode-specific.
 */
[[nodiscard]] bool isKeyValidInMode(std::string_view key, RunMode mode) {
    if (key == kSimulation || key == kNumThreads) {
        return true;
    }
    if (mode == RunMode::Comparative) {
        return key == kMissionControlFolder || key == kAlgorithm;
    }
    return key == kMissionControl || key == kAlgorithmsFolder;
}

/**
 * @brief Human-readable name of a mode, for error text.
 * @param mode The mode to name.
 * @return "comparative" or "competition".
 */
[[nodiscard]] const char* modeName(RunMode mode) {
    return mode == RunMode::Comparative ? "comparative" : "competition";
}

/**
 * @brief The key naming the single fixed plugin in a given mode.
 * @param mode The mode that was requested.
 * @return `algorithm` in comparative mode, `mission_control` in competitive mode.
 */
[[nodiscard]] std::string_view fixedPluginKey(RunMode mode) {
    return mode == RunMode::Comparative ? kAlgorithm : kMissionControl;
}

/**
 * @brief The key naming the varied plugin folder in a given mode.
 * @param mode The mode that was requested.
 * @return `mission_control_folder` in comparative mode, `algorithms_folder` in competitive mode.
 */
[[nodiscard]] std::string_view variedPluginKey(RunMode mode) {
    return mode == RunMode::Comparative ? kMissionControlFolder : kAlgorithmsFolder;
}

/**
 * @brief Split the argument vector into flags and key/value pairs.
 * @param argc Argument count as given to `main`.
 * @param argv Argument vector as given to `main`.
 * @return The raw arguments, with malformed tokens collected rather than rejected outright.
 * @note Splits on the **first** `=`, so a value may itself contain one. Splitting on the last would
 *       silently truncate any path containing an equals sign.
 * @note `argv[0]` is skipped; a null entry is treated as the end of the vector.
 */
[[nodiscard]] RawArguments tokenize(int argc, char** argv) {
    RawArguments raw{};

    for (int i = 1; i < argc; ++i) {
        if (argv[i] == nullptr) {
            break;
        }
        const std::string_view token{argv[i]};

        if (token == kComparativeFlag) {
            raw.comparative = true;
            continue;
        }
        if (token == kCompetitionFlag) {
            raw.competition = true;
            continue;
        }
        if (token == kVerboseFlag) {
            raw.verbose = true;
            continue;
        }

        const std::size_t equals = token.find('=');
        if (equals == std::string_view::npos || equals == 0) {
            /**
             * @note Covers unknown flags, bare positional tokens, and a token beginning with `=`.
             *       All of them are "unsupported arguments" as far as the assignment is concerned.
             */
            raw.unsupported.emplace_back(token);
            continue;
        }

        const std::string key{token.substr(0, equals)};
        const std::string value{token.substr(equals + 1)};

        if (!isKnownKey(key)) {
            raw.unsupported.emplace_back(token);
            continue;
        }
        if (value.empty()) {
            raw.empty_valued.push_back(key);
            continue;
        }
        if (!raw.values.emplace(key, value).second) {
            /**
             * @note A repeated key is ambiguous input. Silently taking the first or the last would
             *       hide a typo in a long command line, so it is reported instead.
             */
            raw.duplicated.push_back(key);
        }
    }

    return raw;
}

/**
 * @brief Parse a `num_threads` value.
 * @param value The text after the `=`.
 * @param out Receives the thread count on success; untouched on failure.
 * @return True when the whole token is a non-negative decimal number.
 * @note A supplied 0 is normalised to 1: under the threading rule, "spawn N threads in addition to
 *       main" with N of 0 is precisely what N of 1 already means, so it is accepted rather than
 *       rejected.
 * @note `std::from_chars` on an unsigned type rejects a leading sign, and comparing its end pointer
 *       against the end of the token rejects trailing junk such as `12abc`.
 */
[[nodiscard]] bool parseNumThreads(const std::string& value, std::size_t& out) {
    unsigned long long parsed = 0;
    const char* const begin = value.data();
    const char* const end = value.data() + value.size();

    const std::from_chars_result outcome = std::from_chars(begin, end, parsed);
    if (outcome.ec != std::errc{} || outcome.ptr != end) {
        return false;
    }

    out = parsed == 0 ? std::size_t{1} : static_cast<std::size_t>(parsed);
    return true;
}

} // namespace

/**
 * @brief Parse the command line into a typed configuration.
 * @param argc Argument count as given to `main`.
 * @param argv Argument vector as given to `main`; `argv[0]` is skipped.
 * @return The resolved arguments plus every problem found, in a stable category order.
 * @note Collects rather than fails fast, so one invocation reports every unsupported and every
 *       missing argument at once.
 * @note Touches no filesystem. `validateCommandLinePaths` is the separate half that does, which is
 *       what lets the parse rules be table-tested without any files on disk.
 */
CommandLineParseResult parseCommandLine(int argc, char** argv) {
    CommandLineParseResult result{};
    const RawArguments raw = tokenize(argc, argv);

    if (!raw.unsupported.empty()) {
        result.errors.push_back("unsupported argument(s): " + join(raw.unsupported));
    }
    if (!raw.duplicated.empty()) {
        result.errors.push_back("argument(s) given more than once: " + join(raw.duplicated));
    }
    if (!raw.empty_valued.empty()) {
        result.errors.push_back("argument(s) with an empty value: " + join(raw.empty_valued));
    }

    result.args.verbose = raw.verbose;

    /**
     * @note Exactly one mode flag is required. Until the mode is known there is no way to say which
     *       keys are required or which belong to the other mode, so those checks are skipped rather
     *       than guessed at - inventing "missing" arguments alongside a missing mode would be
     *       actively misleading.
     */
    const bool mode_known = raw.comparative != raw.competition;
    if (raw.comparative && raw.competition) {
        result.errors.emplace_back(
            "conflicting run modes: -comparative and -competition are mutually exclusive");
    } else if (!mode_known) {
        result.errors.emplace_back(
            "missing run mode: exactly one of -comparative or -competition is required");
    } else {
        result.args.mode = raw.comparative ? RunMode::Comparative : RunMode::Competition;
    }

    if (mode_known) {
        const RunMode mode = result.args.mode;

        std::vector<std::string> wrong_mode;
        for (const auto& entry : raw.values) {
            if (!isKeyValidInMode(entry.first, mode)) {
                wrong_mode.push_back(entry.first);
            }
        }
        if (!wrong_mode.empty()) {
            result.errors.push_back("argument(s) not valid in " + std::string{modeName(mode)} +
                                    " mode: " + join(wrong_mode));
        }

        std::vector<std::string> missing;
        const auto take = [&raw, &missing](std::string_view key, std::filesystem::path& target) {
            const auto found = raw.values.find(std::string{key});
            if (found == raw.values.end()) {
                missing.emplace_back(key);
            } else {
                target = found->second;
            }
        };
        take(kSimulation, result.args.composition_file);
        take(fixedPluginKey(mode), result.args.fixed_plugin_file);
        take(variedPluginKey(mode), result.args.varied_plugin_folder);

        if (!missing.empty()) {
            result.errors.push_back("missing required argument(s): " + join(missing));
        }
    }

    const auto threads = raw.values.find(std::string{kNumThreads});
    if (threads != raw.values.end() && !parseNumThreads(threads->second, result.args.num_threads)) {
        result.errors.push_back("num_threads must be a non-negative whole number, got \"" +
                                threads->second + "\"");
    }

    return result;
}

/**
 * @brief Check that the paths a command line named actually exist and are usable.
 * @param args Arguments produced by `parseCommandLine`.
 * @param errors Error list to append to, normally the one `parseCommandLine` returned.
 * @note Every check is guarded on the path being non-empty. An argument that was never supplied has
 *       already been reported as missing, and reporting it again as nonexistent would double every
 *       error a bare invocation produces.
 * @note Never throws: all filesystem queries use their `std::error_code` overloads, so a permission
 *       failure becomes an error message rather than an exception.
 */
void validateCommandLinePaths(const CommandLineArgs& args, std::vector<std::string>& errors) {
    std::error_code ec;

    if (!args.composition_file.empty()) {
        if (!std::filesystem::is_regular_file(args.composition_file, ec)) {
            errors.push_back(std::string{kSimulation} +
                             ": not an existing file: " + args.composition_file.string());
        } else {
            const std::ifstream probe(args.composition_file);
            if (!probe) {
                errors.push_back(std::string{kSimulation} + ": file cannot be opened for reading: " +
                                 args.composition_file.string());
            }
        }
    }

    if (!args.fixed_plugin_file.empty()) {
        /**
         * @note Only existence and file-ness are checked, deliberately not the `.so` extension.
         *       `is_regular_file` already catches the realistic mistake of passing a folder, and
         *       rejecting an unusually named but perfectly valid library would be worse than
         *       accepting it.
         */
        const std::string key{fixedPluginKey(args.mode)};
        if (!std::filesystem::is_regular_file(args.fixed_plugin_file, ec)) {
            errors.push_back(key + ": not an existing file: " + args.fixed_plugin_file.string());
        }
    }

    if (!args.varied_plugin_folder.empty()) {
        const std::string key{variedPluginKey(args.mode)};
        if (!std::filesystem::is_directory(args.varied_plugin_folder, ec)) {
            errors.push_back(key +
                             ": not an existing directory: " + args.varied_plugin_folder.string());
        } else {
            std::error_code list_ec;
            const std::vector<std::filesystem::path> found =
                enumerateSharedObjects(args.varied_plugin_folder, list_ec);
            if (list_ec) {
                errors.push_back(key + ": directory cannot be traversed: " +
                                 args.varied_plugin_folder.string() + " (" + list_ec.message() + ")");
            } else if (found.empty()) {
                /**
                 * @note The assignment says "zero `.so` files of the expected kind". The kind cannot
                 *       be known without loading a library, so only presence is checked here;
                 *       the registry reports a per-file kind mismatch later.
                 */
                errors.push_back(key +
                                 ": contains no .so files: " + args.varied_plugin_folder.string());
            }
        }
    }
}

/**
 * @brief The usage text printed whenever a command line is rejected.
 * @return A multi-line synopsis of both modes followed by a description of every argument.
 * @note Wording is our choice; the assignment fixes only the argument names and the requirement that
 *       usage accompanies every rejection.
 */
std::string commandLineUsage() {
    return R"(usage:
  simulator_323998450_211633813 -comparative simulation=<composition.yaml>
                                mission_control_folder=<folder> algorithm=<algo.so>
                                [num_threads=<num>] [-verbose]

  simulator_323998450_211633813 -competition simulation=<composition.yaml>
                                mission_control=<mc.so> algorithms_folder=<folder>
                                [num_threads=<num>] [-verbose]

arguments:
  -comparative                  run one algorithm against every mission control in a folder
  -competition                  run one mission control against every algorithm in a folder
  simulation=<file>             composition YAML: simulations, missions, drones and lidars
  mission_control_folder=<dir>  comparative mode: folder of mission control .so files
  algorithm=<file>              comparative mode: the single algorithm .so held fixed
  mission_control=<file>        competition mode: the single mission control .so held fixed
  algorithms_folder=<dir>       competition mode: folder of algorithm .so files
  num_threads=<num>             optional: worker threads besides main (default 1, meaning none)
  -verbose                      optional: ask the mission control to write verbose output

arguments may appear in any order, and '=' takes no surrounding spaces.
)";
}

} // namespace simulator
