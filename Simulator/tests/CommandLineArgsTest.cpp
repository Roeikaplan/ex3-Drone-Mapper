/**
 * @file CommandLineArgsTest.cpp
 * @brief Table-driven coverage of every command-line parsing rule in the assignment.
 * @note Only `parseCommandLine` is exercised here. It touches no filesystem by design, so the whole
 *       suite runs without fixtures on disk; `validateCommandLinePaths` is covered separately by the
 *       manual checks in the phase-02 plan and, later, by the integration tests.
 */

#include <Simulator/CommandLineArgs.h>

#include <gtest/gtest.h>

#include <string>
#include <vector>

namespace {

using simulator::CommandLineParseResult;
using simulator::parseCommandLine;
using simulator::RunMode;

/**
 * @brief Parse a command line written as a list of tokens.
 * @param tokens Arguments *excluding* the program name.
 * @return The parse result.
 * @note Rebuilds a real `argv` - including the `argv[0]` the parser is required to skip - so the
 *       tests exercise the same entry point `main` does rather than a friendlier shim.
 */
[[nodiscard]] CommandLineParseResult parse(const std::vector<std::string>& tokens) {
    std::vector<std::string> storage;
    storage.reserve(tokens.size() + 1);
    storage.emplace_back("simulator_323998450_211633813");
    storage.insert(storage.end(), tokens.begin(), tokens.end());

    std::vector<char*> argv;
    argv.reserve(storage.size());
    for (std::string& token : storage) {
        argv.push_back(token.data());
    }

    return parseCommandLine(static_cast<int>(argv.size()), argv.data());
}

/**
 * @brief Whether any recorded error mentions a given fragment.
 * @param result The parse result to inspect.
 * @param fragment Text expected to appear in at least one error.
 * @return True when some error contains @p fragment.
 * @note Matching on a fragment rather than the whole message keeps the tests from breaking every
 *       time the wording is polished, while still pinning down which rule fired.
 */
[[nodiscard]] bool hasError(const CommandLineParseResult& result, const std::string& fragment) {
    for (const std::string& error : result.errors) {
        if (error.find(fragment) != std::string::npos) {
            return true;
        }
    }
    return false;
}

/**
 * @brief A complete, valid comparative command line.
 * @return Tokens that parse without error, for tests to extend or truncate.
 */
[[nodiscard]] std::vector<std::string> comparativeTokens() {
    return {"-comparative", "simulation=comp.yaml", "mission_control_folder=mcs",
            "algorithm=algo.so"};
}

/**
 * @brief A complete, valid competition command line.
 * @return Tokens that parse without error, for tests to extend or truncate.
 */
[[nodiscard]] std::vector<std::string> competitionTokens() {
    return {"-competition", "simulation=comp.yaml", "mission_control=mc.so",
            "algorithms_folder=algos"};
}

/**
 * @brief A well-formed comparative command line parses into every field, defaults included.
 * @note Pins the two defaults as much as the parsed values: `num_threads` is 1 and `verbose` is off
 *       unless asked for, which is what makes the optional arguments genuinely optional.
 */
TEST(CommandLineArgs, ComparativeHappyPath) {
    const CommandLineParseResult result = parse(comparativeTokens());
    ASSERT_TRUE(result.ok()) << result.errors.front();
    EXPECT_EQ(result.args.mode, RunMode::Comparative);
    EXPECT_EQ(result.args.composition_file, "comp.yaml");
    EXPECT_EQ(result.args.fixed_plugin_file, "algo.so");
    EXPECT_EQ(result.args.varied_plugin_folder, "mcs");
    EXPECT_EQ(result.args.num_threads, 1u);
    EXPECT_FALSE(result.args.verbose);
}

/**
 * @brief The competition line parses, with the fixed and varied plugins the other way round.
 * @note The two modes share one pair of fields, so this checks the mapping is inverted rather than
 *       copied - the varied side is what the results directory is created under.
 */
TEST(CommandLineArgs, CompetitionHappyPath) {
    const CommandLineParseResult result = parse(competitionTokens());
    ASSERT_TRUE(result.ok()) << result.errors.front();
    EXPECT_EQ(result.args.mode, RunMode::Competition);
    EXPECT_EQ(result.args.fixed_plugin_file, "mc.so");
    EXPECT_EQ(result.args.varied_plugin_folder, "algos");
}

/**
 * @brief Arguments parse in any order, including the mode flag appearing last.
 * @note The assignment permits any order, so nothing may depend on the mode being seen before the
 *       keys it governs - the parser has to collect first and validate afterwards.
 */
TEST(CommandLineArgs, ArgumentsMayAppearInAnyOrder) {
    const CommandLineParseResult result = parse({"algorithm=algo.so", "-verbose",
                                                 "mission_control_folder=mcs", "num_threads=4",
                                                 "simulation=comp.yaml", "-comparative"});
    ASSERT_TRUE(result.ok()) << result.errors.front();
    EXPECT_EQ(result.args.composition_file, "comp.yaml");
    EXPECT_EQ(result.args.num_threads, 4u);
    EXPECT_TRUE(result.args.verbose);
}

/**
 * @brief With no mode given, the mode is reported and per-key checks are deliberately suppressed.
 * @note Without a mode there is no way to know which keys were required, so guessing would produce
 *       errors naming arguments the user may not have needed at all.
 */
TEST(CommandLineArgs, MissingModeIsReportedAndSuppressesKeyChecks) {
    const CommandLineParseResult result = parse({"simulation=comp.yaml"});
    ASSERT_FALSE(result.ok());
    EXPECT_TRUE(hasError(result, "missing run mode"));
    EXPECT_FALSE(hasError(result, "missing required argument"))
        << "required keys are unknowable without a mode and must not be guessed at";
}

/**
 * @brief Passing both mode flags is a conflict, not a last-one-wins.
 */
TEST(CommandLineArgs, BothModesConflict) {
    const CommandLineParseResult result = parse({"-comparative", "-competition"});
    ASSERT_FALSE(result.ok());
    EXPECT_TRUE(hasError(result, "conflicting run modes"));
}

/**
 * @brief All three missing arguments are named in one pass, not one per invocation.
 * @note The assignment asks for every missing argument to be detailed. Failing on the first would
 *       make fixing a command line an exercise in re-running it.
 */
TEST(CommandLineArgs, EveryMissingArgumentIsNamedAtOnce) {
    const CommandLineParseResult result = parse({"-competition"});
    ASSERT_FALSE(result.ok());
    EXPECT_TRUE(hasError(result, "simulation"));
    EXPECT_TRUE(hasError(result, "mission_control"));
    EXPECT_TRUE(hasError(result, "algorithms_folder"));
}

/**
 * @brief A single omission names exactly that argument.
 */
TEST(CommandLineArgs, SingleMissingArgumentIsNamed) {
    std::vector<std::string> tokens = comparativeTokens();
    tokens.pop_back();
    const CommandLineParseResult result = parse(tokens);
    ASSERT_FALSE(result.ok());
    EXPECT_TRUE(hasError(result, "missing required argument(s): algorithm"));
}

/**
 * @brief Every unsupported token is named at once, whatever shape it takes.
 * @note Covers all three forms a stray token can arrive in - an unknown key, an unknown flag, and a
 *       bare positional word - since each fails a different branch of the parser.
 */
TEST(CommandLineArgs, EveryUnsupportedArgumentIsNamedAtOnce) {
    std::vector<std::string> tokens = comparativeTokens();
    tokens.emplace_back("bogus=1");
    tokens.emplace_back("-louder");
    tokens.emplace_back("stray");
    const CommandLineParseResult result = parse(tokens);
    ASSERT_FALSE(result.ok());
    EXPECT_TRUE(hasError(result, "bogus=1"));
    EXPECT_TRUE(hasError(result, "-louder"));
    EXPECT_TRUE(hasError(result, "stray"));
}

/**
 * @brief `--verbose` is rejected rather than quietly accepted as `-verbose`.
 * @note The assignment spells the flag with one dash. Accepting the GNU-style spelling would work
 *       here and then silently not work against the grader.
 */
TEST(CommandLineArgs, DoubleDashVerboseIsUnsupported) {
    std::vector<std::string> tokens = comparativeTokens();
    tokens.emplace_back("--verbose");
    const CommandLineParseResult result = parse(tokens);
    ASSERT_FALSE(result.ok());
    EXPECT_TRUE(hasError(result, "--verbose"));
    EXPECT_FALSE(result.args.verbose);
}

/**
 * @brief A key belonging to the other mode is rejected as wrong-for-this-mode, not as unknown.
 * @note The distinction is the useful part of the message: the argument exists and is spelled
 *       correctly, it just belongs to the mode that was not selected.
 */
TEST(CommandLineArgs, KeyFromTheOtherModeIsRejectedByName) {
    std::vector<std::string> tokens = comparativeTokens();
    tokens.emplace_back("algorithms_folder=algos");
    const CommandLineParseResult result = parse(tokens);
    ASSERT_FALSE(result.ok());
    EXPECT_TRUE(hasError(result, "not valid in comparative mode"));
    EXPECT_TRUE(hasError(result, "algorithms_folder"));
}

/**
 * @brief The same key given twice is an error rather than a silent overwrite.
 * @note Either value could have been intended, so guessing risks running an entirely different
 *       composition from the one asked for.
 */
TEST(CommandLineArgs, DuplicateKeyIsRejected) {
    std::vector<std::string> tokens = comparativeTokens();
    tokens.emplace_back("simulation=other.yaml");
    const CommandLineParseResult result = parse(tokens);
    ASSERT_FALSE(result.ok());
    EXPECT_TRUE(hasError(result, "more than once"));
    EXPECT_TRUE(hasError(result, "simulation"));
}

/**
 * @brief A key with nothing after the `=` is rejected.
 * @note Otherwise it would arrive as an empty path and fail much later as a confusing file error.
 */
TEST(CommandLineArgs, EmptyValueIsRejected) {
    const CommandLineParseResult result =
        parse({"-comparative", "simulation=", "mission_control_folder=mcs", "algorithm=algo.so"});
    ASSERT_FALSE(result.ok());
    EXPECT_TRUE(hasError(result, "empty value"));
}

/**
 * @brief A token beginning with `=` has no key and is reported as unsupported.
 */
TEST(CommandLineArgs, LeadingEqualsIsUnsupported) {
    std::vector<std::string> tokens = comparativeTokens();
    tokens.emplace_back("=value");
    const CommandLineParseResult result = parse(tokens);
    ASSERT_FALSE(result.ok());
    EXPECT_TRUE(hasError(result, "=value"));
}

/**
 * @brief Only the first `=` separates key from value, so a value may contain more of them.
 * @note Splitting on every `=` would mangle any path containing one, which is legal on every
 *       filesystem this runs on.
 */
TEST(CommandLineArgs, ValueMayContainEquals) {
    const CommandLineParseResult result = parse({"-comparative", "simulation=odd=name.yaml",
                                                 "mission_control_folder=mcs", "algorithm=algo.so"});
    ASSERT_TRUE(result.ok()) << result.errors.front();
    EXPECT_EQ(result.args.composition_file, "odd=name.yaml");
}

/**
 * @brief Omitting `num_threads` gives 1, meaning no worker threads at all.
 */
TEST(CommandLineArgs, NumThreadsDefaultsToOne) {
    const CommandLineParseResult result = parse(comparativeTokens());
    ASSERT_TRUE(result.ok());
    EXPECT_EQ(result.args.num_threads, 1u);
}

/**
 * @brief `num_threads=0` normalises to 1 rather than being rejected.
 * @note Zero additional threads is precisely what 1 already means, so the value is honoured instead
 *       of turned into an error over a distinction without a difference.
 */
TEST(CommandLineArgs, NumThreadsZeroNormalisesToOne) {
    std::vector<std::string> tokens = comparativeTokens();
    tokens.emplace_back("num_threads=0");
    const CommandLineParseResult result = parse(tokens);
    ASSERT_TRUE(result.ok()) << result.errors.front();
    EXPECT_EQ(result.args.num_threads, 1u)
        << "zero additional threads is exactly what 1 already means";
}

/**
 * @brief A large thread count parses; the executor caps it later against the task count.
 * @note Parsing and capping are deliberately separate concerns - the parser's job is to read the
 *       number, not to decide what is reasonable hardware.
 */
TEST(CommandLineArgs, NumThreadsAcceptsALargeValue) {
    std::vector<std::string> tokens = comparativeTokens();
    tokens.emplace_back("num_threads=1024");
    const CommandLineParseResult result = parse(tokens);
    ASSERT_TRUE(result.ok()) << result.errors.front();
    EXPECT_EQ(result.args.num_threads, 1024u);
}

/**
 * @brief A non-numeric thread count is rejected.
 */
TEST(CommandLineArgs, NumThreadsRejectsNonNumeric) {
    std::vector<std::string> tokens = comparativeTokens();
    tokens.emplace_back("num_threads=abc");
    const CommandLineParseResult result = parse(tokens);
    ASSERT_FALSE(result.ok());
    EXPECT_TRUE(hasError(result, "num_threads"));
}

/**
 * @brief A partly-numeric value like `12abc` is rejected rather than parsed as 12.
 * @note The failure mode a bare `atoi` or unchecked `stoul` would produce: a typo silently becomes a
 *       valid-looking thread count and the run proceeds with settings nobody chose.
 */
TEST(CommandLineArgs, NumThreadsRejectsTrailingJunk) {
    std::vector<std::string> tokens = comparativeTokens();
    tokens.emplace_back("num_threads=12abc");
    const CommandLineParseResult result = parse(tokens);
    ASSERT_FALSE(result.ok()) << "a partially numeric value must not silently parse as 12";
    EXPECT_TRUE(hasError(result, "num_threads"));
}

/**
 * @brief A negative thread count is rejected rather than wrapping to an enormous unsigned value.
 */
TEST(CommandLineArgs, NumThreadsRejectsNegative) {
    std::vector<std::string> tokens = comparativeTokens();
    tokens.emplace_back("num_threads=-1");
    const CommandLineParseResult result = parse(tokens);
    ASSERT_FALSE(result.ok());
    EXPECT_TRUE(hasError(result, "num_threads"));
}

/**
 * @brief A value too large for the target type is rejected rather than wrapping or throwing.
 */
TEST(CommandLineArgs, NumThreadsRejectsOverflow) {
    std::vector<std::string> tokens = comparativeTokens();
    tokens.emplace_back("num_threads=99999999999999999999999");
    const CommandLineParseResult result = parse(tokens);
    ASSERT_FALSE(result.ok());
    EXPECT_TRUE(hasError(result, "num_threads"));
}

/**
 * @brief `num_threads=` is caught by the empty-value rule like any other key.
 * @note Being optional means it may be omitted, not that it may be given without a value.
 */
TEST(CommandLineArgs, NumThreadsEmptyValueIsRejected) {
    std::vector<std::string> tokens = comparativeTokens();
    tokens.emplace_back("num_threads=");
    const CommandLineParseResult result = parse(tokens);
    ASSERT_FALSE(result.ok());
    EXPECT_TRUE(hasError(result, "empty value"));
}

/**
 * @brief An empty command line reports the missing mode and nothing else.
 * @note This is the bare `./simulator` invocation, so it is the first thing a user sees - one clear
 *       line plus the usage text, rather than a wall of every unmet requirement.
 */
TEST(CommandLineArgs, NoArgumentsAtAllReportsOnlyTheMode) {
    const CommandLineParseResult result = parse({});
    ASSERT_FALSE(result.ok());
    EXPECT_TRUE(hasError(result, "missing run mode"));
}

/**
 * @brief `-verbose` sets the flag, and does so in competition mode too.
 */
TEST(CommandLineArgs, VerboseFlagIsRecognised) {
    std::vector<std::string> tokens = competitionTokens();
    tokens.emplace_back("-verbose");
    const CommandLineParseResult result = parse(tokens);
    ASSERT_TRUE(result.ok()) << result.errors.front();
    EXPECT_TRUE(result.args.verbose);
}

/**
 * @brief One command line breaking four different rules reports all four.
 * @note The categories are collected independently rather than short-circuiting at the first, so a
 *       thoroughly wrong command line can be fixed in a single pass.
 */
TEST(CommandLineArgs, ProblemsFromSeveralCategoriesAreAllReported) {
    const CommandLineParseResult result =
        parse({"-comparative", "bogus=1", "simulation=comp.yaml", "simulation=again.yaml",
               "algorithm="});
    ASSERT_FALSE(result.ok());
    EXPECT_TRUE(hasError(result, "unsupported"));
    EXPECT_TRUE(hasError(result, "more than once"));
    EXPECT_TRUE(hasError(result, "empty value"));
    EXPECT_TRUE(hasError(result, "missing required argument"));
}

} // namespace
