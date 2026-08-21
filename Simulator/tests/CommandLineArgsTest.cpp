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

TEST(CommandLineArgs, CompetitionHappyPath) {
    const CommandLineParseResult result = parse(competitionTokens());
    ASSERT_TRUE(result.ok()) << result.errors.front();
    EXPECT_EQ(result.args.mode, RunMode::Competition);
    EXPECT_EQ(result.args.fixed_plugin_file, "mc.so");
    EXPECT_EQ(result.args.varied_plugin_folder, "algos");
}

TEST(CommandLineArgs, ArgumentsMayAppearInAnyOrder) {
    const CommandLineParseResult result = parse({"algorithm=algo.so", "-verbose",
                                                 "mission_control_folder=mcs", "num_threads=4",
                                                 "simulation=comp.yaml", "-comparative"});
    ASSERT_TRUE(result.ok()) << result.errors.front();
    EXPECT_EQ(result.args.composition_file, "comp.yaml");
    EXPECT_EQ(result.args.num_threads, 4u);
    EXPECT_TRUE(result.args.verbose);
}

TEST(CommandLineArgs, MissingModeIsReportedAndSuppressesKeyChecks) {
    const CommandLineParseResult result = parse({"simulation=comp.yaml"});
    ASSERT_FALSE(result.ok());
    EXPECT_TRUE(hasError(result, "missing run mode"));
    EXPECT_FALSE(hasError(result, "missing required argument"))
        << "required keys are unknowable without a mode and must not be guessed at";
}

TEST(CommandLineArgs, BothModesConflict) {
    const CommandLineParseResult result = parse({"-comparative", "-competition"});
    ASSERT_FALSE(result.ok());
    EXPECT_TRUE(hasError(result, "conflicting run modes"));
}

TEST(CommandLineArgs, EveryMissingArgumentIsNamedAtOnce) {
    const CommandLineParseResult result = parse({"-competition"});
    ASSERT_FALSE(result.ok());
    EXPECT_TRUE(hasError(result, "simulation"));
    EXPECT_TRUE(hasError(result, "mission_control"));
    EXPECT_TRUE(hasError(result, "algorithms_folder"));
}

TEST(CommandLineArgs, SingleMissingArgumentIsNamed) {
    std::vector<std::string> tokens = comparativeTokens();
    tokens.pop_back();
    const CommandLineParseResult result = parse(tokens);
    ASSERT_FALSE(result.ok());
    EXPECT_TRUE(hasError(result, "missing required argument(s): algorithm"));
}

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

TEST(CommandLineArgs, DoubleDashVerboseIsUnsupported) {
    std::vector<std::string> tokens = comparativeTokens();
    tokens.emplace_back("--verbose");
    const CommandLineParseResult result = parse(tokens);
    ASSERT_FALSE(result.ok());
    EXPECT_TRUE(hasError(result, "--verbose"));
    EXPECT_FALSE(result.args.verbose);
}

TEST(CommandLineArgs, KeyFromTheOtherModeIsRejectedByName) {
    std::vector<std::string> tokens = comparativeTokens();
    tokens.emplace_back("algorithms_folder=algos");
    const CommandLineParseResult result = parse(tokens);
    ASSERT_FALSE(result.ok());
    EXPECT_TRUE(hasError(result, "not valid in comparative mode"));
    EXPECT_TRUE(hasError(result, "algorithms_folder"));
}

TEST(CommandLineArgs, DuplicateKeyIsRejected) {
    std::vector<std::string> tokens = comparativeTokens();
    tokens.emplace_back("simulation=other.yaml");
    const CommandLineParseResult result = parse(tokens);
    ASSERT_FALSE(result.ok());
    EXPECT_TRUE(hasError(result, "more than once"));
    EXPECT_TRUE(hasError(result, "simulation"));
}

TEST(CommandLineArgs, EmptyValueIsRejected) {
    const CommandLineParseResult result =
        parse({"-comparative", "simulation=", "mission_control_folder=mcs", "algorithm=algo.so"});
    ASSERT_FALSE(result.ok());
    EXPECT_TRUE(hasError(result, "empty value"));
}

TEST(CommandLineArgs, LeadingEqualsIsUnsupported) {
    std::vector<std::string> tokens = comparativeTokens();
    tokens.emplace_back("=value");
    const CommandLineParseResult result = parse(tokens);
    ASSERT_FALSE(result.ok());
    EXPECT_TRUE(hasError(result, "=value"));
}

TEST(CommandLineArgs, ValueMayContainEquals) {
    const CommandLineParseResult result = parse({"-comparative", "simulation=odd=name.yaml",
                                                 "mission_control_folder=mcs", "algorithm=algo.so"});
    ASSERT_TRUE(result.ok()) << result.errors.front();
    EXPECT_EQ(result.args.composition_file, "odd=name.yaml");
}

TEST(CommandLineArgs, NumThreadsDefaultsToOne) {
    const CommandLineParseResult result = parse(comparativeTokens());
    ASSERT_TRUE(result.ok());
    EXPECT_EQ(result.args.num_threads, 1u);
}

TEST(CommandLineArgs, NumThreadsZeroNormalisesToOne) {
    std::vector<std::string> tokens = comparativeTokens();
    tokens.emplace_back("num_threads=0");
    const CommandLineParseResult result = parse(tokens);
    ASSERT_TRUE(result.ok()) << result.errors.front();
    EXPECT_EQ(result.args.num_threads, 1u)
        << "zero additional threads is exactly what 1 already means";
}

TEST(CommandLineArgs, NumThreadsAcceptsALargeValue) {
    std::vector<std::string> tokens = comparativeTokens();
    tokens.emplace_back("num_threads=1024");
    const CommandLineParseResult result = parse(tokens);
    ASSERT_TRUE(result.ok()) << result.errors.front();
    EXPECT_EQ(result.args.num_threads, 1024u);
}

TEST(CommandLineArgs, NumThreadsRejectsNonNumeric) {
    std::vector<std::string> tokens = comparativeTokens();
    tokens.emplace_back("num_threads=abc");
    const CommandLineParseResult result = parse(tokens);
    ASSERT_FALSE(result.ok());
    EXPECT_TRUE(hasError(result, "num_threads"));
}

TEST(CommandLineArgs, NumThreadsRejectsTrailingJunk) {
    std::vector<std::string> tokens = comparativeTokens();
    tokens.emplace_back("num_threads=12abc");
    const CommandLineParseResult result = parse(tokens);
    ASSERT_FALSE(result.ok()) << "a partially numeric value must not silently parse as 12";
    EXPECT_TRUE(hasError(result, "num_threads"));
}

TEST(CommandLineArgs, NumThreadsRejectsNegative) {
    std::vector<std::string> tokens = comparativeTokens();
    tokens.emplace_back("num_threads=-1");
    const CommandLineParseResult result = parse(tokens);
    ASSERT_FALSE(result.ok());
    EXPECT_TRUE(hasError(result, "num_threads"));
}

TEST(CommandLineArgs, NumThreadsRejectsOverflow) {
    std::vector<std::string> tokens = comparativeTokens();
    tokens.emplace_back("num_threads=99999999999999999999999");
    const CommandLineParseResult result = parse(tokens);
    ASSERT_FALSE(result.ok());
    EXPECT_TRUE(hasError(result, "num_threads"));
}

TEST(CommandLineArgs, NumThreadsEmptyValueIsRejected) {
    std::vector<std::string> tokens = comparativeTokens();
    tokens.emplace_back("num_threads=");
    const CommandLineParseResult result = parse(tokens);
    ASSERT_FALSE(result.ok());
    EXPECT_TRUE(hasError(result, "empty value"));
}

TEST(CommandLineArgs, NoArgumentsAtAllReportsOnlyTheMode) {
    const CommandLineParseResult result = parse({});
    ASSERT_FALSE(result.ok());
    EXPECT_TRUE(hasError(result, "missing run mode"));
}

TEST(CommandLineArgs, VerboseFlagIsRecognised) {
    std::vector<std::string> tokens = competitionTokens();
    tokens.emplace_back("-verbose");
    const CommandLineParseResult result = parse(tokens);
    ASSERT_TRUE(result.ok()) << result.errors.front();
    EXPECT_TRUE(result.args.verbose);
}

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
