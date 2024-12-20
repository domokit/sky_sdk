// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/fml/command_line.h"

#include <string_view>
#include <utility>

#include "flutter/fml/macros.h"
#include "gtest/gtest.h"

namespace fml {
namespace {

TEST(CommandLineTest, Basic) {
  // Making this const verifies that the methods called are const.
  const auto cl = CommandLineFromInitializerList(
      {"my_program", "--flag1", "--flag2=value2", "arg1", "arg2", "arg3"});

  EXPECT_TRUE(cl.has_argv0());
  EXPECT_EQ("my_program", cl.argv0());

  EXPECT_EQ(2u, cl.options().size());
  EXPECT_EQ("flag1", cl.options()[0].name);
  EXPECT_EQ(std::string(), cl.options()[0].value);
  EXPECT_EQ("flag2", cl.options()[1].name);
  EXPECT_EQ("value2", cl.options()[1].value);

  EXPECT_EQ(3u, cl.positional_args().size());
  EXPECT_EQ("arg1", cl.positional_args()[0]);
  EXPECT_EQ("arg2", cl.positional_args()[1]);
  EXPECT_EQ("arg3", cl.positional_args()[2]);

  EXPECT_TRUE(cl.HasOption("flag1"));
  EXPECT_TRUE(cl.HasOption("flag1", nullptr));
  size_t index = static_cast<size_t>(-1);
  EXPECT_TRUE(cl.HasOption("flag2", &index));
  EXPECT_EQ(1u, index);
  EXPECT_FALSE(cl.HasOption("flag3"));
  EXPECT_FALSE(cl.HasOption("flag3", nullptr));

  std::string value = "nonempty";
  EXPECT_TRUE(cl.GetOptionValue("flag1", &value));
  EXPECT_EQ(std::string(), value);
  EXPECT_TRUE(cl.GetOptionValue("flag2", &value));
  EXPECT_EQ("value2", value);
  EXPECT_FALSE(cl.GetOptionValue("flag3", &value));

  EXPECT_EQ(std::string(), cl.GetOptionValueWithDefault("flag1", "nope"));
  EXPECT_EQ("value2", cl.GetOptionValueWithDefault("flag2", "nope"));
  EXPECT_EQ("nope", cl.GetOptionValueWithDefault("flag3", "nope"));
}

TEST(CommandLineTest, DefaultConstructor) {
  CommandLine cl;
  EXPECT_FALSE(cl.has_argv0());
  EXPECT_EQ(std::string(), cl.argv0());
  EXPECT_EQ(std::vector<CommandLine::Option>(), cl.options());
  EXPECT_EQ(std::vector<std::string>(), cl.positional_args());
}

TEST(CommandLineTest, ComponentConstructor) {
  const std::string argv0 = "my_program";
  const std::vector<CommandLine::Option> options = {
      CommandLine::Option("flag", "value")};
  const std::vector<std::string> positional_args = {"arg"};

  CommandLine cl(argv0, options, positional_args);
  EXPECT_TRUE(cl.has_argv0());
  EXPECT_EQ(argv0, cl.argv0());
  EXPECT_EQ(options, cl.options());
  EXPECT_EQ(positional_args, cl.positional_args());
  EXPECT_EQ("value", cl.GetOptionValueWithDefault("flag", "nope"));
}

TEST(CommandLineTest, CommandLineFromIteratorsFindFirstPositionalArg) {
  // This shows how one might process subcommands.
  {
    static std::vector<std::string> argv = {"my_program", "--flag1",
                                            "--flag2",    "subcommand",
                                            "--subflag",  "subarg"};
    auto first = argv.cbegin();
    auto last = argv.cend();
    std::vector<std::string>::const_iterator sub_first;
    auto cl =
        CommandLineFromIteratorsFindFirstPositionalArg(first, last, &sub_first);
    EXPECT_TRUE(cl.has_argv0());
    EXPECT_EQ(argv[0], cl.argv0());
    std::vector<CommandLine::Option> expected_options = {
        CommandLine::Option("flag1"), CommandLine::Option("flag2")};
    EXPECT_EQ(expected_options, cl.options());
    std::vector<std::string> expected_positional_args = {argv[3], argv[4],
                                                         argv[5]};
    EXPECT_EQ(expected_positional_args, cl.positional_args());
    EXPECT_TRUE(cl.HasOption("flag1", nullptr));
    EXPECT_TRUE(cl.HasOption("flag2", nullptr));
    EXPECT_FALSE(cl.HasOption("subflag", nullptr));

    EXPECT_EQ(first + 3, sub_first);
    auto sub_cl = CommandLineFromIterators(sub_first, last);
    EXPECT_TRUE(sub_cl.has_argv0());
    EXPECT_EQ(argv[3], sub_cl.argv0());
    std::vector<CommandLine::Option> expected_sub_options = {
        CommandLine::Option("subflag")};
    EXPECT_EQ(expected_sub_options, sub_cl.options());
    std::vector<std::string> expected_sub_positional_args = {argv[5]};
    EXPECT_EQ(expected_sub_positional_args, sub_cl.positional_args());
    EXPECT_FALSE(sub_cl.HasOption("flag1", nullptr));
    EXPECT_FALSE(sub_cl.HasOption("flag2", nullptr));
    EXPECT_TRUE(sub_cl.HasOption("subflag", nullptr));
  }

  // No positional argument.
  {
    static std::vector<std::string> argv = {"my_program", "--flag"};
    std::vector<std::string>::const_iterator sub_first;
    auto cl = CommandLineFromIteratorsFindFirstPositionalArg(
        argv.cbegin(), argv.cend(), &sub_first);
    EXPECT_EQ(argv.cend(), sub_first);
  }

  // Multiple positional arguments.
  {
    static std::vector<std::string> argv = {"my_program", "arg1", "arg2"};
    std::vector<std::string>::const_iterator sub_first;
    auto cl = CommandLineFromIteratorsFindFirstPositionalArg(
        argv.cbegin(), argv.cend(), &sub_first);
    EXPECT_EQ(argv.cbegin() + 1, sub_first);
  }

  // "--".
  {
    static std::vector<std::string> argv = {"my_program", "--", "--arg"};
    std::vector<std::string>::const_iterator sub_first;
    auto cl = CommandLineFromIteratorsFindFirstPositionalArg(
        argv.cbegin(), argv.cend(), &sub_first);
    EXPECT_EQ(argv.cbegin() + 2, sub_first);
  }
}

TEST(CommandLineTest, CommmandLineFromIterators) {
  {
    // Note (here and below): The |const| ensures that the factory method can
    // accept const iterators.
    const std::vector<std::string> argv = {"my_program", "--flag=value", "arg"};

    auto cl = CommandLineFromIterators(argv.begin(), argv.end());
    EXPECT_TRUE(cl.has_argv0());
    EXPECT_EQ(argv[0], cl.argv0());
    std::vector<CommandLine::Option> expected_options = {
        CommandLine::Option("flag", "value")};
    EXPECT_EQ(expected_options, cl.options());
    std::vector<std::string> expected_positional_args = {argv[2]};
    EXPECT_EQ(expected_positional_args, cl.positional_args());
    EXPECT_EQ("value", cl.GetOptionValueWithDefault("flag", "nope"));
  }

  // Can handle empty argv.
  {
    const std::vector<std::string> argv;

    auto cl = CommandLineFromIterators(argv.begin(), argv.end());
    EXPECT_FALSE(cl.has_argv0());
    EXPECT_EQ(std::string(), cl.argv0());
    EXPECT_EQ(std::vector<CommandLine::Option>(), cl.options());
    EXPECT_EQ(std::vector<std::string>(), cl.positional_args());
  }

  // Can handle empty |argv[0]|.
  {
    const std::vector<std::string> argv = {""};

    auto cl = CommandLineFromIterators(argv.begin(), argv.end());
    EXPECT_TRUE(cl.has_argv0());
    EXPECT_EQ(std::string(), cl.argv0());
    EXPECT_EQ(std::vector<CommandLine::Option>(), cl.options());
    EXPECT_EQ(std::vector<std::string>(), cl.positional_args());
  }

  // Can also take a vector of |const char*|s.
  {
    const std::vector<const char*> argv = {"my_program", "--flag=value", "arg"};

    auto cl = CommandLineFromIterators(argv.begin(), argv.end());
    EXPECT_TRUE(cl.has_argv0());
    EXPECT_EQ(argv[0], cl.argv0());
    std::vector<CommandLine::Option> expected_options = {
        CommandLine::Option("flag", "value")};
    EXPECT_EQ(expected_options, cl.options());
    std::vector<std::string> expected_positional_args = {argv[2]};
    EXPECT_EQ(expected_positional_args, cl.positional_args());
    EXPECT_EQ("value", cl.GetOptionValueWithDefault("flag", "nope"));
  }

  // Or a plain old array.
  {
    static const char* const argv[] = {"my_program", "--flag=value", "arg"};

    auto cl = CommandLineFromIterators(argv, argv + std::size(argv));
    EXPECT_TRUE(cl.has_argv0());
    EXPECT_EQ(argv[0], cl.argv0());
    std::vector<CommandLine::Option> expected_options = {
        CommandLine::Option("flag", "value")};
    EXPECT_EQ(expected_options, cl.options());
    std::vector<std::string> expected_positional_args = {argv[2]};
    EXPECT_EQ(expected_positional_args, cl.positional_args());
    EXPECT_EQ("value", cl.GetOptionValueWithDefault("flag", "nope"));
  }
}

TEST(CommandLineTest, CommandLineFromArgcArgv) {
  static const char* const argv[] = {"my_program", "--flag=value", "arg"};
  const int argc = static_cast<int>(std::size(argv));

  auto cl = CommandLineFromArgcArgv(argc, argv);
  EXPECT_TRUE(cl.has_argv0());
  EXPECT_EQ(argv[0], cl.argv0());
  std::vector<CommandLine::Option> expected_options = {
      CommandLine::Option("flag", "value")};
  EXPECT_EQ(expected_options, cl.options());
  std::vector<std::string> expected_positional_args = {argv[2]};
  EXPECT_EQ(expected_positional_args, cl.positional_args());
  EXPECT_EQ("value", cl.GetOptionValueWithDefault("flag", "nope"));
}

TEST(CommandLineTest, CommandLineFromInitializerList) {
  {
    std::initializer_list<const char*> il = {"my_program", "--flag=value",
                                             "arg"};
    auto cl = CommandLineFromInitializerList(il);
    EXPECT_TRUE(cl.has_argv0());
    EXPECT_EQ("my_program", cl.argv0());
    std::vector<CommandLine::Option> expected_options = {
        CommandLine::Option("flag", "value")};
    EXPECT_EQ(expected_options, cl.options());
    std::vector<std::string> expected_positional_args = {"arg"};
    EXPECT_EQ(expected_positional_args, cl.positional_args());
    EXPECT_EQ("value", cl.GetOptionValueWithDefault("flag", "nope"));
  }

  {
    std::initializer_list<std::string> il = {"my_program", "--flag=value",
                                             "arg"};
    auto cl = CommandLineFromInitializerList(il);
    EXPECT_TRUE(cl.has_argv0());
    EXPECT_EQ("my_program", cl.argv0());
    std::vector<CommandLine::Option> expected_options = {
        CommandLine::Option("flag", "value")};
    EXPECT_EQ(expected_options, cl.options());
    std::vector<std::string> expected_positional_args = {"arg"};
    EXPECT_EQ(expected_positional_args, cl.positional_args());
    EXPECT_EQ("value", cl.GetOptionValueWithDefault("flag", "nope"));
  }
}

TEST(CommandLineTest, OddArguments) {
  {
    // Except for "arg", these are all options.
    auto cl = CommandLineFromInitializerList(
        {"my_program", "--=", "--=foo", "--bar=", "--==", "--===", "--==x",
         "arg"});
    EXPECT_TRUE(cl.has_argv0());
    EXPECT_EQ("my_program", cl.argv0());
    std::vector<CommandLine::Option> expected_options = {
        CommandLine::Option("="),      CommandLine::Option("=foo"),
        CommandLine::Option("bar"),    CommandLine::Option("="),
        CommandLine::Option("=", "="), CommandLine::Option("=", "x")};
    EXPECT_EQ(expected_options, cl.options());
    std::vector<std::string> expected_positional_args = {"arg"};
    EXPECT_EQ(expected_positional_args, cl.positional_args());
  }

  // "-x" is an argument, not an options.
  {
    auto cl = CommandLineFromInitializerList({"", "-x"});
    EXPECT_TRUE(cl.has_argv0());
    EXPECT_EQ(std::string(), cl.argv0());
    EXPECT_EQ(std::vector<CommandLine::Option>(), cl.options());
    std::vector<std::string> expected_positional_args = {"-x"};
    EXPECT_EQ(expected_positional_args, cl.positional_args());
  }

  // Ditto for "-".
  {
    auto cl = CommandLineFromInitializerList({"", "-"});
    EXPECT_TRUE(cl.has_argv0());
    EXPECT_EQ(std::string(), cl.argv0());
    EXPECT_EQ(std::vector<CommandLine::Option>(), cl.options());
    std::vector<std::string> expected_positional_args = {"-"};
    EXPECT_EQ(expected_positional_args, cl.positional_args());
  }

  // "--" terminates option processing, but isn't an argument in the first
  // occurrence.
  {
    auto cl = CommandLineFromInitializerList(
        {"", "--flag=value", "--", "--not-a-flag", "arg", "--"});
    EXPECT_TRUE(cl.has_argv0());
    EXPECT_EQ(std::string(), cl.argv0());
    std::vector<CommandLine::Option> expected_options = {
        CommandLine::Option("flag", "value")};
    std::vector<std::string> expected_positional_args = {"--not-a-flag", "arg",
                                                         "--"};
    EXPECT_EQ(expected_positional_args, cl.positional_args());
  }
}

TEST(CommandLineTest, MultipleOccurrencesOfOption) {
  auto cl = CommandLineFromInitializerList(
      {"my_program", "--flag1=value1", "--flag2=value2", "--flag1=value3"});
  std::vector<CommandLine::Option> expected_options = {
      CommandLine::Option("flag1", "value1"),
      CommandLine::Option("flag2", "value2"),
      CommandLine::Option("flag1", "value3")};
  EXPECT_EQ("value3", cl.GetOptionValueWithDefault("flag1", "nope"));
  EXPECT_EQ("value2", cl.GetOptionValueWithDefault("flag2", "nope"));
  std::vector<std::string_view> values = cl.GetOptionValues("flag1");
  ASSERT_EQ(2u, values.size());
  EXPECT_EQ("value1", values[0]);
  EXPECT_EQ("value3", values[1]);
}

// |cl1| and |cl2| should be not equal.
void ExpectNotEqual(const char* message,
                    std::initializer_list<std::string> c1,
                    std::initializer_list<std::string> c2) {
  SCOPED_TRACE(message);

  const auto cl1 = CommandLineFromInitializerList(c1);
  const auto cl2 = CommandLineFromInitializerList(c2);

  // These are tautological.
  EXPECT_TRUE(cl1 == cl1);
  EXPECT_FALSE(cl1 != cl1);
  EXPECT_TRUE(cl2 == cl2);
  EXPECT_FALSE(cl2 != cl2);

  // These rely on |cl1| not being equal to |cl2|.
  EXPECT_FALSE(cl1 == cl2);
  EXPECT_TRUE(cl1 != cl2);
  EXPECT_FALSE(cl2 == cl1);
  EXPECT_TRUE(cl2 != cl1);
}

void ExpectEqual(const char* message,
                 std::initializer_list<std::string> c1,
                 std::initializer_list<std::string> c2) {
  SCOPED_TRACE(message);

  const auto cl1 = CommandLineFromInitializerList(c1);
  const auto cl2 = CommandLineFromInitializerList(c2);

  // These are tautological.
  EXPECT_TRUE(cl1 == cl1);
  EXPECT_FALSE(cl1 != cl1);
  EXPECT_TRUE(cl2 == cl2);
  EXPECT_FALSE(cl2 != cl2);

  // These rely on |cl1| being equal to |cl2|.
  EXPECT_TRUE(cl1 == cl2);
  EXPECT_FALSE(cl1 != cl2);
  EXPECT_TRUE(cl2 == cl1);
  EXPECT_FALSE(cl2 != cl1);
}

TEST(CommandLineTest, ComparisonOperators) {
  ExpectNotEqual("1", {}, {""});
  ExpectNotEqual("2", {"abc"}, {"def"});
  ExpectNotEqual("3", {"abc", "--flag"}, {"abc"});
  ExpectNotEqual("4", {"abc", "--flag1"}, {"abc", "--flag2"});
  ExpectNotEqual("5", {"abc", "--flag1", "--flag2"}, {"abc", "--flag1"});
  ExpectNotEqual("6", {"abc", "arg"}, {"abc"});
  ExpectNotEqual("7", {"abc", "arg1"}, {"abc", "arg2"});
  ExpectNotEqual("8", {"abc", "arg1", "arg2"}, {"abc", "arg1"});
  ExpectNotEqual("9", {"abc", "--flag", "arg1"}, {"abc", "--flag", "arg2"});

  // However, the presence of an unnecessary "--" shouldn't affect what's
  // constructed.
  ExpectEqual("10", {"abc", "--flag", "arg"}, {"abc", "--flag", "--", "arg"});
}

TEST(CommandLineTest, MoveAndCopy) {
  const auto cl = CommandLineFromInitializerList(
      {"my_program", "--flag1=value1", "--flag2", "arg"});

  // Copy constructor.
  CommandLine cl2(cl);
  EXPECT_EQ(cl, cl2);
  // Check that |option_index_| gets copied too.
  EXPECT_EQ("value1", cl2.GetOptionValueWithDefault("flag1", "nope"));

  // Move constructor.
  CommandLine cl3(std::move(cl2));
  EXPECT_EQ(cl, cl3);
  EXPECT_EQ("value1", cl3.GetOptionValueWithDefault("flag1", "nope"));

  // Copy assignment.
  CommandLine cl4;
  EXPECT_NE(cl, cl4);
  cl4 = cl;
  EXPECT_EQ(cl, cl4);
  EXPECT_EQ("value1", cl4.GetOptionValueWithDefault("flag1", "nope"));

  // Move assignment.
  CommandLine cl5;
  EXPECT_NE(cl, cl5);
  cl5 = std::move(cl4);
  EXPECT_EQ(cl, cl5);
  EXPECT_EQ("value1", cl5.GetOptionValueWithDefault("flag1", "nope"));
}

void ToArgvHelper(const char* message, std::initializer_list<std::string> c) {
  SCOPED_TRACE(message);
  std::vector<std::string> argv = c;
  auto cl = CommandLineFromInitializerList(c);
  EXPECT_EQ(argv, CommandLineToArgv(cl));
}

TEST(CommandLineTest, CommandLineToArgv) {
  ToArgvHelper("1", {});
  ToArgvHelper("2", {""});
  ToArgvHelper("3", {"my_program"});
  ToArgvHelper("4", {"my_program", "--flag"});
  ToArgvHelper("5", {"my_program", "--flag1", "--flag2=value"});
  ToArgvHelper("6", {"my_program", "arg"});
  ToArgvHelper("7", {"my_program", "arg1", "arg2"});
  ToArgvHelper("8", {"my_program", "--flag1", "--flag2=value", "arg1", "arg2"});
  ToArgvHelper("9", {"my_program", "--flag", "--", "--not-a-flag"});
  ToArgvHelper("10", {"my_program", "--flag", "arg", "--"});

  // However, |CommandLineToArgv()| will "strip" an unneeded "--".
  {
    auto cl = CommandLineFromInitializerList({"my_program", "--"});
    std::vector<std::string> argv = {"my_program"};
    EXPECT_EQ(argv, CommandLineToArgv(cl));
  }
  {
    auto cl =
        CommandLineFromInitializerList({"my_program", "--flag", "--", "arg"});
    std::vector<std::string> argv = {"my_program", "--flag", "arg"};
    EXPECT_EQ(argv, CommandLineToArgv(cl));
  }
}

}  // namespace
}  // namespace fml
