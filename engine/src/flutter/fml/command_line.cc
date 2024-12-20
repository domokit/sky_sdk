// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/fml/command_line.h"

namespace fml {

// CommandLine -----------------------------------------------------------------

CommandLine::Option::Option(const std::string& name) : name(name) {}

CommandLine::Option::Option(const std::string& name, const std::string& value)
    : name(name), value(value) {}

CommandLine::CommandLine() = default;

CommandLine::CommandLine(const CommandLine& from) = default;

CommandLine::CommandLine(CommandLine&& from) = default;

CommandLine::CommandLine(const std::string& argv0,
                         const std::vector<Option>& options,
                         const std::vector<std::string>& positional_args)
    : has_argv0_(true),
      argv0_(argv0),
      options_(options),
      positional_args_(positional_args) {
  for (size_t i = 0; i < options_.size(); i++) {
    option_index_[options_[i].name] = i;
  }
}

CommandLine::~CommandLine() = default;

CommandLine& CommandLine::operator=(const CommandLine& from) = default;

CommandLine& CommandLine::operator=(CommandLine&& from) = default;

bool CommandLine::HasOption(std::string_view name, size_t* index) const {
  auto it = option_index_.find(name.data());
  if (it == option_index_.end()) {
    return false;
  }
  if (index) {
    *index = it->second;
  }
  return true;
}

bool CommandLine::GetOptionValue(std::string_view name,
                                 std::string* value) const {
  size_t index;
  if (!HasOption(name, &index)) {
    return false;
  }
  *value = options_[index].value;
  return true;
}

std::vector<std::string_view> CommandLine::GetOptionValues(
    std::string_view name) const {
  std::vector<std::string_view> ret;
  for (const auto& option : options_) {
    if (option.name == name) {
      ret.push_back(option.value);
    }
  }
  return ret;
}

std::string CommandLine::GetOptionValueWithDefault(
    std::string_view name,
    std::string_view default_value) const {
  size_t index;
  if (!HasOption(name, &index)) {
    return {default_value.data(), default_value.size()};
  }
  return options_[index].value;
}

// Factory functions (etc.) ----------------------------------------------------

namespace internal {

CommandLineBuilder::CommandLineBuilder() {}
CommandLineBuilder::~CommandLineBuilder() {}

bool CommandLineBuilder::ProcessArg(const std::string& arg) {
  if (!has_argv0_) {
    has_argv0_ = true;
    argv0_ = arg;
    return false;
  }

  // If we've seen a positional argument, then the remaining arguments are also
  // positional.
  if (started_positional_args_) {
    bool rv = positional_args_.empty();
    positional_args_.push_back(arg);
    return rv;
  }

  // Anything that doesn't start with "--" is a positional argument.
  if (arg.size() < 2u || arg[0] != '-' || arg[1] != '-') {
    bool rv = positional_args_.empty();
    started_positional_args_ = true;
    positional_args_.push_back(arg);
    return rv;
  }

  // "--" ends option processing, but isn't stored as a positional argument.
  if (arg.size() == 2u) {
    started_positional_args_ = true;
    return false;
  }

  // Note: The option name *must* be at least one character, so start at
  // position 3 -- "--=foo" will yield a name of "=foo" and no value. (Passing a
  // starting |pos| that's "too big" is OK.)
  size_t equals_pos = arg.find('=', 3u);
  if (equals_pos == std::string::npos) {
    options_.push_back(CommandLine::Option(arg.substr(2u)));
    return false;
  }

  options_.push_back(CommandLine::Option(arg.substr(2u, equals_pos - 2u),
                                         arg.substr(equals_pos + 1u)));
  return false;
}

CommandLine CommandLineBuilder::Build() const {
  if (!has_argv0_) {
    return CommandLine();
  }
  return CommandLine(argv0_, options_, positional_args_);
}

}  // namespace internal

std::vector<std::string> CommandLineToArgv(const CommandLine& command_line) {
  if (!command_line.has_argv0()) {
    return std::vector<std::string>();
  }

  std::vector<std::string> argv;
  const std::vector<CommandLine::Option>& options = command_line.options();
  const std::vector<std::string>& positional_args =
      command_line.positional_args();
  // Reserve space for argv[0], options, maybe a "--" (if needed), and the
  // positional arguments.
  argv.reserve(1u + options.size() + 1u + positional_args.size());

  argv.push_back(command_line.argv0());
  for (const auto& option : options) {
    if (option.value.empty()) {
      argv.push_back("--" + option.name);
    } else {
      argv.push_back("--" + option.name + "=" + option.value);
    }
  }

  if (!positional_args.empty()) {
    // Insert a "--" if necessary.
    if (positional_args[0].size() >= 2u && positional_args[0][0] == '-' &&
        positional_args[0][1] == '-') {
      argv.push_back("--");
    }

    argv.insert(argv.end(), positional_args.begin(), positional_args.end());
  }

  return argv;
}

}  // namespace fml
