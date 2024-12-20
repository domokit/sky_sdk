// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/fml/command_line.h"

#include <windows.h>

#include <Shellapi.h>
#include <memory>

namespace fml {

std::optional<CommandLine> CommandLineFromPlatform() {
  wchar_t* command_line = GetCommandLineW();
  int unicode_argc;
  std::unique_ptr<wchar_t*[], decltype(::LocalFree)*> unicode_argv(
      CommandLineToArgvW(command_line, &unicode_argc), ::LocalFree);
  if (!unicode_argv) {
    return std::nullopt;
  }
  std::vector<std::string> utf8_argv;
  for (int i = 0; i < unicode_argc; ++i) {
    wchar_t* arg = unicode_argv[i];
    int arg_len = WideCharToMultiByte(CP_UTF8, 0, arg, wcslen(arg), nullptr, 0,
                                      nullptr, nullptr);
    std::string utf8_arg(arg_len, 0);
    WideCharToMultiByte(CP_UTF8, 0, arg, -1, utf8_arg.data(), utf8_arg.size(),
                        nullptr, nullptr);
    utf8_argv.push_back(std::move(utf8_arg));
  }
  return CommandLineFromIterators(utf8_argv.begin(), utf8_argv.end());
}

}  // namespace fml
