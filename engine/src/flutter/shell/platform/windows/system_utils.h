// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// This file contains utilities for system-level information/settings.

#ifndef FLUTTER_SHELL_PLATFORM_WINDOWS_SYSTEM_UTILS_H_
#define FLUTTER_SHELL_PLATFORM_WINDOWS_SYSTEM_UTILS_H_

#include <string>
#include <vector>

#include "flutter/shell/platform/windows/windows_proc_table.h"

namespace flutter {

// Registry key for user-preferred languages.
constexpr const wchar_t kGetPreferredLanguageRegKey[] =
    L"Control panel\\International\\User Profile";
constexpr const wchar_t kGetPreferredLanguageRegValue[] = L"Languages";

// Components of a system language/locale.
struct LanguageInfo {
  std::string language;
  std::string region;
  std::string script;
};

// Returns the list of user-preferred languages, in preference order,
// parsed into LanguageInfo structures.
std::vector<LanguageInfo> GetPreferredLanguageInfo(
    const WindowsProcTable& windows_proc_table);

// Retrieve the preferred languages from the MUI API.
std::wstring GetPreferredLanguagesFromMUI(
    const WindowsProcTable& windows_proc_table);

// Returns the list of user-preferred languages, in preference order.
// The language names are as described at:
// https://docs.microsoft.com/en-us/windows/win32/intl/language-names
std::vector<std::wstring> GetPreferredLanguages(
    const WindowsProcTable& windows_proc_table);

// Parses a Windows language name into its components.
LanguageInfo ParseLanguageName(std::wstring language_name);

// Returns the user's system time format string.
std::wstring GetUserTimeFormat();

// Returns true if the time_format is set to use 24 hour time.
bool Prefer24HourTime(std::wstring time_format);

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_WINDOWS_SYSTEM_UTILS_H_
