// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <cstring>
#include <cwchar>

#include "flutter/fml/macros.h"
#include "flutter/shell/platform/windows/system_utils.h"
#include "flutter/shell/platform/windows/testing/mock_windows_proc_table.h"
#include "gtest/gtest.h"

namespace flutter {
namespace testing {

TEST(SystemUtils, GetPreferredLanguageInfo) {
  WindowsProcTable proc_table;
  std::vector<LanguageInfo> languages =
      GetPreferredLanguageInfo(WindowsProcTable());
  // There should be at least one language.
  ASSERT_GE(languages.size(), 1);
  // The info should have a valid languge.
  EXPECT_GE(languages[0].language.size(), 2);
}

TEST(SystemUtils, GetPreferredLanguages) {
  MockWindowsProcTable proc_table;
  EXPECT_CALL(proc_table, GetThreadPreferredUILanguages)
      .WillRepeatedly(
          [](DWORD flags, PULONG count, PZZWSTR languages, PULONG size) {
            // Languages string ends in a double-null.
            static const wchar_t lang[] = L"en-US\0";
            static const size_t lang_len = sizeof(lang) / sizeof(wchar_t);
            static const int cnt = 1;
            if (languages == nullptr) {
              *size = lang_len;
              *count = cnt;
            } else if (*size >= lang_len) {
              memcpy(languages, lang, lang_len * sizeof(wchar_t));
            }
            return TRUE;
          });
  std::vector<std::wstring> languages = GetPreferredLanguages(proc_table);
  // There should be at least one language.
  ASSERT_GE(languages.size(), 1);
  // The language should be non-empty.
  EXPECT_FALSE(languages[0].empty());
  // There should not be a trailing null from the parsing step.
  EXPECT_EQ(languages[0].size(), wcslen(languages[0].c_str()));
  EXPECT_EQ(languages[0], L"en-US");
}

TEST(SystemUtils, ParseLanguageNameGeneric) {
  LanguageInfo info = ParseLanguageName(L"en");
  EXPECT_EQ(info.language, "en");
  EXPECT_TRUE(info.region.empty());
  EXPECT_TRUE(info.script.empty());
}

TEST(SystemUtils, ParseLanguageNameWithRegion) {
  LanguageInfo info = ParseLanguageName(L"hu-HU");
  EXPECT_EQ(info.language, "hu");
  EXPECT_EQ(info.region, "HU");
  EXPECT_TRUE(info.script.empty());
}

TEST(SystemUtils, ParseLanguageNameWithScript) {
  LanguageInfo info = ParseLanguageName(L"us-Latn");
  EXPECT_EQ(info.language, "us");
  EXPECT_TRUE(info.region.empty());
  EXPECT_EQ(info.script, "Latn");
}

TEST(SystemUtils, ParseLanguageNameWithRegionAndScript) {
  LanguageInfo info = ParseLanguageName(L"uz-Latn-UZ");
  EXPECT_EQ(info.language, "uz");
  EXPECT_EQ(info.region, "UZ");
  EXPECT_EQ(info.script, "Latn");
}

TEST(SystemUtils, ParseLanguageNameWithSuplementalLanguage) {
  LanguageInfo info = ParseLanguageName(L"en-US-x-fabricam");
  EXPECT_EQ(info.language, "en");
  EXPECT_EQ(info.region, "US");
  EXPECT_TRUE(info.script.empty());
}

// Ensure that ISO 639-2/T codes are handled.
TEST(SystemUtils, ParseLanguageNameWithThreeCharacterLanguage) {
  LanguageInfo info = ParseLanguageName(L"ale-ZZ");
  EXPECT_EQ(info.language, "ale");
  EXPECT_EQ(info.region, "ZZ");
  EXPECT_TRUE(info.script.empty());
}

TEST(SystemUtils, GetUserTimeFormat) {
  // The value varies based on machine; just ensure that something is returned.
  EXPECT_FALSE(GetUserTimeFormat().empty());
}

TEST(SystemUtils, Prefer24HourTimeHandlesEmptyFormat) {
  EXPECT_FALSE(Prefer24HourTime(L""));
}

TEST(SystemUtils, Prefer24HourTimeHandles12Hour) {
  EXPECT_FALSE(Prefer24HourTime(L"h:mm:ss tt"));
}

TEST(SystemUtils, Prefer24HourTimeHandles24Hour) {
  EXPECT_TRUE(Prefer24HourTime(L"HH:mm:ss"));
}

}  // namespace testing
}  // namespace flutter
