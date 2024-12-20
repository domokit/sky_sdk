// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/glfw/system_utils.h"

#include <cstdlib>

#include "gtest/gtest.h"

namespace flutter {
namespace {

// This is a helper for setting up the different environment variables to
// specific strings, calling GetPreferredLanguageInfo, and then restoring those
// environment variables to any previously existing values.
std::vector<LanguageInfo> SetAndRestoreLanguageAroundGettingLanguageInfo(
    const char* language,
    const char* lc_all,
    const char* lc_messages,
    const char* lang) {
  std::vector<const char*> env_vars{
      "LANGUAGE",
      "LC_ALL",
      "LC_MESSAGES",
      "LANG",
  };
  std::map<const char*, const char*> new_values{
      {env_vars[0], language},
      {env_vars[1], lc_all},
      {env_vars[2], lc_messages},
      {env_vars[3], lang},
  };
  std::map<const char*, const char*> prior_values;
  for (auto var : env_vars) {
    const char* value = getenv(var);
    if (value != nullptr) {
      prior_values.emplace(var, value);
    }
    const char* new_value = new_values.at(var);
    if (new_value != nullptr) {
      setenv(var, new_value, 1);
    } else {
      unsetenv(var);
    }
  }

  std::vector<LanguageInfo> languages = GetPreferredLanguageInfo();

  for (auto [var, value] : prior_values) {
    setenv(var, value, 1);
  }

  return languages;
}

TEST(FlutterGlfwSystemUtilsTest, GetPreferredLanuageInfoFull) {
  const char* locale_string = "en_GB.ISO-8859-1@euro:en_US:sv:zh_CN.UTF-8";

  std::vector<LanguageInfo> languages =
      SetAndRestoreLanguageAroundGettingLanguageInfo(locale_string, nullptr,
                                                     nullptr, nullptr);

  EXPECT_EQ(languages.size(), 15UL);

  EXPECT_STREQ(languages[0].language.c_str(), "en");
  EXPECT_STREQ(languages[0].territory.c_str(), "GB");
  EXPECT_STREQ(languages[0].codeset.c_str(), "ISO-8859-1");
  EXPECT_STREQ(languages[0].modifier.c_str(), "euro");

  EXPECT_STREQ(languages[1].language.c_str(), "en");
  EXPECT_STREQ(languages[1].territory.c_str(), "GB");
  EXPECT_STREQ(languages[1].codeset.c_str(), "");
  EXPECT_STREQ(languages[1].modifier.c_str(), "euro");

  EXPECT_STREQ(languages[2].language.c_str(), "en");
  EXPECT_STREQ(languages[2].territory.c_str(), "");
  EXPECT_STREQ(languages[2].codeset.c_str(), "ISO-8859-1");
  EXPECT_STREQ(languages[2].modifier.c_str(), "euro");

  EXPECT_STREQ(languages[3].language.c_str(), "en");
  EXPECT_STREQ(languages[3].territory.c_str(), "");
  EXPECT_STREQ(languages[3].codeset.c_str(), "");
  EXPECT_STREQ(languages[3].modifier.c_str(), "euro");

  EXPECT_STREQ(languages[4].language.c_str(), "en");
  EXPECT_STREQ(languages[4].territory.c_str(), "GB");
  EXPECT_STREQ(languages[4].codeset.c_str(), "ISO-8859-1");
  EXPECT_STREQ(languages[4].modifier.c_str(), "");

  EXPECT_STREQ(languages[5].language.c_str(), "en");
  EXPECT_STREQ(languages[5].territory.c_str(), "GB");
  EXPECT_STREQ(languages[5].codeset.c_str(), "");
  EXPECT_STREQ(languages[5].modifier.c_str(), "");

  EXPECT_STREQ(languages[6].language.c_str(), "en");
  EXPECT_STREQ(languages[6].territory.c_str(), "");
  EXPECT_STREQ(languages[6].codeset.c_str(), "ISO-8859-1");
  EXPECT_STREQ(languages[6].modifier.c_str(), "");

  EXPECT_STREQ(languages[7].language.c_str(), "en");
  EXPECT_STREQ(languages[7].territory.c_str(), "");
  EXPECT_STREQ(languages[7].codeset.c_str(), "");
  EXPECT_STREQ(languages[7].modifier.c_str(), "");

  EXPECT_STREQ(languages[8].language.c_str(), "en");
  EXPECT_STREQ(languages[8].territory.c_str(), "US");
  EXPECT_STREQ(languages[8].codeset.c_str(), "");
  EXPECT_STREQ(languages[8].modifier.c_str(), "");

  EXPECT_STREQ(languages[9].language.c_str(), "en");
  EXPECT_STREQ(languages[9].territory.c_str(), "");
  EXPECT_STREQ(languages[9].codeset.c_str(), "");
  EXPECT_STREQ(languages[9].modifier.c_str(), "");

  EXPECT_STREQ(languages[10].language.c_str(), "sv");
  EXPECT_STREQ(languages[10].territory.c_str(), "");
  EXPECT_STREQ(languages[10].codeset.c_str(), "");
  EXPECT_STREQ(languages[10].modifier.c_str(), "");

  EXPECT_STREQ(languages[11].language.c_str(), "zh");
  EXPECT_STREQ(languages[11].territory.c_str(), "CN");
  EXPECT_STREQ(languages[11].codeset.c_str(), "UTF-8");
  EXPECT_STREQ(languages[11].modifier.c_str(), "");

  EXPECT_STREQ(languages[12].language.c_str(), "zh");
  EXPECT_STREQ(languages[12].territory.c_str(), "CN");
  EXPECT_STREQ(languages[12].codeset.c_str(), "");
  EXPECT_STREQ(languages[12].modifier.c_str(), "");

  EXPECT_STREQ(languages[13].language.c_str(), "zh");
  EXPECT_STREQ(languages[13].territory.c_str(), "");
  EXPECT_STREQ(languages[13].codeset.c_str(), "UTF-8");
  EXPECT_STREQ(languages[13].modifier.c_str(), "");

  EXPECT_STREQ(languages[14].language.c_str(), "zh");
  EXPECT_STREQ(languages[14].territory.c_str(), "");
  EXPECT_STREQ(languages[14].codeset.c_str(), "");
  EXPECT_STREQ(languages[14].modifier.c_str(), "");
}

TEST(FlutterGlfwSystemUtilsTest, GetPreferredLanuageInfoWeird) {
  const char* locale_string = "tt_RU@iqtelif.UTF-8";
  std::vector<LanguageInfo> languages =
      SetAndRestoreLanguageAroundGettingLanguageInfo(locale_string, nullptr,
                                                     nullptr, nullptr);

  EXPECT_EQ(languages.size(), 4UL);

  EXPECT_STREQ(languages[0].language.c_str(), "tt");
  EXPECT_STREQ(languages[0].territory.c_str(), "RU");
  EXPECT_STREQ(languages[0].codeset.c_str(), "");
  EXPECT_STREQ(languages[0].modifier.c_str(), "iqtelif.UTF-8");

  EXPECT_STREQ(languages[1].language.c_str(), "tt");
  EXPECT_STREQ(languages[1].territory.c_str(), "");
  EXPECT_STREQ(languages[1].codeset.c_str(), "");
  EXPECT_STREQ(languages[1].modifier.c_str(), "iqtelif.UTF-8");

  EXPECT_STREQ(languages[2].language.c_str(), "tt");
  EXPECT_STREQ(languages[2].territory.c_str(), "RU");
  EXPECT_STREQ(languages[2].codeset.c_str(), "");
  EXPECT_STREQ(languages[2].modifier.c_str(), "");

  EXPECT_STREQ(languages[3].language.c_str(), "tt");
  EXPECT_STREQ(languages[3].territory.c_str(), "");
  EXPECT_STREQ(languages[3].codeset.c_str(), "");
  EXPECT_STREQ(languages[3].modifier.c_str(), "");
}

TEST(FlutterGlfwSystemUtilsTest, GetPreferredLanuageInfoEmpty) {
  const char* locale_string = "";
  std::vector<LanguageInfo> languages =
      SetAndRestoreLanguageAroundGettingLanguageInfo(
          locale_string, locale_string, locale_string, locale_string);

  EXPECT_EQ(languages.size(), 1UL);

  EXPECT_STREQ(languages[0].language.c_str(), "C");
  EXPECT_TRUE(languages[0].territory.empty());
  EXPECT_TRUE(languages[0].codeset.empty());
  EXPECT_TRUE(languages[0].modifier.empty());
}

TEST(FlutterGlfwSystemUtilsTest, GetPreferredLanuageInfoEnvVariableOrdering1) {
  const char* language = "de";
  const char* lc_all = "en";
  const char* lc_messages = "zh";
  const char* lang = "tt";

  std::vector<LanguageInfo> languages =
      SetAndRestoreLanguageAroundGettingLanguageInfo(language, lc_all,
                                                     lc_messages, lang);

  EXPECT_EQ(languages.size(), 1UL);
  EXPECT_STREQ(languages[0].language.c_str(), language);
}

TEST(FlutterGlfwSystemUtilsTest, GetPreferredLanuageInfoEnvVariableOrdering2) {
  const char* lc_all = "en";
  const char* lc_messages = "zh";
  const char* lang = "tt";

  std::vector<LanguageInfo> languages =
      SetAndRestoreLanguageAroundGettingLanguageInfo(nullptr, lc_all,
                                                     lc_messages, lang);

  EXPECT_EQ(languages.size(), 1UL);
  EXPECT_STREQ(languages[0].language.c_str(), lc_all);
}

TEST(FlutterGlfwSystemUtilsTest, GetPreferredLanuageInfoEnvVariableOrdering3) {
  const char* lc_messages = "zh";
  const char* lang = "tt";

  std::vector<LanguageInfo> languages =
      SetAndRestoreLanguageAroundGettingLanguageInfo(nullptr, nullptr,
                                                     lc_messages, lang);

  EXPECT_EQ(languages.size(), 1UL);
  EXPECT_STREQ(languages[0].language.c_str(), lc_messages);
}

TEST(FlutterGlfwSystemUtilsTest, GetPreferredLanuageInfoEnvVariableOrdering4) {
  const char* lang = "tt";

  std::vector<LanguageInfo> languages =
      SetAndRestoreLanguageAroundGettingLanguageInfo(nullptr, nullptr, nullptr,
                                                     lang);

  EXPECT_EQ(languages.size(), 1UL);
  EXPECT_STREQ(languages[0].language.c_str(), lang);
}

TEST(FlutterGlfwSystemUtilsTest, GetPreferredLanuageInfoEnvVariableOrdering5) {
  std::vector<LanguageInfo> languages =
      SetAndRestoreLanguageAroundGettingLanguageInfo(nullptr, nullptr, nullptr,
                                                     nullptr);

  EXPECT_EQ(languages.size(), 1UL);
  EXPECT_STREQ(languages[0].language.c_str(), "C");
}

TEST(FlutterGlfwSystemUtilsTest, ConvertToFlutterLocaleEmpty) {
  std::vector<LanguageInfo> languages;

  std::vector<FlutterLocale> locales = ConvertToFlutterLocale(languages);

  EXPECT_TRUE(locales.empty());
}

TEST(FlutterGlfwSystemUtilsTest, ConvertToFlutterLocaleNonEmpty) {
  std::vector<LanguageInfo> languages;
  languages.push_back(LanguageInfo{"en", "US", "", ""});
  languages.push_back(LanguageInfo{"tt", "RU", "", "iqtelif.UTF-8"});
  languages.push_back(LanguageInfo{"sv", "", "", ""});
  languages.push_back(LanguageInfo{"de", "DE", "UTF-8", "euro"});
  languages.push_back(LanguageInfo{"zh", "CN", "UTF-8", ""});

  std::vector<FlutterLocale> locales = ConvertToFlutterLocale(languages);

  EXPECT_EQ(locales.size(), 5UL);

  EXPECT_EQ(locales[0].struct_size, sizeof(FlutterLocale));
  EXPECT_STREQ(locales[0].language_code, "en");
  EXPECT_STREQ(locales[0].country_code, "US");
  EXPECT_EQ(locales[0].script_code, nullptr);
  EXPECT_EQ(locales[0].variant_code, nullptr);

  EXPECT_STREQ(locales[1].language_code, "tt");
  EXPECT_STREQ(locales[1].country_code, "RU");
  EXPECT_EQ(locales[1].script_code, nullptr);
  EXPECT_STREQ(locales[1].variant_code, "iqtelif.UTF-8");

  EXPECT_STREQ(locales[2].language_code, "sv");
  EXPECT_EQ(locales[2].country_code, nullptr);
  EXPECT_EQ(locales[2].script_code, nullptr);
  EXPECT_EQ(locales[2].variant_code, nullptr);

  EXPECT_STREQ(locales[3].language_code, "de");
  EXPECT_STREQ(locales[3].country_code, "DE");
  EXPECT_STREQ(locales[3].script_code, "UTF-8");
  EXPECT_STREQ(locales[3].variant_code, "euro");

  EXPECT_STREQ(locales[4].language_code, "zh");
  EXPECT_STREQ(locales[4].country_code, "CN");
  EXPECT_STREQ(locales[4].script_code, "UTF-8");
  EXPECT_EQ(locales[4].variant_code, nullptr);
}

}  // namespace
}  // namespace flutter
