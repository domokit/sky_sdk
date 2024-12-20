// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/glfw/system_utils.h"

#include <cstdlib>
#include <sstream>

namespace flutter {

namespace {

const char* GetLocaleStringFromEnvironment() {
  const char* retval;
  retval = getenv("LANGUAGE");
  if ((retval != NULL) && (retval[0] != '\0')) {
    return retval;
  }
  retval = getenv("LC_ALL");
  if ((retval != NULL) && (retval[0] != '\0')) {
    return retval;
  }
  retval = getenv("LC_MESSAGES");
  if ((retval != NULL) && (retval[0] != '\0')) {
    return retval;
  }
  retval = getenv("LANG");
  if ((retval != NULL) && (retval[0] != '\0')) {
    return retval;
  }

  return NULL;
}

// The least specific to most specific components of a locale.
enum Component {
  kCodeset = 1 << 0,
  kTerritory = 1 << 1,
  kModifier = 1 << 2,
};

// Construct a mask indicating which of the components in |info| are set.
int ComputeVariantMask(const LanguageInfo& info) {
  int mask = 0;
  if (!info.territory.empty()) {
    mask |= kTerritory;
  }
  if (!info.codeset.empty()) {
    mask |= kCodeset;
  }
  if (!info.modifier.empty()) {
    mask |= kModifier;
  }
  return mask;
}

// Appends most specific to least specific variants of |info| to |languages|.
// For example, "de_DE@euro" would append "de_DE@euro", "de@euro", "de_DE",
// and "de".
void AppendLocaleVariants(std::vector<LanguageInfo>& languages,
                          const LanguageInfo& info) {
  int mask = ComputeVariantMask(info);
  for (int i = mask; i >= 0; --i) {
    if ((i & ~mask) == 0) {
      LanguageInfo variant;
      variant.language = info.language;

      if (i & kTerritory) {
        variant.territory = info.territory;
      }
      if (i & kCodeset) {
        variant.codeset = info.codeset;
      }
      if (i & kModifier) {
        variant.modifier = info.modifier;
      }
      languages.push_back(variant);
    }
  }
}

// Parses a locale into its components.
LanguageInfo ParseLocale(const std::string& locale) {
  // Locales are of the form "language[_territory][.codeset][@modifier]"
  LanguageInfo result;
  std::string::size_type end = locale.size();
  std::string::size_type modifier_pos = locale.rfind('@');
  if (modifier_pos != std::string::npos) {
    result.modifier = locale.substr(modifier_pos + 1, end - modifier_pos - 1);
    end = modifier_pos;
  }

  std::string::size_type codeset_pos = locale.rfind('.', end);
  if (codeset_pos != std::string::npos) {
    result.codeset = locale.substr(codeset_pos + 1, end - codeset_pos - 1);
    end = codeset_pos;
  }

  std::string::size_type territory_pos = locale.rfind('_', end);
  if (territory_pos != std::string::npos) {
    result.territory =
        locale.substr(territory_pos + 1, end - territory_pos - 1);
    end = territory_pos;
  }

  result.language = locale.substr(0, end);

  return result;
}

}  // namespace

std::vector<LanguageInfo> GetPreferredLanguageInfo() {
  const char* locale_string;
  locale_string = GetLocaleStringFromEnvironment();
  if (!locale_string || locale_string[0] == '\0') {
    // This is the default locale if none is specified according to ISO C.
    locale_string = "C";
  }
  std::istringstream locales_stream(locale_string);
  std::vector<LanguageInfo> languages;
  std::string s;
  while (getline(locales_stream, s, ':')) {
    LanguageInfo info = ParseLocale(s);
    AppendLocaleVariants(languages, info);
  }
  return languages;
}

std::vector<FlutterLocale> ConvertToFlutterLocale(
    const std::vector<LanguageInfo>& languages) {
  std::vector<FlutterLocale> flutter_locales;
  flutter_locales.reserve(languages.size());
  for (const auto& info : languages) {
    FlutterLocale locale = {};
    locale.struct_size = sizeof(FlutterLocale);
    locale.language_code = info.language.c_str();
    if (!info.territory.empty()) {
      locale.country_code = info.territory.c_str();
    }
    if (!info.codeset.empty()) {
      locale.script_code = info.codeset.c_str();
    }
    if (!info.modifier.empty()) {
      locale.variant_code = info.modifier.c_str();
    }
    flutter_locales.push_back(locale);
  }

  return flutter_locales;
}

}  // namespace flutter
