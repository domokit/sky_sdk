// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <fuchsia/intl/cpp/fidl.h>

#include "flutter/fml/icu_util.h"
#include "gtest/gtest.h"

#include "fuchsia_intl.h"

using fuchsia::intl::CalendarId;
using fuchsia::intl::LocaleId;
using fuchsia::intl::Profile;
using fuchsia::intl::TemperatureUnit;
using fuchsia::intl::TimeZoneId;

namespace flutter_runner {
namespace {

class FuchsiaIntlTest : public testing::Test {
 public:
  static void SetUpTestCase() {
    testing::Test::SetUpTestCase();
    // The icudtl data must be present as a resource in the package for this
    // load to succeed.
    fml::icu::InitializeICU("/pkg/data/icudtl.dat");
  }
};

TEST_F(FuchsiaIntlTest, MakeLocalizationPlatformMessageData_SimpleLocale) {
  Profile profile{};
  profile.set_locales({LocaleId{.id = "en-US"}});
  const std::string expected =
      R"({"method":"setLocale","args":["en","US","",""]})";
  const auto actual = MakeLocalizationPlatformMessageData(profile);
  ASSERT_EQ(expected, std::string(actual.GetMapping(),
                                  actual.GetMapping() + actual.GetSize()));
}

TEST_F(FuchsiaIntlTest, MakeLocalizationPlatformMessageData_OneLocale) {
  Profile profile{};
  profile
      .set_locales({LocaleId{.id = "en-US-u-ca-gregory-fw-sun-hc-h12-ms-"
                                   "ussystem-nu-latn-tz-usnyc"}})
      .set_calendars({CalendarId{.id = "und-u-gregory"}})
      .set_time_zones({TimeZoneId{.id = "America/New_York"}})
      .set_temperature_unit(TemperatureUnit::FAHRENHEIT);
  const std::string expected =
      R"({"method":"setLocale","args":["en","US","",""]})";
  const auto actual = MakeLocalizationPlatformMessageData(profile);
  ASSERT_EQ(expected, std::string(actual.GetMapping(),
                                  actual.GetMapping() + actual.GetSize()));
}

TEST_F(FuchsiaIntlTest, MakeLocalizationPlatformMessageData_MultipleLocales) {
  Profile profile{};
  profile
      .set_locales({LocaleId{.id = "en-US-u-ca-gregory-fw-sun-hc-h12-ms-"
                                   "ussystem-nu-latn-tz-usnyc"},
                    LocaleId{.id = "sl-Latn-IT-nedis"},
                    LocaleId{.id = "zh-Hans"}, LocaleId{.id = "sr-Cyrl-CS"}})
      .set_calendars({CalendarId{.id = "und-u-gregory"}})
      .set_time_zones({TimeZoneId{.id = "America/New_York"}})
      .set_temperature_unit(TemperatureUnit::FAHRENHEIT);
  const std::string expected =
      R"({"method":"setLocale","args":["en","US","","","sl","IT","Latn","nedis",)"
      R"("zh","","Hans","","sr","CS","Cyrl",""]})";
  const auto actual = MakeLocalizationPlatformMessageData(profile);
  ASSERT_EQ(expected, std::string(actual.GetMapping(),
                                  actual.GetMapping() + actual.GetSize()));
}

}  // namespace
}  // namespace flutter_runner
