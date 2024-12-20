// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <gtest/gtest.h>

#include <cstdlib>

#include "runner.h"
#include "third_party/icu/source/i18n/unicode/timezone.h"

namespace flutter_runner {

TEST(RunnerTZDataTest, LoadsWithTZDataPresent) {
  // TODO(fxbug.dev/69570): Move to cml file if env_vars gains supported for the
  // gtest_runner.
  setenv("ICU_TIMEZONE_FILES_DIR", "/pkg/data/tzdata", true);

  UErrorCode err = U_ZERO_ERROR;
  const auto version_before = std::string(icu::TimeZone::getTZDataVersion(err));
  ASSERT_EQ(U_ZERO_ERROR, err) << "unicode error: " << u_errorName(err);

  // This loads the tzdata. In Fuchsia, we force the data from this package
  // to be version 2019a, so that we can test the resource load.
  bool success = Runner::SetupICUInternal();
  ASSERT_TRUE(success) << "failed to load timezone data";

  const auto version_after = std::string(icu::TimeZone::getTZDataVersion(err));
  ASSERT_EQ(U_ZERO_ERROR, err) << "unicode error: " << u_errorName(err);

  EXPECT_EQ("2019a", version_after);
}

}  // namespace flutter_runner
