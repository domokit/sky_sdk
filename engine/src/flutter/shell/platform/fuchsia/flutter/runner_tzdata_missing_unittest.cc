// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <gtest/gtest.h>

#include "runner.h"
#include "third_party/icu/source/i18n/unicode/timezone.h"

namespace flutter_runner {

// This test has not been configured with tzdata files.  This test shows that
// even without the data files, the runner continues initialization.  It will
// use whatever the base data exists in icudtl.dat.
TEST(RunnerTZDataTest, LoadsWithoutTZDataPresent) {
  bool success = Runner::SetupICUInternal();
  ASSERT_TRUE(success) << "failed to load set up ICU data";
}

}  // namespace flutter_runner
