// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_DISPLAY_LIST_TESTING_RMSE_H_
#define FLUTTER_IMPELLER_DISPLAY_LIST_TESTING_RMSE_H_

#include "flutter/impeller/golden_tests/screenshot.h"

namespace flutter {
namespace testing {
double RMSE(const impeller::testing::Screenshot* left,
            const impeller::testing::Screenshot* right);
}
}  // namespace flutter
#endif  // FLUTTER_IMPELLER_DISPLAY_LIST_TESTING_RMSE_H_
