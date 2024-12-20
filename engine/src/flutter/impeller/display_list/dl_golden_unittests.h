// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_IMPELLER_DISPLAY_LIST_DL_GOLDEN_UNITTESTS_H_
#define FLUTTER_IMPELLER_DISPLAY_LIST_DL_GOLDEN_UNITTESTS_H_

#include "impeller/display_list/dl_playground.h"
#include "impeller/golden_tests/golden_playground_test.h"

namespace flutter {
namespace testing {

#ifdef IMPELLER_GOLDEN_TESTS
using DlGoldenTest = impeller::GoldenPlaygroundTest;
#else
using DlGoldenTest = impeller::DlPlayground;
#endif

}  // namespace testing
}  // namespace flutter

#endif  // FLUTTER_IMPELLER_DISPLAY_LIST_DL_GOLDEN_UNITTESTS_H_
