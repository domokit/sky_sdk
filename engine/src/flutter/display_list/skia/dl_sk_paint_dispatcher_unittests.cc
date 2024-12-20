// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/display_list/effects/dl_color_source.h"
#include "flutter/display_list/skia/dl_sk_paint_dispatcher.h"

#include "flutter/display_list/skia/dl_sk_dispatcher.h"
#include "flutter/display_list/testing/dl_test_snippets.h"
#include "flutter/display_list/utils/dl_receiver_utils.h"
#include "gtest/gtest.h"

namespace flutter {
namespace testing {

class MockDispatchHelper final : public virtual DlOpReceiver,
                                 public DlSkPaintDispatchHelper,
                                 public IgnoreClipDispatchHelper,
                                 public IgnoreTransformDispatchHelper,
                                 public IgnoreDrawDispatchHelper {
 public:
  void save() override { DlSkPaintDispatchHelper::save_opacity(0.5f); }

  void restore() override { DlSkPaintDispatchHelper::restore_opacity(); }
};

static const DlColor kTestColors[2] = {DlColor(0xFF000000),
                                       DlColor(0xFFFFFFFF)};
static const float kTestStops[2] = {0.0f, 1.0f};
static const auto kTestLinearGradient =
    DlColorSource::MakeLinear(DlPoint(0.0f, 0.0f),
                              DlPoint(100.0f, 100.0f),
                              2,
                              kTestColors,
                              kTestStops,
                              DlTileMode::kClamp,
                              nullptr);

// Regression test for https://github.com/flutter/flutter/issues/100176.
TEST(DisplayListUtils, OverRestore) {
  MockDispatchHelper helper;
  helper.save();
  helper.restore();
  // There should be a protection here for over-restore to keep the program from
  // crashing.
  helper.restore();
}

// https://github.com/flutter/flutter/issues/132860.
TEST(DisplayListUtils, SetColorSourceDithersIfGradient) {
  MockDispatchHelper helper;

  helper.setColorSource(kTestLinearGradient.get());
  EXPECT_TRUE(helper.paint(true).isDither());
  EXPECT_FALSE(helper.paint(false).isDither());
}

// https://github.com/flutter/flutter/issues/132860.
TEST(DisplayListUtils, SetColorSourceDoesNotDitherIfNotGradient) {
  MockDispatchHelper helper;

  helper.setColorSource(kTestLinearGradient.get());
  helper.setColorSource(nullptr);
  EXPECT_FALSE(helper.paint(true).isDither());
  EXPECT_FALSE(helper.paint(false).isDither());

  helper.setColorSource(kTestSource1.get());
  EXPECT_FALSE(helper.paint(true).isDither());
  EXPECT_FALSE(helper.paint(false).isDither());
}

// https://github.com/flutter/flutter/issues/132860.
TEST(DisplayListUtils, SkDispatcherSetColorSourceDithersIfGradient) {
  SkCanvas canvas;
  DlSkCanvasDispatcher dispatcher(&canvas);

  dispatcher.setColorSource(kTestLinearGradient.get());
  EXPECT_TRUE(dispatcher.paint(true).isDither());
  EXPECT_FALSE(dispatcher.paint(false).isDither());
  EXPECT_FALSE(dispatcher.safe_paint(true)->isDither());
  // Calling safe_paint(false) returns a nullptr
}

// https://github.com/flutter/flutter/issues/132860.
TEST(DisplayListUtils, SkDispatcherSetColorSourceDoesNotDitherIfNotGradient) {
  SkCanvas canvas;
  DlSkCanvasDispatcher dispatcher(&canvas);

  dispatcher.setColorSource(kTestLinearGradient.get());
  dispatcher.setColorSource(nullptr);
  EXPECT_FALSE(dispatcher.paint(true).isDither());
  EXPECT_FALSE(dispatcher.paint(false).isDither());
  EXPECT_FALSE(dispatcher.safe_paint(true)->isDither());
  // Calling safe_paint(false) returns a nullptr

  dispatcher.setColorSource(kTestSource1.get());
  EXPECT_FALSE(dispatcher.paint(true).isDither());
  EXPECT_FALSE(dispatcher.paint(false).isDither());
  EXPECT_FALSE(dispatcher.safe_paint(true)->isDither());
  // Calling safe_paint(false) returns a nullptr
}

}  // namespace testing
}  // namespace flutter
