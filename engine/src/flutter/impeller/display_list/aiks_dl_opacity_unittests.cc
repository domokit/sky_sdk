// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "display_list/dl_blend_mode.h"
#include "flutter/impeller/display_list/aiks_unittests.h"

#include "flutter/display_list/dl_builder.h"
#include "flutter/display_list/dl_color.h"
#include "flutter/display_list/dl_paint.h"
#include "flutter/testing/testing.h"
#include "include/core/SkRect.h"

namespace impeller {
namespace testing {

using namespace flutter;

TEST_P(AiksTest, DrawOpacityPeephole) {
  DisplayListBuilder builder;

  DlPaint green;
  green.setColor(DlColor::kGreen().modulateOpacity(0.5));

  DlPaint alpha;
  alpha.setColor(DlColor::kRed().modulateOpacity(0.5));

  builder.SaveLayer(nullptr, &alpha);
  builder.DrawRect(SkRect::MakeXYWH(0, 0, 100, 100), green);
  builder.Restore();

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, CanRenderGroupOpacity) {
  DisplayListBuilder builder;

  DlPaint red;
  red.setColor(DlColor::kRed());
  DlPaint green;
  green.setColor(DlColor::kGreen().modulateOpacity(0.5));
  DlPaint blue;
  blue.setColor(DlColor::kBlue());

  DlPaint alpha;
  alpha.setColor(DlColor::kRed().modulateOpacity(0.5));

  builder.SaveLayer(nullptr, &alpha);
  builder.DrawRect(SkRect::MakeXYWH(0, 0, 100, 100), red);
  builder.DrawRect(SkRect::MakeXYWH(200, 200, 100, 100), green);
  builder.DrawRect(SkRect::MakeXYWH(400, 400, 100, 100), blue);
  builder.Restore();

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

TEST_P(AiksTest, CanRenderGroupOpacityToSavelayer) {
  DisplayListBuilder builder;

  DlPaint red;
  red.setColor(DlColor::kRed());

  DlPaint alpha;
  alpha.setColor(DlColor::kRed().modulateOpacity(0.7));

  // Create a saveLayer that will forward its opacity to another
  // saveLayer, to verify that we correctly distribute opacity.
  SkRect bounds = SkRect::MakeLTRB(0, 0, 500, 500);
  builder.SaveLayer(&bounds, &alpha);
  builder.SaveLayer(&bounds, &alpha);
  builder.DrawRect(SkRect::MakeXYWH(0, 0, 400, 400), red);
  builder.DrawRect(SkRect::MakeXYWH(0, 0, 450, 450), red);
  builder.Restore();
  builder.Restore();

  ASSERT_TRUE(OpenPlaygroundHere(builder.Build()));
}

}  // namespace testing
}  // namespace impeller
