// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/display_list/geometry/dl_geometry_types.h"
#include "gtest/gtest.h"

namespace flutter {
namespace testing {

TEST(DisplayListGeometryTypes, PointConversion) {
  SkPoint sk_p = SkPoint::Make(1.0f, 2.0f);
  DlPoint dl_p = DlPoint(1.0f, 2.0f);

  EXPECT_EQ(sk_p, ToSkPoint(dl_p));
  EXPECT_EQ(ToDlPoint(sk_p), dl_p);

  sk_p = SkPoint::Make(1.0f, 2.0f);
  dl_p = DlPoint(1.0f, 3.0f);

  EXPECT_NE(sk_p, ToSkPoint(dl_p));
  EXPECT_NE(ToDlPoint(sk_p), dl_p);
}

TEST(DisplayListGeometryTypes, RectConversion) {
  SkRect sk_r = SkRect::MakeLTRB(1.0f, 2.0f, 3.0f, 4.0f);
  DlRect dl_r = DlRect::MakeLTRB(1.0f, 2.0f, 3.0f, 4.0f);

  EXPECT_EQ(sk_r, ToSkRect(dl_r));
  EXPECT_EQ(ToDlRect(sk_r), dl_r);

  sk_r = SkRect::MakeLTRB(1.0f, 2.0f, 3.0f, 4.0f);
  dl_r = DlRect::MakeLTRB(1.0f, 2.0f, 3.0f, 5.0f);

  EXPECT_NE(sk_r, ToSkRect(dl_r));
  EXPECT_NE(ToDlRect(sk_r), dl_r);
}

TEST(DisplayListGeometryTypes, ISizeConversion) {
  SkISize sk_s = SkISize::Make(1.0f, 2.0f);
  DlISize dl_s = DlISize(1.0f, 2.0f);

  EXPECT_EQ(sk_s, ToSkISize(dl_s));
  EXPECT_EQ(ToDlISize(sk_s), dl_s);

  sk_s = SkISize::Make(1.0f, 2.0f);
  dl_s = DlISize(1.0f, 3.0f);

  EXPECT_NE(sk_s, ToSkISize(dl_s));
  EXPECT_NE(ToDlISize(sk_s), dl_s);
}

TEST(DisplayListGeometryTypes, VectorToSizeConversion) {
  SkVector sk_v = SkVector::Make(1.0f, 2.0f);
  DlSize dl_s = DlSize(1.0f, 2.0f);

  EXPECT_EQ(sk_v, ToSkVector(dl_s));
  EXPECT_EQ(ToDlSize(sk_v), dl_s);

  dl_s = DlSize(1.0f, 3.0f);

  EXPECT_NE(sk_v, ToSkVector(dl_s));
  EXPECT_NE(ToDlSize(sk_v), dl_s);

  dl_s = DlSize(3.0f, 2.0f);

  EXPECT_NE(sk_v, ToSkVector(dl_s));
  EXPECT_NE(ToDlSize(sk_v), dl_s);
}

}  // namespace testing
}  // namespace flutter
