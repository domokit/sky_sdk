// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "gtest/gtest.h"

#include "flutter/impeller/geometry/round_rect.h"

#include "flutter/impeller/geometry/geometry_asserts.h"

namespace impeller {
namespace testing {

TEST(RoundRectTest, RoundingRadiiEmptyDeclaration) {
  RoundingRadii radii;

  EXPECT_TRUE(radii.AreAllCornersEmpty());
  EXPECT_TRUE(radii.AreAllCornersSame());
  EXPECT_TRUE(radii.IsFinite());
  EXPECT_EQ(radii.top_left, Size());
  EXPECT_EQ(radii.top_right, Size());
  EXPECT_EQ(radii.bottom_left, Size());
  EXPECT_EQ(radii.bottom_right, Size());
  EXPECT_EQ(radii.top_left.width, 0.0f);
  EXPECT_EQ(radii.top_left.height, 0.0f);
  EXPECT_EQ(radii.top_right.width, 0.0f);
  EXPECT_EQ(radii.top_right.height, 0.0f);
  EXPECT_EQ(radii.bottom_left.width, 0.0f);
  EXPECT_EQ(radii.bottom_left.height, 0.0f);
  EXPECT_EQ(radii.bottom_right.width, 0.0f);
  EXPECT_EQ(radii.bottom_right.height, 0.0f);
}

TEST(RoundRectTest, RoundingRadiiDefaultConstructor) {
  RoundingRadii radii = RoundingRadii();

  EXPECT_TRUE(radii.AreAllCornersEmpty());
  EXPECT_TRUE(radii.AreAllCornersSame());
  EXPECT_TRUE(radii.IsFinite());
  EXPECT_EQ(radii.top_left, Size());
  EXPECT_EQ(radii.top_right, Size());
  EXPECT_EQ(radii.bottom_left, Size());
  EXPECT_EQ(radii.bottom_right, Size());
}

TEST(RoundRectTest, RoundingRadiiScalarConstructor) {
  RoundingRadii radii = RoundingRadii::MakeRadius(5.0f);

  EXPECT_FALSE(radii.AreAllCornersEmpty());
  EXPECT_TRUE(radii.AreAllCornersSame());
  EXPECT_TRUE(radii.IsFinite());
  EXPECT_EQ(radii.top_left, Size(5.0f, 5.0f));
  EXPECT_EQ(radii.top_right, Size(5.0f, 5.0f));
  EXPECT_EQ(radii.bottom_left, Size(5.0f, 5.0f));
  EXPECT_EQ(radii.bottom_right, Size(5.0f, 5.0f));
}

TEST(RoundRectTest, RoundingRadiiEmptyScalarConstructor) {
  RoundingRadii radii = RoundingRadii::MakeRadius(-5.0f);

  EXPECT_TRUE(radii.AreAllCornersEmpty());
  EXPECT_TRUE(radii.AreAllCornersSame());
  EXPECT_TRUE(radii.IsFinite());
  EXPECT_EQ(radii.top_left, Size(-5.0f, -5.0f));
  EXPECT_EQ(radii.top_right, Size(-5.0f, -5.0f));
  EXPECT_EQ(radii.bottom_left, Size(-5.0f, -5.0f));
  EXPECT_EQ(radii.bottom_right, Size(-5.0f, -5.0f));
}

TEST(RoundRectTest, RoundingRadiiSizeConstructor) {
  RoundingRadii radii = RoundingRadii::MakeRadii(Size(5.0f, 6.0f));

  EXPECT_FALSE(radii.AreAllCornersEmpty());
  EXPECT_TRUE(radii.AreAllCornersSame());
  EXPECT_TRUE(radii.IsFinite());
  EXPECT_EQ(radii.top_left, Size(5.0f, 6.0f));
  EXPECT_EQ(radii.top_right, Size(5.0f, 6.0f));
  EXPECT_EQ(radii.bottom_left, Size(5.0f, 6.0f));
  EXPECT_EQ(radii.bottom_right, Size(5.0f, 6.0f));
}

TEST(RoundRectTest, RoundingRadiiEmptySizeConstructor) {
  {
    RoundingRadii radii = RoundingRadii::MakeRadii(Size(-5.0f, 6.0f));

    EXPECT_TRUE(radii.AreAllCornersEmpty());
    EXPECT_TRUE(radii.AreAllCornersSame());
    EXPECT_TRUE(radii.IsFinite());
    EXPECT_EQ(radii.top_left, Size(-5.0f, 6.0f));
    EXPECT_EQ(radii.top_right, Size(-5.0f, 6.0f));
    EXPECT_EQ(radii.bottom_left, Size(-5.0f, 6.0f));
    EXPECT_EQ(radii.bottom_right, Size(-5.0f, 6.0f));
  }

  {
    RoundingRadii radii = RoundingRadii::MakeRadii(Size(5.0f, -6.0f));

    EXPECT_TRUE(radii.AreAllCornersEmpty());
    EXPECT_TRUE(radii.AreAllCornersSame());
    EXPECT_TRUE(radii.IsFinite());
    EXPECT_EQ(radii.top_left, Size(5.0f, -6.0f));
    EXPECT_EQ(radii.top_right, Size(5.0f, -6.0f));
    EXPECT_EQ(radii.bottom_left, Size(5.0f, -6.0f));
    EXPECT_EQ(radii.bottom_right, Size(5.0f, -6.0f));
  }
}

TEST(RoundRectTest, RoundingRadiiNamedSizesConstructor) {
  RoundingRadii radii = {
      .top_left = Size(5.0f, 5.5f),
      .top_right = Size(6.0f, 6.5f),
      .bottom_left = Size(7.0f, 7.5f),
      .bottom_right = Size(8.0f, 8.5f),
  };

  EXPECT_FALSE(radii.AreAllCornersEmpty());
  EXPECT_FALSE(radii.AreAllCornersSame());
  EXPECT_TRUE(radii.IsFinite());
  EXPECT_EQ(radii.top_left, Size(5.0f, 5.5f));
  EXPECT_EQ(radii.top_right, Size(6.0f, 6.5f));
  EXPECT_EQ(radii.bottom_left, Size(7.0f, 7.5f));
  EXPECT_EQ(radii.bottom_right, Size(8.0f, 8.5f));
}

TEST(RoundRectTest, RoundingRadiiPartialNamedSizesConstructor) {
  {
    RoundingRadii radii = {
        .top_left = Size(5.0f, 5.5f),
    };

    EXPECT_FALSE(radii.AreAllCornersEmpty());
    EXPECT_FALSE(radii.AreAllCornersSame());
    EXPECT_TRUE(radii.IsFinite());
    EXPECT_EQ(radii.top_left, Size(5.0f, 5.5f));
    EXPECT_EQ(radii.top_right, Size());
    EXPECT_EQ(radii.bottom_left, Size());
    EXPECT_EQ(radii.bottom_right, Size());
  }

  {
    RoundingRadii radii = {
        .top_right = Size(6.0f, 6.5f),
    };

    EXPECT_FALSE(radii.AreAllCornersEmpty());
    EXPECT_FALSE(radii.AreAllCornersSame());
    EXPECT_TRUE(radii.IsFinite());
    EXPECT_EQ(radii.top_left, Size());
    EXPECT_EQ(radii.top_right, Size(6.0f, 6.5f));
    EXPECT_EQ(radii.bottom_left, Size());
    EXPECT_EQ(radii.bottom_right, Size());
  }

  {
    RoundingRadii radii = {
        .bottom_left = Size(7.0f, 7.5f),
    };

    EXPECT_FALSE(radii.AreAllCornersEmpty());
    EXPECT_FALSE(radii.AreAllCornersSame());
    EXPECT_TRUE(radii.IsFinite());
    EXPECT_EQ(radii.top_left, Size());
    EXPECT_EQ(radii.top_right, Size());
    EXPECT_EQ(radii.bottom_left, Size(7.0f, 7.5f));
    EXPECT_EQ(radii.bottom_right, Size());
  }

  {
    RoundingRadii radii = {
        .bottom_right = Size(8.0f, 8.5f),
    };

    EXPECT_FALSE(radii.AreAllCornersEmpty());
    EXPECT_FALSE(radii.AreAllCornersSame());
    EXPECT_TRUE(radii.IsFinite());
    EXPECT_EQ(radii.top_left, Size());
    EXPECT_EQ(radii.top_right, Size());
    EXPECT_EQ(radii.bottom_left, Size());
    EXPECT_EQ(radii.bottom_right, Size(8.0f, 8.5f));
  }
}

TEST(RoundRectTest, RoundingRadiiMultiply) {
  RoundingRadii radii = {
      .top_left = Size(5.0f, 5.5f),
      .top_right = Size(6.0f, 6.5f),
      .bottom_left = Size(7.0f, 7.5f),
      .bottom_right = Size(8.0f, 8.5f),
  };
  RoundingRadii doubled = radii * 2.0f;

  EXPECT_FALSE(doubled.AreAllCornersEmpty());
  EXPECT_FALSE(doubled.AreAllCornersSame());
  EXPECT_TRUE(doubled.IsFinite());
  EXPECT_EQ(doubled.top_left, Size(10.0f, 11.0f));
  EXPECT_EQ(doubled.top_right, Size(12.0f, 13.0f));
  EXPECT_EQ(doubled.bottom_left, Size(14.0f, 15.0f));
  EXPECT_EQ(doubled.bottom_right, Size(16.0f, 17.0f));
}

TEST(RoundRectTest, RoundingRadiiEquals) {
  RoundingRadii radii = {
      .top_left = Size(5.0f, 5.5f),
      .top_right = Size(6.0f, 6.5f),
      .bottom_left = Size(7.0f, 7.5f),
      .bottom_right = Size(8.0f, 8.5f),
  };
  RoundingRadii other = {
      .top_left = Size(5.0f, 5.5f),
      .top_right = Size(6.0f, 6.5f),
      .bottom_left = Size(7.0f, 7.5f),
      .bottom_right = Size(8.0f, 8.5f),
  };

  EXPECT_EQ(radii, other);
}

TEST(RoundRectTest, RoundingRadiiNotEquals) {
  const RoundingRadii radii = {
      .top_left = Size(5.0f, 5.5f),
      .top_right = Size(6.0f, 6.5f),
      .bottom_left = Size(7.0f, 7.5f),
      .bottom_right = Size(8.0f, 8.5f),
  };

  {
    RoundingRadii different = radii;
    different.top_left.width = 100.0f;
    EXPECT_NE(different, radii);
  }
  {
    RoundingRadii different = radii;
    different.top_left.height = 100.0f;
    EXPECT_NE(different, radii);
  }
  {
    RoundingRadii different = radii;
    different.top_right.width = 100.0f;
    EXPECT_NE(different, radii);
  }
  {
    RoundingRadii different = radii;
    different.top_right.height = 100.0f;
    EXPECT_NE(different, radii);
  }
  {
    RoundingRadii different = radii;
    different.bottom_left.width = 100.0f;
    EXPECT_NE(different, radii);
  }
  {
    RoundingRadii different = radii;
    different.bottom_left.height = 100.0f;
    EXPECT_NE(different, radii);
  }
  {
    RoundingRadii different = radii;
    different.bottom_right.width = 100.0f;
    EXPECT_NE(different, radii);
  }
  {
    RoundingRadii different = radii;
    different.bottom_right.height = 100.0f;
    EXPECT_NE(different, radii);
  }
}

TEST(RoundRectTest, RoundingRadiiCornersSameTolerance) {
  RoundingRadii radii{
      .top_left = {10, 20},
      .top_right = {10.01, 20.01},
      .bottom_left = {9.99, 19.99},
      .bottom_right = {9.99, 20.01},
  };

  EXPECT_TRUE(radii.AreAllCornersSame(.02));

  {
    RoundingRadii different = radii;
    different.top_left.width = 10.03;
    EXPECT_FALSE(different.AreAllCornersSame(.02));
  }
  {
    RoundingRadii different = radii;
    different.top_left.height = 20.03;
    EXPECT_FALSE(different.AreAllCornersSame(.02));
  }
  {
    RoundingRadii different = radii;
    different.top_right.width = 10.03;
    EXPECT_FALSE(different.AreAllCornersSame(.02));
  }
  {
    RoundingRadii different = radii;
    different.top_right.height = 20.03;
    EXPECT_FALSE(different.AreAllCornersSame(.02));
  }
  {
    RoundingRadii different = radii;
    different.bottom_left.width = 9.97;
    EXPECT_FALSE(different.AreAllCornersSame(.02));
  }
  {
    RoundingRadii different = radii;
    different.bottom_left.height = 19.97;
    EXPECT_FALSE(different.AreAllCornersSame(.02));
  }
  {
    RoundingRadii different = radii;
    different.bottom_right.width = 9.97;
    EXPECT_FALSE(different.AreAllCornersSame(.02));
  }
  {
    RoundingRadii different = radii;
    different.bottom_right.height = 20.03;
    EXPECT_FALSE(different.AreAllCornersSame(.02));
  }
}

TEST(RoundRectTest, EmptyDeclaration) {
  RoundRect round_rect;

  EXPECT_TRUE(round_rect.IsEmpty());
  EXPECT_FALSE(round_rect.IsRect());
  EXPECT_FALSE(round_rect.IsOval());
  EXPECT_TRUE(round_rect.IsFinite());
  EXPECT_TRUE(round_rect.GetBounds().IsEmpty());
  EXPECT_EQ(round_rect.GetBounds(), Rect());
  EXPECT_EQ(round_rect.GetBounds().GetLeft(), 0.0f);
  EXPECT_EQ(round_rect.GetBounds().GetTop(), 0.0f);
  EXPECT_EQ(round_rect.GetBounds().GetRight(), 0.0f);
  EXPECT_EQ(round_rect.GetBounds().GetBottom(), 0.0f);
  EXPECT_EQ(round_rect.GetRadii().top_left, Size());
  EXPECT_EQ(round_rect.GetRadii().top_right, Size());
  EXPECT_EQ(round_rect.GetRadii().bottom_left, Size());
  EXPECT_EQ(round_rect.GetRadii().bottom_right, Size());
  EXPECT_EQ(round_rect.GetRadii().top_left.width, 0.0f);
  EXPECT_EQ(round_rect.GetRadii().top_left.height, 0.0f);
  EXPECT_EQ(round_rect.GetRadii().top_right.width, 0.0f);
  EXPECT_EQ(round_rect.GetRadii().top_right.height, 0.0f);
  EXPECT_EQ(round_rect.GetRadii().bottom_left.width, 0.0f);
  EXPECT_EQ(round_rect.GetRadii().bottom_left.height, 0.0f);
  EXPECT_EQ(round_rect.GetRadii().bottom_right.width, 0.0f);
  EXPECT_EQ(round_rect.GetRadii().bottom_right.height, 0.0f);
}

TEST(RoundRectTest, DefaultConstructor) {
  RoundRect round_rect = RoundRect();

  EXPECT_TRUE(round_rect.IsEmpty());
  EXPECT_FALSE(round_rect.IsRect());
  EXPECT_FALSE(round_rect.IsOval());
  EXPECT_TRUE(round_rect.IsFinite());
  EXPECT_TRUE(round_rect.GetBounds().IsEmpty());
  EXPECT_EQ(round_rect.GetBounds(), Rect());
  EXPECT_EQ(round_rect.GetRadii().top_left, Size());
  EXPECT_EQ(round_rect.GetRadii().top_right, Size());
  EXPECT_EQ(round_rect.GetRadii().bottom_left, Size());
  EXPECT_EQ(round_rect.GetRadii().bottom_right, Size());
}

TEST(RoundRectTest, EmptyRectConstruction) {
  RoundRect round_rect = RoundRect::MakeRectXY(
      Rect::MakeLTRB(20.0f, 20.0f, 10.0f, 10.0f), 10.0f, 10.0f);

  EXPECT_TRUE(round_rect.IsEmpty());
  EXPECT_FALSE(round_rect.IsRect());
  EXPECT_FALSE(round_rect.IsOval());
  EXPECT_TRUE(round_rect.IsFinite());
  EXPECT_TRUE(round_rect.GetBounds().IsEmpty());
  EXPECT_EQ(round_rect.GetBounds(), Rect::MakeLTRB(20.0f, 20.0f, 10.0f, 10.0f));
  EXPECT_EQ(round_rect.GetRadii().top_left, Size());
  EXPECT_EQ(round_rect.GetRadii().top_right, Size());
  EXPECT_EQ(round_rect.GetRadii().bottom_left, Size());
  EXPECT_EQ(round_rect.GetRadii().bottom_right, Size());
}

TEST(RoundRectTest, RectConstructor) {
  RoundRect round_rect =
      RoundRect::MakeRect(Rect::MakeLTRB(10.0f, 10.0f, 20.0f, 20.0f));

  EXPECT_FALSE(round_rect.IsEmpty());
  EXPECT_TRUE(round_rect.IsRect());
  EXPECT_FALSE(round_rect.IsOval());
  EXPECT_TRUE(round_rect.IsFinite());
  EXPECT_FALSE(round_rect.GetBounds().IsEmpty());
  EXPECT_EQ(round_rect.GetBounds(), Rect::MakeLTRB(10.0f, 10.0f, 20.0f, 20.0f));
  EXPECT_EQ(round_rect.GetRadii().top_left, Size());
  EXPECT_EQ(round_rect.GetRadii().top_right, Size());
  EXPECT_EQ(round_rect.GetRadii().bottom_left, Size());
  EXPECT_EQ(round_rect.GetRadii().bottom_right, Size());
}

TEST(RoundRectTest, OvalConstructor) {
  RoundRect round_rect =
      RoundRect::MakeOval(Rect::MakeLTRB(10.0f, 10.0f, 20.0f, 20.0f));

  EXPECT_FALSE(round_rect.IsEmpty());
  EXPECT_FALSE(round_rect.IsRect());
  EXPECT_TRUE(round_rect.IsOval());
  EXPECT_TRUE(round_rect.IsFinite());
  EXPECT_FALSE(round_rect.GetBounds().IsEmpty());
  EXPECT_EQ(round_rect.GetBounds(), Rect::MakeLTRB(10.0f, 10.0f, 20.0f, 20.0f));
  EXPECT_EQ(round_rect.GetRadii().top_left, Size(5.0f, 5.0f));
  EXPECT_EQ(round_rect.GetRadii().top_right, Size(5.0f, 5.0f));
  EXPECT_EQ(round_rect.GetRadii().bottom_left, Size(5.0f, 5.0f));
  EXPECT_EQ(round_rect.GetRadii().bottom_right, Size(5.0f, 5.0f));
}

TEST(RoundRectTest, RectRadiusConstructor) {
  RoundRect round_rect = RoundRect::MakeRectRadius(
      Rect::MakeLTRB(10.0f, 10.0f, 20.0f, 20.0f), 2.0f);

  EXPECT_FALSE(round_rect.IsEmpty());
  EXPECT_FALSE(round_rect.IsRect());
  EXPECT_FALSE(round_rect.IsOval());
  EXPECT_TRUE(round_rect.IsFinite());
  EXPECT_FALSE(round_rect.GetBounds().IsEmpty());
  EXPECT_EQ(round_rect.GetBounds(), Rect::MakeLTRB(10.0f, 10.0f, 20.0f, 20.0f));
  EXPECT_EQ(round_rect.GetRadii().top_left, Size(2.0f, 2.0f));
  EXPECT_EQ(round_rect.GetRadii().top_right, Size(2.0f, 2.0f));
  EXPECT_EQ(round_rect.GetRadii().bottom_left, Size(2.0f, 2.0f));
  EXPECT_EQ(round_rect.GetRadii().bottom_right, Size(2.0f, 2.0f));
}

TEST(RoundRectTest, RectXYConstructor) {
  RoundRect round_rect = RoundRect::MakeRectXY(
      Rect::MakeLTRB(10.0f, 10.0f, 20.0f, 20.0f), 2.0f, 3.0f);

  EXPECT_FALSE(round_rect.IsEmpty());
  EXPECT_FALSE(round_rect.IsRect());
  EXPECT_FALSE(round_rect.IsOval());
  EXPECT_TRUE(round_rect.IsFinite());
  EXPECT_FALSE(round_rect.GetBounds().IsEmpty());
  EXPECT_EQ(round_rect.GetBounds(), Rect::MakeLTRB(10.0f, 10.0f, 20.0f, 20.0f));
  EXPECT_EQ(round_rect.GetRadii().top_left, Size(2.0f, 3.0f));
  EXPECT_EQ(round_rect.GetRadii().top_right, Size(2.0f, 3.0f));
  EXPECT_EQ(round_rect.GetRadii().bottom_left, Size(2.0f, 3.0f));
  EXPECT_EQ(round_rect.GetRadii().bottom_right, Size(2.0f, 3.0f));
}

TEST(RoundRectTest, RectSizeConstructor) {
  RoundRect round_rect = RoundRect::MakeRectXY(
      Rect::MakeLTRB(10.0f, 10.0f, 20.0f, 20.0f), Size(2.0f, 3.0f));

  EXPECT_FALSE(round_rect.IsEmpty());
  EXPECT_FALSE(round_rect.IsRect());
  EXPECT_FALSE(round_rect.IsOval());
  EXPECT_TRUE(round_rect.IsFinite());
  EXPECT_FALSE(round_rect.GetBounds().IsEmpty());
  EXPECT_EQ(round_rect.GetBounds(), Rect::MakeLTRB(10.0f, 10.0f, 20.0f, 20.0f));
  EXPECT_EQ(round_rect.GetRadii().top_left, Size(2.0f, 3.0f));
  EXPECT_EQ(round_rect.GetRadii().top_right, Size(2.0f, 3.0f));
  EXPECT_EQ(round_rect.GetRadii().bottom_left, Size(2.0f, 3.0f));
  EXPECT_EQ(round_rect.GetRadii().bottom_right, Size(2.0f, 3.0f));
}

TEST(RoundRectTest, RectRadiiConstructor) {
  RoundRect round_rect =
      RoundRect::MakeRectRadii(Rect::MakeLTRB(10.0f, 10.0f, 20.0f, 20.0f),
                               {
                                   .top_left = Size(1.0, 1.5),
                                   .top_right = Size(2.0, 2.5f),
                                   .bottom_left = Size(3.0, 3.5f),
                                   .bottom_right = Size(4.0, 4.5f),
                               });

  EXPECT_FALSE(round_rect.IsEmpty());
  EXPECT_FALSE(round_rect.IsRect());
  EXPECT_FALSE(round_rect.IsOval());
  EXPECT_TRUE(round_rect.IsFinite());
  EXPECT_FALSE(round_rect.GetBounds().IsEmpty());
  EXPECT_EQ(round_rect.GetBounds(), Rect::MakeLTRB(10.0f, 10.0f, 20.0f, 20.0f));
  EXPECT_EQ(round_rect.GetRadii().top_left, Size(1.0f, 1.5f));
  EXPECT_EQ(round_rect.GetRadii().top_right, Size(2.0f, 2.5f));
  EXPECT_EQ(round_rect.GetRadii().bottom_left, Size(3.0f, 3.5f));
  EXPECT_EQ(round_rect.GetRadii().bottom_right, Size(4.0f, 4.5f));
}

TEST(RoundRectTest, RectRadiiOverflowWidthConstructor) {
  RoundRect round_rect =
      RoundRect::MakeRectRadii(Rect::MakeXYWH(10.0f, 10.0f, 6.0f, 30.0f),
                               {
                                   .top_left = Size(1.0f, 2.0f),
                                   .top_right = Size(3.0f, 4.0f),
                                   .bottom_left = Size(5.0f, 6.0f),
                                   .bottom_right = Size(7.0f, 8.0f),
                               });
  // Largest sum of paired radii widths is the bottom edge which sums to 12
  // Rect is only 6 wide so all radii are scaled by half
  // Rect is 30 tall so no scaling should happen due to radii heights

  EXPECT_FALSE(round_rect.IsEmpty());
  EXPECT_FALSE(round_rect.IsRect());
  EXPECT_FALSE(round_rect.IsOval());
  EXPECT_TRUE(round_rect.IsFinite());
  EXPECT_FALSE(round_rect.GetBounds().IsEmpty());
  EXPECT_EQ(round_rect.GetBounds(), Rect::MakeLTRB(10.0f, 10.0f, 16.0f, 40.0f));
  EXPECT_EQ(round_rect.GetRadii().top_left, Size(0.5f, 1.0f));
  EXPECT_EQ(round_rect.GetRadii().top_right, Size(1.5f, 2.0f));
  EXPECT_EQ(round_rect.GetRadii().bottom_left, Size(2.5f, 3.0f));
  EXPECT_EQ(round_rect.GetRadii().bottom_right, Size(3.5f, 4.0f));
}

TEST(RoundRectTest, RectRadiiOverflowHeightConstructor) {
  RoundRect round_rect =
      RoundRect::MakeRectRadii(Rect::MakeXYWH(10.0f, 10.0f, 30.0f, 6.0f),
                               {
                                   .top_left = Size(1.0f, 2.0f),
                                   .top_right = Size(3.0f, 4.0f),
                                   .bottom_left = Size(5.0f, 6.0f),
                                   .bottom_right = Size(7.0f, 8.0f),
                               });
  // Largest sum of paired radii heights is the right edge which sums to 12
  // Rect is only 6 tall so all radii are scaled by half
  // Rect is 30 wide so no scaling should happen due to radii widths

  EXPECT_FALSE(round_rect.IsEmpty());
  EXPECT_FALSE(round_rect.IsRect());
  EXPECT_FALSE(round_rect.IsOval());
  EXPECT_TRUE(round_rect.IsFinite());
  EXPECT_FALSE(round_rect.GetBounds().IsEmpty());
  EXPECT_EQ(round_rect.GetBounds(), Rect::MakeLTRB(10.0f, 10.0f, 40.0f, 16.0f));
  EXPECT_EQ(round_rect.GetRadii().top_left, Size(0.5f, 1.0f));
  EXPECT_EQ(round_rect.GetRadii().top_right, Size(1.5f, 2.0f));
  EXPECT_EQ(round_rect.GetRadii().bottom_left, Size(2.5f, 3.0f));
  EXPECT_EQ(round_rect.GetRadii().bottom_right, Size(3.5f, 4.0f));
}

TEST(RoundRectTest, Shift) {
  RoundRect round_rect =
      RoundRect::MakeRectRadii(Rect::MakeXYWH(10.0f, 10.0f, 30.0f, 30.0f),
                               {
                                   .top_left = Size(1.0f, 2.0f),
                                   .top_right = Size(3.0f, 4.0f),
                                   .bottom_left = Size(5.0f, 6.0f),
                                   .bottom_right = Size(7.0f, 8.0f),
                               });
  RoundRect shifted = round_rect.Shift(5.0, 6.0);

  EXPECT_FALSE(shifted.IsEmpty());
  EXPECT_FALSE(shifted.IsRect());
  EXPECT_FALSE(shifted.IsOval());
  EXPECT_TRUE(shifted.IsFinite());
  EXPECT_FALSE(shifted.GetBounds().IsEmpty());
  EXPECT_EQ(shifted.GetBounds(), Rect::MakeLTRB(15.0f, 16.0f, 45.0f, 46.0f));
  EXPECT_EQ(shifted.GetRadii().top_left, Size(1.0f, 2.0f));
  EXPECT_EQ(shifted.GetRadii().top_right, Size(3.0f, 4.0f));
  EXPECT_EQ(shifted.GetRadii().bottom_left, Size(5.0f, 6.0f));
  EXPECT_EQ(shifted.GetRadii().bottom_right, Size(7.0f, 8.0f));

  EXPECT_EQ(shifted,
            RoundRect::MakeRectRadii(Rect::MakeXYWH(15.0f, 16.0f, 30.0f, 30.0f),
                                     {
                                         .top_left = Size(1.0f, 2.0f),
                                         .top_right = Size(3.0f, 4.0f),
                                         .bottom_left = Size(5.0f, 6.0f),
                                         .bottom_right = Size(7.0f, 8.0f),
                                     }));
}

TEST(RoundRectTest, ExpandScalar) {
  RoundRect round_rect =
      RoundRect::MakeRectRadii(Rect::MakeXYWH(10.0f, 10.0f, 30.0f, 30.0f),
                               {
                                   .top_left = Size(1.0f, 2.0f),
                                   .top_right = Size(3.0f, 4.0f),
                                   .bottom_left = Size(5.0f, 6.0f),
                                   .bottom_right = Size(7.0f, 8.0f),
                               });
  RoundRect expanded = round_rect.Expand(5.0);

  EXPECT_FALSE(expanded.IsEmpty());
  EXPECT_FALSE(expanded.IsRect());
  EXPECT_FALSE(expanded.IsOval());
  EXPECT_TRUE(expanded.IsFinite());
  EXPECT_FALSE(expanded.GetBounds().IsEmpty());
  EXPECT_EQ(expanded.GetBounds(), Rect::MakeLTRB(5.0f, 5.0f, 45.0f, 45.0f));
  EXPECT_EQ(expanded.GetRadii().top_left, Size(1.0f, 2.0f));
  EXPECT_EQ(expanded.GetRadii().top_right, Size(3.0f, 4.0f));
  EXPECT_EQ(expanded.GetRadii().bottom_left, Size(5.0f, 6.0f));
  EXPECT_EQ(expanded.GetRadii().bottom_right, Size(7.0f, 8.0f));

  EXPECT_EQ(expanded,
            RoundRect::MakeRectRadii(Rect::MakeXYWH(5.0f, 5.0f, 40.0f, 40.0f),
                                     {
                                         .top_left = Size(1.0f, 2.0f),
                                         .top_right = Size(3.0f, 4.0f),
                                         .bottom_left = Size(5.0f, 6.0f),
                                         .bottom_right = Size(7.0f, 8.0f),
                                     }));
}

TEST(RoundRectTest, ExpandTwoScalars) {
  RoundRect round_rect =
      RoundRect::MakeRectRadii(Rect::MakeXYWH(10.0f, 10.0f, 30.0f, 30.0f),
                               {
                                   .top_left = Size(1.0f, 2.0f),
                                   .top_right = Size(3.0f, 4.0f),
                                   .bottom_left = Size(5.0f, 6.0f),
                                   .bottom_right = Size(7.0f, 8.0f),
                               });
  RoundRect expanded = round_rect.Expand(5.0, 6.0);

  EXPECT_FALSE(expanded.IsEmpty());
  EXPECT_FALSE(expanded.IsRect());
  EXPECT_FALSE(expanded.IsOval());
  EXPECT_TRUE(expanded.IsFinite());
  EXPECT_FALSE(expanded.GetBounds().IsEmpty());
  EXPECT_EQ(expanded.GetBounds(), Rect::MakeLTRB(5.0f, 4.0f, 45.0f, 46.0f));
  EXPECT_EQ(expanded.GetRadii().top_left, Size(1.0f, 2.0f));
  EXPECT_EQ(expanded.GetRadii().top_right, Size(3.0f, 4.0f));
  EXPECT_EQ(expanded.GetRadii().bottom_left, Size(5.0f, 6.0f));
  EXPECT_EQ(expanded.GetRadii().bottom_right, Size(7.0f, 8.0f));

  EXPECT_EQ(expanded,
            RoundRect::MakeRectRadii(Rect::MakeXYWH(5.0f, 4.0f, 40.0f, 42.0f),
                                     {
                                         .top_left = Size(1.0f, 2.0f),
                                         .top_right = Size(3.0f, 4.0f),
                                         .bottom_left = Size(5.0f, 6.0f),
                                         .bottom_right = Size(7.0f, 8.0f),
                                     }));
}

TEST(RoundRectTest, ExpandFourScalars) {
  RoundRect round_rect =
      RoundRect::MakeRectRadii(Rect::MakeXYWH(10.0f, 10.0f, 30.0f, 30.0f),
                               {
                                   .top_left = Size(1.0f, 2.0f),
                                   .top_right = Size(3.0f, 4.0f),
                                   .bottom_left = Size(5.0f, 6.0f),
                                   .bottom_right = Size(7.0f, 8.0f),
                               });
  RoundRect expanded = round_rect.Expand(5.0, 6.0, 7.0, 8.0);

  EXPECT_FALSE(expanded.IsEmpty());
  EXPECT_FALSE(expanded.IsRect());
  EXPECT_FALSE(expanded.IsOval());
  EXPECT_TRUE(expanded.IsFinite());
  EXPECT_FALSE(expanded.GetBounds().IsEmpty());
  EXPECT_EQ(expanded.GetBounds(), Rect::MakeLTRB(5.0f, 4.0f, 47.0f, 48.0f));
  EXPECT_EQ(expanded.GetRadii().top_left, Size(1.0f, 2.0f));
  EXPECT_EQ(expanded.GetRadii().top_right, Size(3.0f, 4.0f));
  EXPECT_EQ(expanded.GetRadii().bottom_left, Size(5.0f, 6.0f));
  EXPECT_EQ(expanded.GetRadii().bottom_right, Size(7.0f, 8.0f));

  EXPECT_EQ(expanded,
            RoundRect::MakeRectRadii(Rect::MakeXYWH(5.0f, 4.0f, 42.0f, 44.0f),
                                     {
                                         .top_left = Size(1.0f, 2.0f),
                                         .top_right = Size(3.0f, 4.0f),
                                         .bottom_left = Size(5.0f, 6.0f),
                                         .bottom_right = Size(7.0f, 8.0f),
                                     }));
}

TEST(RoundRectTest, ContractScalar) {
  RoundRect round_rect =
      RoundRect::MakeRectRadii(Rect::MakeXYWH(10.0f, 10.0f, 30.0f, 30.0f),
                               {
                                   .top_left = Size(1.0f, 2.0f),
                                   .top_right = Size(3.0f, 4.0f),
                                   .bottom_left = Size(5.0f, 6.0f),
                                   .bottom_right = Size(7.0f, 8.0f),
                               });
  RoundRect expanded = round_rect.Expand(-2.0);

  EXPECT_FALSE(expanded.IsEmpty());
  EXPECT_FALSE(expanded.IsRect());
  EXPECT_FALSE(expanded.IsOval());
  EXPECT_TRUE(expanded.IsFinite());
  EXPECT_FALSE(expanded.GetBounds().IsEmpty());
  EXPECT_EQ(expanded.GetBounds(), Rect::MakeLTRB(12.0f, 12.0f, 38.0f, 38.0f));
  EXPECT_EQ(expanded.GetRadii().top_left, Size(1.0f, 2.0f));
  EXPECT_EQ(expanded.GetRadii().top_right, Size(3.0f, 4.0f));
  EXPECT_EQ(expanded.GetRadii().bottom_left, Size(5.0f, 6.0f));
  EXPECT_EQ(expanded.GetRadii().bottom_right, Size(7.0f, 8.0f));

  EXPECT_EQ(expanded,
            RoundRect::MakeRectRadii(Rect::MakeXYWH(12.0f, 12.0f, 26.0f, 26.0f),
                                     {
                                         .top_left = Size(1.0f, 2.0f),
                                         .top_right = Size(3.0f, 4.0f),
                                         .bottom_left = Size(5.0f, 6.0f),
                                         .bottom_right = Size(7.0f, 8.0f),
                                     }));
}

TEST(RoundRectTest, ContractTwoScalars) {
  RoundRect round_rect =
      RoundRect::MakeRectRadii(Rect::MakeXYWH(10.0f, 10.0f, 30.0f, 30.0f),
                               {
                                   .top_left = Size(1.0f, 2.0f),
                                   .top_right = Size(3.0f, 4.0f),
                                   .bottom_left = Size(5.0f, 6.0f),
                                   .bottom_right = Size(7.0f, 8.0f),
                               });
  RoundRect expanded = round_rect.Expand(-1.0, -2.0);

  EXPECT_FALSE(expanded.IsEmpty());
  EXPECT_FALSE(expanded.IsRect());
  EXPECT_FALSE(expanded.IsOval());
  EXPECT_TRUE(expanded.IsFinite());
  EXPECT_FALSE(expanded.GetBounds().IsEmpty());
  EXPECT_EQ(expanded.GetBounds(), Rect::MakeLTRB(11.0f, 12.0f, 39.0f, 38.0f));
  EXPECT_EQ(expanded.GetRadii().top_left, Size(1.0f, 2.0f));
  EXPECT_EQ(expanded.GetRadii().top_right, Size(3.0f, 4.0f));
  EXPECT_EQ(expanded.GetRadii().bottom_left, Size(5.0f, 6.0f));
  EXPECT_EQ(expanded.GetRadii().bottom_right, Size(7.0f, 8.0f));

  EXPECT_EQ(expanded,
            RoundRect::MakeRectRadii(Rect::MakeXYWH(11.0f, 12.0f, 28.0f, 26.0f),
                                     {
                                         .top_left = Size(1.0f, 2.0f),
                                         .top_right = Size(3.0f, 4.0f),
                                         .bottom_left = Size(5.0f, 6.0f),
                                         .bottom_right = Size(7.0f, 8.0f),
                                     }));
}

TEST(RoundRectTest, ContractFourScalars) {
  RoundRect round_rect =
      RoundRect::MakeRectRadii(Rect::MakeXYWH(10.0f, 10.0f, 30.0f, 30.0f),
                               {
                                   .top_left = Size(1.0f, 2.0f),
                                   .top_right = Size(3.0f, 4.0f),
                                   .bottom_left = Size(5.0f, 6.0f),
                                   .bottom_right = Size(7.0f, 8.0f),
                               });
  RoundRect expanded = round_rect.Expand(-1.0, -1.5, -2.0, -2.5);

  EXPECT_FALSE(expanded.IsEmpty());
  EXPECT_FALSE(expanded.IsRect());
  EXPECT_FALSE(expanded.IsOval());
  EXPECT_TRUE(expanded.IsFinite());
  EXPECT_FALSE(expanded.GetBounds().IsEmpty());
  EXPECT_EQ(expanded.GetBounds(), Rect::MakeLTRB(11.0f, 11.5f, 38.0f, 37.5f));
  EXPECT_EQ(expanded.GetRadii().top_left, Size(1.0f, 2.0f));
  EXPECT_EQ(expanded.GetRadii().top_right, Size(3.0f, 4.0f));
  EXPECT_EQ(expanded.GetRadii().bottom_left, Size(5.0f, 6.0f));
  EXPECT_EQ(expanded.GetRadii().bottom_right, Size(7.0f, 8.0f));

  EXPECT_EQ(expanded,
            RoundRect::MakeRectRadii(Rect::MakeXYWH(11.0f, 11.5f, 27.0f, 26.0f),
                                     {
                                         .top_left = Size(1.0f, 2.0f),
                                         .top_right = Size(3.0f, 4.0f),
                                         .bottom_left = Size(5.0f, 6.0f),
                                         .bottom_right = Size(7.0f, 8.0f),
                                     }));
}

TEST(RoundRectTest, ContractAndRequireRadiiAdjustment) {
  RoundRect round_rect =
      RoundRect::MakeRectRadii(Rect::MakeXYWH(10.0f, 10.0f, 30.0f, 30.0f),
                               {
                                   .top_left = Size(1.0f, 2.0f),
                                   .top_right = Size(3.0f, 4.0f),
                                   .bottom_left = Size(5.0f, 6.0f),
                                   .bottom_right = Size(7.0f, 8.0f),
                               });
  RoundRect expanded = round_rect.Expand(-12.0);
  // Largest sum of paired radii sizes are the bottom and right edges
  // both of which sum to 12
  // Rect was 30x30 reduced by 12 on all sides leaving only 6x6, so all
  // radii are scaled by half to avoid overflowing the contracted rect

  EXPECT_FALSE(expanded.IsEmpty());
  EXPECT_FALSE(expanded.IsRect());
  EXPECT_FALSE(expanded.IsOval());
  EXPECT_TRUE(expanded.IsFinite());
  EXPECT_FALSE(expanded.GetBounds().IsEmpty());
  EXPECT_EQ(expanded.GetBounds(), Rect::MakeLTRB(22.0f, 22.0f, 28.0f, 28.0f));
  EXPECT_EQ(expanded.GetRadii().top_left, Size(0.5f, 1.0f));
  EXPECT_EQ(expanded.GetRadii().top_right, Size(1.5f, 2.0f));
  EXPECT_EQ(expanded.GetRadii().bottom_left, Size(2.5f, 3.0f));
  EXPECT_EQ(expanded.GetRadii().bottom_right, Size(3.5f, 4.0f));

  // In this test, the MakeRectRadii constructor will make the same
  // adjustment to the radii that the Expand method applied.
  EXPECT_EQ(expanded,
            RoundRect::MakeRectRadii(Rect::MakeXYWH(22.0f, 22.0f, 6.0f, 6.0f),
                                     {
                                         .top_left = Size(1.0f, 2.0f),
                                         .top_right = Size(3.0f, 4.0f),
                                         .bottom_left = Size(5.0f, 6.0f),
                                         .bottom_right = Size(7.0f, 8.0f),
                                     }));

  // In this test, the arguments to the constructor supply the correctly
  // adjusted radii (though there is no real way to tell other than
  // the result is the same).
  EXPECT_EQ(expanded,
            RoundRect::MakeRectRadii(Rect::MakeXYWH(22.0f, 22.0f, 6.0f, 6.0f),
                                     {
                                         .top_left = Size(0.5f, 1.0f),
                                         .top_right = Size(1.5f, 2.0f),
                                         .bottom_left = Size(2.5f, 3.0f),
                                         .bottom_right = Size(3.5f, 4.0f),
                                     }));
}

TEST(RoundRectTest, NoCornerRoundRectContains) {
  Rect bounds = Rect::MakeLTRB(-50.0f, -50.0f, 50.0f, 50.0f);
  // RRect of bounds with no corners contains corners just barely
  auto no_corners = RoundRect::MakeRectXY(bounds, 0.0f, 0.0f);

  EXPECT_TRUE(no_corners.Contains({-50, -50}));
  // Rectangles have half-in, half-out containment so we need
  // to be careful about testing containment of right/bottom corners.
  EXPECT_TRUE(no_corners.Contains({-50, 49.99}));
  EXPECT_TRUE(no_corners.Contains({49.99, -50}));
  EXPECT_TRUE(no_corners.Contains({49.99, 49.99}));
  EXPECT_FALSE(no_corners.Contains({-50.01, -50}));
  EXPECT_FALSE(no_corners.Contains({-50, -50.01}));
  EXPECT_FALSE(no_corners.Contains({-50.01, 50}));
  EXPECT_FALSE(no_corners.Contains({-50, 50.01}));
  EXPECT_FALSE(no_corners.Contains({50.01, -50}));
  EXPECT_FALSE(no_corners.Contains({50, -50.01}));
  EXPECT_FALSE(no_corners.Contains({50.01, 50}));
  EXPECT_FALSE(no_corners.Contains({50, 50.01}));
}

TEST(RoundRectTest, TinyCornerRoundRectContains) {
  Rect bounds = Rect::MakeLTRB(-50.0f, -50.0f, 50.0f, 50.0f);
  // RRect of bounds with even the tiniest corners does not contain corners
  auto tiny_corners = RoundRect::MakeRectXY(bounds, 0.01f, 0.01f);

  EXPECT_FALSE(tiny_corners.Contains({-50, -50}));
  EXPECT_FALSE(tiny_corners.Contains({-50, 50}));
  EXPECT_FALSE(tiny_corners.Contains({50, -50}));
  EXPECT_FALSE(tiny_corners.Contains({50, 50}));
}

TEST(RoundRectTest, UniformCircularRoundRectContains) {
  Rect bounds = Rect::MakeLTRB(-50.0f, -50.0f, 50.0f, 50.0f);
  auto expanded_2_r_2 = RoundRect::MakeRectXY(bounds.Expand(2.0), 2.0f, 2.0f);

  // Expanded by 2.0 and then with a corner of 2.0 obviously still
  // contains the corners
  EXPECT_TRUE(expanded_2_r_2.Contains({-50, -50}));
  EXPECT_TRUE(expanded_2_r_2.Contains({-50, 50}));
  EXPECT_TRUE(expanded_2_r_2.Contains({50, -50}));
  EXPECT_TRUE(expanded_2_r_2.Contains({50, 50}));

  // Now we try to box in the corner containment to exactly where the
  // rounded corner of the expanded round rect with radii of 2.0 lies.
  // The 45-degree diagonal point of a circle of radius 2.0 lies at:
  //
  // (2 * sqrt(2) / 2, 2 * sqrt(2) / 2)
  // (sqrt(2), sqrt(2))
  //
  // So we test +/- (50 + sqrt(2) +/- epsilon)
  const auto coord_out = 50 + kSqrt2 + kEhCloseEnough;
  const auto coord_in = 50 + kSqrt2 - kEhCloseEnough;
  // Upper left corner
  EXPECT_TRUE(expanded_2_r_2.Contains({-coord_in, -coord_in}));
  EXPECT_FALSE(expanded_2_r_2.Contains({-coord_out, -coord_out}));
  // Upper right corner
  EXPECT_TRUE(expanded_2_r_2.Contains({coord_in, -coord_in}));
  EXPECT_FALSE(expanded_2_r_2.Contains({coord_out, -coord_out}));
  // Lower left corner
  EXPECT_TRUE(expanded_2_r_2.Contains({-coord_in, coord_in}));
  EXPECT_FALSE(expanded_2_r_2.Contains({-coord_out, coord_out}));
  // Lower right corner
  EXPECT_TRUE(expanded_2_r_2.Contains({coord_in, coord_in}));
  EXPECT_FALSE(expanded_2_r_2.Contains({coord_out, coord_out}));
}

TEST(RoundRectTest, UniformEllipticalRoundRectContains) {
  Rect bounds = Rect::MakeLTRB(-50.0f, -50.0f, 50.0f, 50.0f);
  auto expanded_2_r_2 = RoundRect::MakeRectXY(bounds.Expand(2.0), 2.0f, 3.0f);

  // Expanded by 2.0 and then with a corner of 2x3 should still
  // contain the corners
  EXPECT_TRUE(expanded_2_r_2.Contains({-50, -50}));
  EXPECT_TRUE(expanded_2_r_2.Contains({-50, 50}));
  EXPECT_TRUE(expanded_2_r_2.Contains({50, -50}));
  EXPECT_TRUE(expanded_2_r_2.Contains({50, 50}));

  // Now we try to box in the corner containment to exactly where the
  // rounded corner of the expanded round rect with radii of 2x3 lies.
  // The "45-degree diagonal point" of an ellipse of radii 2x3 lies at:
  //
  // (2 * sqrt(2) / 2, 3 * sqrt(2) / 2)
  // (sqrt(2), 3 * sqrt(2) / 2)
  //
  // And the center(s) of these corners are at:
  // (+/-(50 + 2 - 2), +/-(50 + 2 - 3))
  // = (+/-50, +/-49)
  const auto x_coord_out = 50 + kSqrt2 + kEhCloseEnough;
  const auto x_coord_in = 50 + kSqrt2 - kEhCloseEnough;
  const auto y_coord_out = 49 + 3 * kSqrt2 / 2 + kEhCloseEnough;
  const auto y_coord_in = 49 + 3 * kSqrt2 / 2 - kEhCloseEnough;
  // Upper left corner
  EXPECT_TRUE(expanded_2_r_2.Contains({-x_coord_in, -y_coord_in}));
  EXPECT_FALSE(expanded_2_r_2.Contains({-x_coord_out, -y_coord_out}));
  // Upper right corner
  EXPECT_TRUE(expanded_2_r_2.Contains({x_coord_in, -y_coord_in}));
  EXPECT_FALSE(expanded_2_r_2.Contains({x_coord_out, -y_coord_out}));
  // Lower left corner
  EXPECT_TRUE(expanded_2_r_2.Contains({-x_coord_in, y_coord_in}));
  EXPECT_FALSE(expanded_2_r_2.Contains({-x_coord_out, y_coord_out}));
  // Lower right corner
  EXPECT_TRUE(expanded_2_r_2.Contains({x_coord_in, y_coord_in}));
  EXPECT_FALSE(expanded_2_r_2.Contains({x_coord_out, y_coord_out}));
}

TEST(RoundRectTest, DifferingCornersRoundRectContains) {
  Rect bounds = Rect::MakeLTRB(-50.0f, -50.0f, 50.0f, 50.0f);
  auto round_rect =
      RoundRect::MakeRectRadii(bounds, {
                                           .top_left = Size(2.0, 3.0),
                                           .top_right = Size(4.0, 5.0),
                                           .bottom_left = Size(6.0, 7.0),
                                           .bottom_right = Size(8.0, 9.0),
                                       });

  // For a corner with radii {A, B}, the "45 degree point" on the
  // corner curve will be at an offset of:
  //
  // (A * sqrt(2) / 2, B * sqrt(2) / 2)
  //
  // And the center(s) of these corners are at:
  //
  // (+/-(50 - A), +/-(50 - B))
  auto coord = [](Scalar radius) {
    return 50 - radius + radius * kSqrt2 / 2.0f - kEhCloseEnough;
  };
  auto coord_in = [&coord](Scalar radius) {
    return coord(radius) - kEhCloseEnough;
  };
  auto coord_out = [&coord](Scalar radius) {
    // For some reason 1 kEhCloseEnough is not enough to put us outside
    // in some of the cases, so we use 2x the epsilon.
    return coord(radius) + 2 * kEhCloseEnough;
  };
  // Upper left corner (radii = {2.0, 3.0})
  EXPECT_TRUE(round_rect.Contains({-coord_in(2.0), -coord_in(3.0)}));
  EXPECT_FALSE(round_rect.Contains({-coord_out(2.0), -coord_out(3.0)}));
  // Upper right corner (radii = {4.0, 5.0})
  EXPECT_TRUE(round_rect.Contains({coord_in(4.0), -coord_in(5.0)}));
  EXPECT_FALSE(round_rect.Contains({coord_out(4.0), -coord_out(5.0)}));
  // Lower left corner (radii = {6.0, 7.0})
  EXPECT_TRUE(round_rect.Contains({-coord_in(6.0), coord_in(7.0)}));
  EXPECT_FALSE(round_rect.Contains({-coord_out(6.0), coord_out(7.0)}));
  // Lower right corner (radii = {8.0, 9.0})
  EXPECT_TRUE(round_rect.Contains({coord_in(8.0), coord_in(9.0)}));
  EXPECT_FALSE(round_rect.Contains({coord_out(8.0), coord_out(9.0)}));
}

}  // namespace testing
}  // namespace impeller
