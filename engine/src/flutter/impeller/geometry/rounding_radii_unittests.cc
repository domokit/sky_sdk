// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "gtest/gtest.h"

#include "flutter/impeller/geometry/rounding_radii.h"

#include "flutter/impeller/geometry/geometry_asserts.h"

namespace impeller {
namespace testing {

TEST(RoudingRadiiTest, RoundingRadiiEmptyDeclaration) {
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

TEST(RoudingRadiiTest, RoundingRadiiDefaultConstructor) {
  RoundingRadii radii = RoundingRadii();

  EXPECT_TRUE(radii.AreAllCornersEmpty());
  EXPECT_TRUE(radii.AreAllCornersSame());
  EXPECT_TRUE(radii.IsFinite());
  EXPECT_EQ(radii.top_left, Size());
  EXPECT_EQ(radii.top_right, Size());
  EXPECT_EQ(radii.bottom_left, Size());
  EXPECT_EQ(radii.bottom_right, Size());
}

TEST(RoudingRadiiTest, RoundingRadiiScalarConstructor) {
  RoundingRadii radii = RoundingRadii::MakeRadius(5.0f);

  EXPECT_FALSE(radii.AreAllCornersEmpty());
  EXPECT_TRUE(radii.AreAllCornersSame());
  EXPECT_TRUE(radii.IsFinite());
  EXPECT_EQ(radii.top_left, Size(5.0f, 5.0f));
  EXPECT_EQ(radii.top_right, Size(5.0f, 5.0f));
  EXPECT_EQ(radii.bottom_left, Size(5.0f, 5.0f));
  EXPECT_EQ(radii.bottom_right, Size(5.0f, 5.0f));
}

TEST(RoudingRadiiTest, RoundingRadiiEmptyScalarConstructor) {
  RoundingRadii radii = RoundingRadii::MakeRadius(-5.0f);

  EXPECT_TRUE(radii.AreAllCornersEmpty());
  EXPECT_TRUE(radii.AreAllCornersSame());
  EXPECT_TRUE(radii.IsFinite());
  EXPECT_EQ(radii.top_left, Size(-5.0f, -5.0f));
  EXPECT_EQ(radii.top_right, Size(-5.0f, -5.0f));
  EXPECT_EQ(radii.bottom_left, Size(-5.0f, -5.0f));
  EXPECT_EQ(radii.bottom_right, Size(-5.0f, -5.0f));
}

TEST(RoudingRadiiTest, RoundingRadiiSizeConstructor) {
  RoundingRadii radii = RoundingRadii::MakeRadii(Size(5.0f, 6.0f));

  EXPECT_FALSE(radii.AreAllCornersEmpty());
  EXPECT_TRUE(radii.AreAllCornersSame());
  EXPECT_TRUE(radii.IsFinite());
  EXPECT_EQ(radii.top_left, Size(5.0f, 6.0f));
  EXPECT_EQ(radii.top_right, Size(5.0f, 6.0f));
  EXPECT_EQ(radii.bottom_left, Size(5.0f, 6.0f));
  EXPECT_EQ(radii.bottom_right, Size(5.0f, 6.0f));
}

TEST(RoudingRadiiTest, RoundingRadiiEmptySizeConstructor) {
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

TEST(RoudingRadiiTest, RoundingRadiiNamedSizesConstructor) {
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

TEST(RoudingRadiiTest, RoundingRadiiPartialNamedSizesConstructor) {
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

TEST(RoudingRadiiTest, RoundingRadiiMultiply) {
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

TEST(RoudingRadiiTest, RoundingRadiiEquals) {
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

TEST(RoudingRadiiTest, RoundingRadiiNotEquals) {
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

TEST(RoudingRadiiTest, RoundingRadiiCornersSameTolerance) {
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

}  // namespace testing
}  // namespace impeller
