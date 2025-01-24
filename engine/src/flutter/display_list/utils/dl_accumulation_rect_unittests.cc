// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/display_list/utils/dl_accumulation_rect.h"
#include "gtest/gtest.h"

namespace flutter {
namespace testing {

TEST(DisplayListAccumulationRect, Constructor) {
  AccumulationRect accumulator;

  EXPECT_TRUE(accumulator.is_empty());
  EXPECT_TRUE(accumulator.GetBounds().IsEmpty());
  EXPECT_FALSE(accumulator.overlap_detected());
}

TEST(DisplayListAccumulationRect, OnePoint) {
  AccumulationRect accumulator;
  accumulator.accumulate(10.0f, 10.0f);

  EXPECT_TRUE(accumulator.is_empty());
  EXPECT_TRUE(accumulator.GetBounds().IsEmpty());
  EXPECT_FALSE(accumulator.overlap_detected());
}

TEST(DisplayListAccumulationRect, TwoPoints) {
  auto test = [](DlScalar x1, DlScalar y1,  //
                 DlScalar x2, DlScalar y2,  //
                 DlRect bounds,             //
                 bool should_be_empty, bool should_overlap,
                 const std::string& label) {
    {
      AccumulationRect accumulator;
      accumulator.accumulate(x1, y1);
      accumulator.accumulate(x2, y2);

      EXPECT_EQ(accumulator.is_empty(), should_be_empty) << label;
      EXPECT_EQ(accumulator.GetBounds().IsEmpty(), should_be_empty) << label;
      EXPECT_EQ(accumulator.GetBounds(), bounds) << label;
      EXPECT_EQ(accumulator.overlap_detected(), should_overlap) << label;
    }

    {
      AccumulationRect accumulator;
      accumulator.accumulate(DlPoint(x1, y1));
      accumulator.accumulate(DlPoint(x2, y2));

      EXPECT_EQ(accumulator.is_empty(), should_be_empty) << label;
      EXPECT_EQ(accumulator.GetBounds().IsEmpty(), should_be_empty) << label;
      EXPECT_EQ(accumulator.GetBounds(), bounds) << label;
      EXPECT_EQ(accumulator.overlap_detected(), should_overlap) << label;
    }
  };

  test(10.0f, 10.0f, 10.0f, 10.0f, DlRect::MakeLTRB(10.0f, 10.0f, 10.0f, 10.0f),
       true, false, "Same");
  test(10.0f, 10.0f, 20.0f, 10.0f, DlRect::MakeLTRB(10.0f, 10.0f, 20.0f, 10.0f),
       true, false, "Horizontal");
  test(10.0f, 10.0f, 10.0f, 20.0f, DlRect::MakeLTRB(10.0f, 10.0f, 10.0f, 20.0f),
       true, false, "Vertical");
  test(10.0f, 10.0f, 20.0f, 20.0f, DlRect::MakeLTRB(10.0f, 10.0f, 20.0f, 20.0f),
       false, false, "Diagonal");
}

TEST(DisplayListAccumulationRect, ThreePoints) {
  auto test = [](DlScalar x1, DlScalar y1,  //
                 DlScalar x2, DlScalar y2,  //
                 DlScalar x3, DlScalar y3,  //
                 DlRect bounds,             //
                 bool should_be_empty, bool should_overlap,
                 const std::string& label) {
    {
      AccumulationRect accumulator;
      accumulator.accumulate(x1, y1);
      accumulator.accumulate(x2, y2);
      accumulator.accumulate(x3, y3);

      EXPECT_EQ(accumulator.is_empty(), should_be_empty) << label;
      EXPECT_EQ(accumulator.GetBounds().IsEmpty(), should_be_empty) << label;
      EXPECT_EQ(accumulator.GetBounds(), bounds) << label;
      EXPECT_EQ(accumulator.overlap_detected(), should_overlap) << label;
    }

    {
      AccumulationRect accumulator;
      accumulator.accumulate(DlPoint(x1, y1));
      accumulator.accumulate(DlPoint(x2, y2));
      accumulator.accumulate(DlPoint(x3, y3));

      EXPECT_EQ(accumulator.is_empty(), should_be_empty) << label;
      EXPECT_EQ(accumulator.GetBounds().IsEmpty(), should_be_empty) << label;
      EXPECT_EQ(accumulator.GetBounds(), bounds) << label;
      EXPECT_EQ(accumulator.overlap_detected(), should_overlap) << label;
    }
  };

  test(10.0f, 10.0f, 10.0f, 10.0f, 10.0f, 10.0f,
       DlRect::MakeLTRB(10.0f, 10.0f, 10.0f, 10.0f), true, false, "Same");
  test(10.0f, 10.0f, 20.0f, 10.0f, 15.0f, 10.0f,
       DlRect::MakeLTRB(10.0f, 10.0f, 20.0f, 10.0f), true, false, "Horizontal");
  test(10.0f, 10.0f, 10.0f, 20.0f, 10.0f, 15.0f,
       DlRect::MakeLTRB(10.0f, 10.0f, 10.0f, 20.0f), true, false, "Vertical");
  test(10.0f, 10.0f, 20.0f, 20.0f, 25.0f, 15.0f,
       DlRect::MakeLTRB(10.0f, 10.0f, 25.0f, 20.0f), false, false, "Disjoint");
  test(10.0f, 10.0f, 20.0f, 20.0f, 15.0f, 15.0f,
       DlRect::MakeLTRB(10.0f, 10.0f, 20.0f, 20.0f), false, true, "Inside");
}

TEST(DisplayListAccumulationRect, EmptyRect) {
  auto test = [](DlScalar l, DlScalar t, DlScalar r, DlScalar b,  //
                 DlRect bounds,                                   //
                 bool should_be_empty, bool should_overlap,
                 const std::string& label) {
    {
      AccumulationRect accumulator;
      accumulator.accumulate(DlRect::MakeLTRB(l, t, r, b));

      EXPECT_EQ(accumulator.is_empty(), should_be_empty) << label;
      EXPECT_EQ(accumulator.GetBounds().IsEmpty(), should_be_empty) << label;
      EXPECT_EQ(accumulator.GetBounds(), bounds) << label;
      EXPECT_EQ(accumulator.overlap_detected(), should_overlap) << label;
    }

    {
      AccumulationRect accumulator;
      accumulator.accumulate(DlRect::MakeLTRB(l, t, r, b));

      EXPECT_EQ(accumulator.is_empty(), should_be_empty) << label;
      EXPECT_EQ(accumulator.GetBounds().IsEmpty(), should_be_empty) << label;
      EXPECT_EQ(accumulator.GetBounds(), bounds) << label;
      EXPECT_EQ(accumulator.overlap_detected(), should_overlap) << label;
    }

    {
      AccumulationRect content;
      content.accumulate(l, t);
      content.accumulate(r, b);
      EXPECT_EQ(content.is_empty(), should_be_empty) << label;
      EXPECT_EQ(content.GetBounds().IsEmpty(), should_be_empty) << label;
      // bounds for an accumulation by points may be different than the
      // bounds for an accumulation by the rect they produce because
      // construction by points has no "empty rejection" case.
      if (!should_be_empty) {
        EXPECT_EQ(content.GetBounds(), bounds) << label;
      }
      EXPECT_EQ(content.overlap_detected(), should_overlap) << label;

      AccumulationRect accumulator;
      accumulator.accumulate(content);

      EXPECT_EQ(accumulator.is_empty(), should_be_empty) << label;
      EXPECT_EQ(accumulator.GetBounds().IsEmpty(), should_be_empty) << label;
      EXPECT_EQ(accumulator.GetBounds(), bounds) << label;
      EXPECT_EQ(accumulator.overlap_detected(), should_overlap) << label;
    }
  };

  test(10.0f, 10.0f, 10.0f, 10.0f, DlRect::MakeLTRB(0.0f, 0.0f, 0.0f, 0.0f),
       true, false, "Singular");
  test(10.0f, 10.0f, 20.0f, 10.0f, DlRect::MakeLTRB(0.0f, 0.0f, 0.0f, 0.0f),
       true, false, "Horizontal Empty");
  test(10.0f, 10.0f, 10.0f, 20.0f, DlRect::MakeLTRB(0.0f, 0.0f, 0.0f, 0.0f),
       true, false, "Vertical Empty");
  test(10.0f, 10.0f, 20.0f, 20.0f, DlRect::MakeLTRB(10.0f, 10.0f, 20.0f, 20.0f),
       false, false, "Non-Empty");
}

TEST(DisplayListAccumulationRect, TwoRects) {
  auto test = [](DlScalar l1, DlScalar t1, DlScalar r1, DlScalar b1,  //
                 DlScalar l2, DlScalar t2, DlScalar r2, DlScalar b2,  //
                 DlRect bounds,                                       //
                 bool should_be_empty, bool should_overlap,
                 const std::string& label) {
    {
      AccumulationRect accumulator;
      accumulator.accumulate(DlRect::MakeLTRB(l1, t1, r1, b1));
      accumulator.accumulate(DlRect::MakeLTRB(l2, t2, r2, b2));

      EXPECT_EQ(accumulator.is_empty(), should_be_empty) << label;
      EXPECT_EQ(accumulator.GetBounds().IsEmpty(), should_be_empty) << label;
      EXPECT_EQ(accumulator.GetBounds(), bounds) << label;
      EXPECT_EQ(accumulator.overlap_detected(), should_overlap) << label;
    }

    {
      AccumulationRect content1;
      content1.accumulate(l1, t1);
      content1.accumulate(r1, b1);

      AccumulationRect content2;
      content2.accumulate(l2, t2);
      content2.accumulate(r2, b2);

      AccumulationRect accumulator;
      accumulator.accumulate(content1);
      accumulator.accumulate(content2);

      EXPECT_EQ(accumulator.is_empty(), should_be_empty) << label;
      EXPECT_EQ(accumulator.GetBounds().IsEmpty(), should_be_empty) << label;
      EXPECT_EQ(accumulator.GetBounds(), bounds) << label;
      EXPECT_EQ(accumulator.overlap_detected(), should_overlap) << label;
    }
  };

  test(10.0f, 10.0f, 10.0f, 10.0f,                //
       20.0f, 20.0f, 20.0f, 20.0f,                //
       DlRect::MakeLTRB(0.0f, 0.0f, 0.0f, 0.0f),  //
       true, false, "Empty + Empty");
  test(10.0f, 10.0f, 20.0f, 10.0f,                //
       10.0f, 10.0f, 10.0f, 20.0f,                //
       DlRect::MakeLTRB(0.0f, 0.0f, 0.0f, 0.0f),  //
       true, false, "Horizontal + Vertical");
  test(10.0f, 10.0f, 10.0f, 10.0f,                    //
       15.0f, 15.0f, 20.0f, 20.0f,                    //
       DlRect::MakeLTRB(15.0f, 15.0f, 20.0f, 20.0f),  //
       false, false, "Empty + Non-Empty");
  test(10.0f, 10.0f, 15.0f, 15.0f,                    //
       20.0f, 20.0f, 20.0f, 20.0f,                    //
       DlRect::MakeLTRB(10.0f, 10.0f, 15.0f, 15.0f),  //
       false, false, "Non-Empty + Empty");
  test(10.0f, 10.0f, 15.0f, 15.0f,                    //
       15.0f, 15.0f, 20.0f, 20.0f,                    //
       DlRect::MakeLTRB(10.0f, 10.0f, 20.0f, 20.0f),  //
       false, false, "Abutting");
  test(10.0f, 10.0f, 15.0f, 15.0f,                    //
       16.0f, 16.0f, 20.0f, 20.0f,                    //
       DlRect::MakeLTRB(10.0f, 10.0f, 20.0f, 20.0f),  //
       false, false, "Disjoint");
  test(10.0f, 10.0f, 16.0f, 16.0f,                    //
       15.0f, 15.0f, 20.0f, 20.0f,                    //
       DlRect::MakeLTRB(10.0f, 10.0f, 20.0f, 20.0f),  //
       false, true, "Overlapping");
}

}  // namespace testing
}  // namespace flutter
