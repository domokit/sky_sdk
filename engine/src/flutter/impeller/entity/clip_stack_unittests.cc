// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/testing/testing.h"
#include "gtest/gtest.h"
#include "impeller/entity/contents/clip_contents.h"
#include "impeller/entity/entity.h"
#include "impeller/entity/entity_pass_clip_stack.h"

namespace impeller {
namespace testing {

TEST(EntityPassClipStackTest, CanPushAndPopEntities) {
  EntityPassClipStack recorder =
      EntityPassClipStack(Rect::MakeLTRB(0, 0, 100, 100));

  EXPECT_TRUE(recorder.GetReplayEntities().empty());

  recorder.RecordClip(ClipContents(Rect::MakeLTRB(0, 0, 100, 100),
                                   /*is_axis_aligned_rect=*/false),
                      Matrix(), {0, 0}, 0, 100, /*is_aa=*/true);

  EXPECT_EQ(recorder.GetReplayEntities().size(), 1u);

  recorder.RecordClip(ClipContents(Rect::MakeLTRB(0, 0, 50.5, 50.5),
                                   /*is_axis_aligned_rect=*/true),
                      Matrix(), {0, 0}, 2, 100, /*is_aa=*/true);

  EXPECT_EQ(recorder.GetReplayEntities().size(), 2u);
  ASSERT_TRUE(recorder.GetReplayEntities()[0].clip_coverage.has_value());
  ASSERT_TRUE(recorder.GetReplayEntities()[1].clip_coverage.has_value());

  // NOLINTBEGIN(bugprone-unchecked-optional-access)
  EXPECT_EQ(recorder.GetReplayEntities()[0].clip_coverage.value(),
            Rect::MakeLTRB(0, 0, 100, 100));
  EXPECT_EQ(recorder.GetReplayEntities()[1].clip_coverage.value(),
            Rect::MakeLTRB(0, 0, 50.5, 50.5));
  // NOLINTEND(bugprone-unchecked-optional-access)

  recorder.RecordRestore({0, 0}, 1);
  EXPECT_EQ(recorder.GetReplayEntities().size(), 1u);

  recorder.RecordRestore({0, 0}, 0);
  EXPECT_TRUE(recorder.GetReplayEntities().empty());
}

TEST(EntityPassClipStackTest, CanPopEntitiesSafely) {
  EntityPassClipStack recorder =
      EntityPassClipStack(Rect::MakeLTRB(0, 0, 100, 100));

  EXPECT_TRUE(recorder.GetReplayEntities().empty());

  recorder.RecordRestore({0, 0}, 0);
  EXPECT_TRUE(recorder.GetReplayEntities().empty());
}

TEST(EntityPassClipStackTest, AppendAndRestoreClipCoverage) {
  EntityPassClipStack recorder =
      EntityPassClipStack(Rect::MakeLTRB(0, 0, 100, 100));

  ASSERT_EQ(recorder.GetClipCoverageLayers().size(), 1u);

  // Push a clip.
  EntityPassClipStack::ClipStateResult result =
      recorder.RecordClip(ClipContents(Rect::MakeLTRB(50, 50, 55.5, 55.5),
                                       /*is_axis_aligned_rect=*/true),
                          Matrix(), {0, 0}, 0, 100, /*is_aa=*/true);
  EXPECT_TRUE(result.should_render);
  EXPECT_TRUE(result.clip_did_change);

  ASSERT_EQ(recorder.GetClipCoverageLayers().size(), 2u);
  EXPECT_EQ(recorder.GetClipCoverageLayers()[1].coverage,
            Rect::MakeLTRB(50, 50, 55.5, 55.5));
  EXPECT_EQ(recorder.GetClipCoverageLayers()[1].clip_height, 1u);
  EXPECT_EQ(recorder.GetReplayEntities().size(), 1u);

  // Restore the clip.
  recorder.RecordRestore({0, 0}, 0);

  ASSERT_EQ(recorder.GetClipCoverageLayers().size(), 1u);
  EXPECT_EQ(recorder.GetClipCoverageLayers()[0].coverage,
            Rect::MakeSize(Size::MakeWH(100, 100)));
  EXPECT_EQ(recorder.GetClipCoverageLayers()[0].clip_height, 0u);
  EXPECT_EQ(recorder.GetReplayEntities().size(), 0u);
}

TEST(EntityPassClipStackTest, AppendAndRestoreClipCoverageNonAA) {
  EntityPassClipStack recorder =
      EntityPassClipStack(Rect::MakeLTRB(0, 0, 100, 100));

  ASSERT_EQ(recorder.GetClipCoverageLayers().size(), 1u);

  // Push a clip.
  EntityPassClipStack::ClipStateResult result =
      recorder.RecordClip(ClipContents(Rect::MakeLTRB(50, 50, 55.4, 55.4),
                                       /*is_axis_aligned_rect=*/true),
                          Matrix(), {0, 0}, 0, 100, /*is_aa=*/false);
  EXPECT_FALSE(result.should_render);
  EXPECT_TRUE(result.clip_did_change);

  ASSERT_EQ(recorder.GetClipCoverageLayers().size(), 2u);
  EXPECT_EQ(recorder.GetClipCoverageLayers()[1].coverage,
            Rect::MakeLTRB(50, 50, 55, 55));
  EXPECT_EQ(recorder.GetClipCoverageLayers()[1].clip_height, 1u);
  EXPECT_EQ(recorder.GetReplayEntities().size(), 1u);

  // Restore the clip.
  recorder.RecordRestore({0, 0}, 0);

  ASSERT_EQ(recorder.GetClipCoverageLayers().size(), 1u);
  EXPECT_EQ(recorder.GetClipCoverageLayers()[0].coverage,
            Rect::MakeSize(Size::MakeWH(100, 100)));
  EXPECT_EQ(recorder.GetClipCoverageLayers()[0].clip_height, 0u);
  EXPECT_EQ(recorder.GetReplayEntities().size(), 0u);
}

// Append two clip coverages, the second is larger the first. This
// should result in the second clip not requiring any update.
TEST(EntityPassClipStackTest, AppendLargerClipCoverage) {
  EntityPassClipStack recorder =
      EntityPassClipStack(Rect::MakeLTRB(0, 0, 100, 100));

  ASSERT_EQ(recorder.GetClipCoverageLayers().size(), 1u);

  // Push a clip.
  EntityPassClipStack::ClipStateResult result =
      recorder.RecordClip(ClipContents(Rect::MakeLTRB(50, 50, 55.5, 55.5),
                                       /*is_axis_aligned_rect=*/true),
                          Matrix(), {0, 0}, 0, 100, /*is_aa=*/true);

  EXPECT_TRUE(result.should_render);
  EXPECT_TRUE(result.clip_did_change);

  // Push a clip with larger coverage than the previous state.
  result = recorder.RecordClip(ClipContents(Rect::MakeLTRB(0, 0, 100.5, 100.5),
                                            /*is_axis_aligned_rect=*/true),
                               Matrix(), {0, 0}, 1, 100, /*is_aa=*/true);

  EXPECT_FALSE(result.should_render);
  EXPECT_FALSE(result.clip_did_change);
}

// Since clip entities return the outer coverage we can only cull axis aligned
// rectangles and intersect clips.
TEST(EntityPassClipStackTest,
     AppendLargerClipCoverageWithDifferenceOrNonSquare) {
  EntityPassClipStack recorder =
      EntityPassClipStack(Rect::MakeLTRB(0, 0, 100, 100));

  ASSERT_EQ(recorder.GetClipCoverageLayers().size(), 1u);

  // Push a clip.
  EntityPassClipStack::ClipStateResult result =
      recorder.RecordClip(ClipContents(Rect::MakeLTRB(50, 50, 55, 55),
                                       /*is_axis_aligned_rect=*/true),
                          Matrix(), {0, 0}, 0, 100, /*is_aa=*/true);

  EXPECT_FALSE(result.should_render);
  EXPECT_TRUE(result.clip_did_change);

  // Push a clip with larger coverage than the previous state.
  result = recorder.RecordClip(ClipContents(Rect::MakeLTRB(0, 0, 100, 100),
                                            /*is_axis_aligned_rect=*/false),
                               Matrix(), {0, 0}, 0, 100, /*is_aa=*/true);

  EXPECT_TRUE(result.should_render);
  EXPECT_TRUE(result.clip_did_change);
}

TEST(EntityPassClipStackTest, AppendDecreasingSizeClipCoverage) {
  EntityPassClipStack recorder =
      EntityPassClipStack(Rect::MakeLTRB(0, 0, 100, 100));

  ASSERT_EQ(recorder.GetClipCoverageLayers().size(), 1u);

  // Push Clips that shrink in size. All should be applied.
  Entity entity;

  for (auto i = 1; i < 20; i++) {
    EntityPassClipStack::ClipStateResult result = recorder.RecordClip(
        ClipContents(Rect::MakeLTRB(i, i, 99.6 - i, 99.6 - i),
                     /*is_axis_aligned_rect=*/true),
        Matrix(), {0, 0}, 0, 100, /*is_aa=*/true);

    EXPECT_TRUE(result.should_render);
    EXPECT_TRUE(result.clip_did_change);
    EXPECT_EQ(recorder.CurrentClipCoverage(),
              Rect::MakeLTRB(i, i, 99.6 - i, 99.6 - i));
  }
}

TEST(EntityPassClipStackTest, AppendIncreasingSizeClipCoverage) {
  EntityPassClipStack recorder =
      EntityPassClipStack(Rect::MakeLTRB(0, 0, 100, 100));

  ASSERT_EQ(recorder.GetClipCoverageLayers().size(), 1u);

  // Push Clips that grow in size. All should be skipped.

  for (auto i = 1; i < 20; i++) {
    EntityPassClipStack::ClipStateResult result = recorder.RecordClip(
        ClipContents(Rect::MakeLTRB(0 - i, 0 - i, 100 + i, 100 + i),
                     /*is_axis_aligned_rect=*/true),
        Matrix(), {0, 0}, 0, 100, /*is_aa=*/true);

    EXPECT_FALSE(result.should_render);
    EXPECT_FALSE(result.clip_did_change);
    EXPECT_EQ(recorder.CurrentClipCoverage(), Rect::MakeLTRB(0, 0, 100, 100));
  }
}

TEST(EntityPassClipStackTest, UnbalancedRestore) {
  EntityPassClipStack recorder =
      EntityPassClipStack(Rect::MakeLTRB(0, 0, 100, 100));

  ASSERT_EQ(recorder.GetClipCoverageLayers().size(), 1u);

  // Restore the clip.
  EntityPassClipStack::ClipStateResult result =
      recorder.RecordRestore(Point(0, 0), 0);
  EXPECT_FALSE(result.should_render);
  EXPECT_FALSE(result.clip_did_change);

  ASSERT_EQ(recorder.GetClipCoverageLayers().size(), 1u);
  EXPECT_EQ(recorder.GetClipCoverageLayers()[0].coverage,
            Rect::MakeSize(Size::MakeWH(100, 100)));
  EXPECT_EQ(recorder.GetClipCoverageLayers()[0].clip_height, 0u);
  EXPECT_EQ(recorder.GetReplayEntities().size(), 0u);
}

TEST(EntityPassClipStackTest, ClipAndRestoreWithSubpasses) {
  EntityPassClipStack recorder =
      EntityPassClipStack(Rect::MakeLTRB(0, 0, 100, 100));

  ASSERT_EQ(recorder.GetClipCoverageLayers().size(), 1u);

  // Push a clip.
  {
    EntityPassClipStack::ClipStateResult result =
        recorder.RecordClip(ClipContents(Rect::MakeLTRB(50, 50, 55.5, 55.5),
                                         /*is_axis_aligned_rect=*/true),
                            Matrix(), {0, 0}, 0, 100, /*is_aa=*/true);

    EXPECT_TRUE(result.should_render);
    EXPECT_TRUE(result.clip_did_change);
  }

  ASSERT_EQ(recorder.GetClipCoverageLayers().size(), 2u);
  EXPECT_EQ(recorder.GetClipCoverageLayers()[1].coverage,
            Rect::MakeLTRB(50, 50, 55.5, 55.5));
  EXPECT_EQ(recorder.GetClipCoverageLayers()[1].clip_height, 1u);
  EXPECT_EQ(recorder.GetReplayEntities().size(), 1u);

  // Begin a subpass.
  recorder.PushSubpass(Rect::MakeLTRB(50, 50, 55, 55), 1);
  ASSERT_EQ(recorder.GetClipCoverageLayers().size(), 1u);
  EXPECT_EQ(recorder.GetClipCoverageLayers()[0].coverage,
            Rect::MakeLTRB(50, 50, 55, 55));

  {
    EntityPassClipStack::ClipStateResult result =
        recorder.RecordClip(ClipContents(Rect::MakeLTRB(54, 54, 54.5, 54.5),
                                         /*is_axis_aligned_rect=*/true),
                            Matrix(), {0, 0}, 0, 100, /*is_aa=*/true);

    EXPECT_TRUE(result.should_render);
    EXPECT_TRUE(result.clip_did_change);
  }

  EXPECT_EQ(recorder.GetClipCoverageLayers()[1].coverage,
            Rect::MakeLTRB(54, 54, 54.5, 54.5));

  // End subpass.
  recorder.PopSubpass();

  EXPECT_EQ(recorder.GetClipCoverageLayers()[1].coverage,
            Rect::MakeLTRB(50, 50, 55.5, 55.5));
}

TEST(EntityPassClipStackTest, ClipAndRestoreWithSubpassesNonAA) {
  EntityPassClipStack recorder =
      EntityPassClipStack(Rect::MakeLTRB(0, 0, 100, 100));

  ASSERT_EQ(recorder.GetClipCoverageLayers().size(), 1u);

  // Push a clip.
  {
    EntityPassClipStack::ClipStateResult result =
        recorder.RecordClip(ClipContents(Rect::MakeLTRB(50, 50, 55.4, 55.4),
                                         /*is_axis_aligned_rect=*/true),
                            Matrix(), {0, 0}, 0, 100, /*is_aa=*/false);

    EXPECT_FALSE(result.should_render);
    EXPECT_TRUE(result.clip_did_change);
  }

  ASSERT_EQ(recorder.GetClipCoverageLayers().size(), 2u);
  EXPECT_EQ(recorder.GetClipCoverageLayers()[1].coverage,
            Rect::MakeLTRB(50, 50, 55.0, 55.0));
  EXPECT_EQ(recorder.GetClipCoverageLayers()[1].clip_height, 1u);
  EXPECT_EQ(recorder.GetReplayEntities().size(), 1u);

  // Begin a subpass.
  recorder.PushSubpass(Rect::MakeLTRB(50, 50, 55, 55), 1);
  ASSERT_EQ(recorder.GetClipCoverageLayers().size(), 1u);
  EXPECT_EQ(recorder.GetClipCoverageLayers()[0].coverage,
            Rect::MakeLTRB(50, 50, 55, 55));

  {
    EntityPassClipStack::ClipStateResult result =
        recorder.RecordClip(ClipContents(Rect::MakeLTRB(54, 54, 55.4, 55.4),
                                         /*is_axis_aligned_rect=*/true),
                            Matrix(), {0, 0}, 0, 100, /*is_aa=*/false);

    EXPECT_FALSE(result.should_render);
    EXPECT_TRUE(result.clip_did_change);
  }

  EXPECT_EQ(recorder.GetClipCoverageLayers()[1].coverage,
            Rect::MakeLTRB(54, 54, 55.0, 55.0));

  // End subpass.
  recorder.PopSubpass();

  EXPECT_EQ(recorder.GetClipCoverageLayers()[1].coverage,
            Rect::MakeLTRB(50, 50, 55, 55));
}

}  // namespace testing
}  // namespace impeller
