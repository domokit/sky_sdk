// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import <OCMock/OCMock.h>
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterMutatorView.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterView.h"

#include "third_party/googletest/googletest/include/gtest/gtest.h"

#import "flutter/shell/platform/darwin/macos/framework/Source/NSView+ClipsToBounds.h"

@interface FlutterMutatorView (Private)

@property(readonly, nonatomic, nonnull) NSMutableArray<NSView*>* pathClipViews;
@property(readonly, nonatomic, nullable) NSView* platformViewContainer;

@end

static constexpr float kMaxErr = 1e-10;

namespace {
void ApplyFlutterLayer(FlutterMutatorView* view,
                       FlutterSize size,
                       const std::vector<FlutterPlatformViewMutation>& mutations) {
  flutter::PlatformViewLayer layer(0,  // identifier
                                   mutations,
                                   // Offset is ignored by mutator view, the bounding rect is
                                   // determined by width and transform.
                                   FlutterPoint{0, 0},  // offset
                                   size);

  [view applyFlutterLayer:&layer];
}

// Expect that each element within two CATransform3Ds is within an error bound.
//
// In order to avoid architecture-specific floating point differences we don't check for exact
// equality using, for example, CATransform3DEqualToTransform.
void ExpectTransform3DEqual(const CATransform3D& t, const CATransform3D& u) {
  EXPECT_NEAR(t.m11, u.m11, kMaxErr);
  EXPECT_NEAR(t.m12, u.m12, kMaxErr);
  EXPECT_NEAR(t.m13, u.m13, kMaxErr);
  EXPECT_NEAR(t.m14, u.m14, kMaxErr);

  EXPECT_NEAR(t.m21, u.m21, kMaxErr);
  EXPECT_NEAR(t.m22, u.m22, kMaxErr);
  EXPECT_NEAR(t.m23, u.m23, kMaxErr);
  EXPECT_NEAR(t.m24, u.m24, kMaxErr);

  EXPECT_NEAR(t.m31, u.m31, kMaxErr);
  EXPECT_NEAR(t.m32, u.m32, kMaxErr);
  EXPECT_NEAR(t.m33, u.m33, kMaxErr);
  EXPECT_NEAR(t.m34, u.m34, kMaxErr);

  EXPECT_NEAR(t.m41, u.m41, kMaxErr);
  EXPECT_NEAR(t.m42, u.m42, kMaxErr);
  EXPECT_NEAR(t.m43, u.m43, kMaxErr);
  EXPECT_NEAR(t.m44, u.m44, kMaxErr);
}
}  // namespace

TEST(FlutterMutatorViewTest, BasicFrameIsCorrect) {
  NSView* platformView = [[NSView alloc] init];
  FlutterMutatorView* mutatorView = [[FlutterMutatorView alloc] initWithPlatformView:platformView];

  EXPECT_EQ(mutatorView.platformView, platformView);

  std::vector<FlutterPlatformViewMutation> mutations{
      {
          .type = kFlutterPlatformViewMutationTypeTransformation,
          .transformation =
              FlutterTransformation{
                  .scaleX = 1,
                  .transX = 100,
                  .scaleY = 1,
                  .transY = 50,
              },
      },
  };

  ApplyFlutterLayer(mutatorView, FlutterSize{30, 20}, mutations);

  EXPECT_TRUE(CGRectEqualToRect(mutatorView.frame, CGRectMake(100, 50, 30, 20)));
  EXPECT_TRUE(CGRectEqualToRect(platformView.frame, CGRectMake(0, 0, 30, 20)));
  EXPECT_EQ(mutatorView.pathClipViews.count, 0ull);
  EXPECT_NE(mutatorView.platformViewContainer, nil);
}

TEST(FlutterMutatorViewTest, ClipsToBounds) {
  NSView* platformView = [[NSView alloc] init];
  FlutterMutatorView* mutatorView = [[FlutterMutatorView alloc] initWithPlatformView:platformView];
  EXPECT_TRUE(mutatorView.clipsToBounds);
}

TEST(FlutterMutatorViewTest, TransformedFrameIsCorrect) {
  NSView* platformView = [[NSView alloc] init];
  FlutterMutatorView* mutatorView = [[FlutterMutatorView alloc] initWithPlatformView:platformView];
  NSView* mutatorViewParent = [[NSView alloc] init];
  mutatorViewParent.wantsLayer = YES;
  mutatorViewParent.layer.contentsScale = 2.0;
  [mutatorViewParent addSubview:mutatorView];

  std::vector<FlutterPlatformViewMutation> mutations{
      {
          .type = kFlutterPlatformViewMutationTypeTransformation,
          .transformation =
              FlutterTransformation{
                  .scaleX = 2,
                  .scaleY = 2,
              },
      },
      {
          .type = kFlutterPlatformViewMutationTypeTransformation,
          .transformation =
              FlutterTransformation{
                  .scaleX = 1,
                  .transX = 100,
                  .scaleY = 1,
                  .transY = 50,
              },
      },
      {
          .type = kFlutterPlatformViewMutationTypeTransformation,
          .transformation =
              FlutterTransformation{
                  .scaleX = 1.5,
                  .transX = -7.5,
                  .scaleY = 1.5,
                  .transY = -5,
              },
      },
  };

  // PlatformView size form engine comes in physical pixels
  ApplyFlutterLayer(mutatorView, FlutterSize{30 * 2, 20 * 2}, mutations);
  EXPECT_TRUE(CGRectEqualToRect(mutatorView.frame, CGRectMake(92.5, 45, 45, 30)));
  EXPECT_TRUE(CGRectEqualToRect(platformView.frame, CGRectMake(0, 0, 30, 20)));

  ExpectTransform3DEqual(mutatorView.platformViewContainer.layer.sublayerTransform,
                         CATransform3DMakeScale(1.5, 1.5, 1));
}

TEST(FlutterMutatorViewTest, FrameWithLooseClipIsCorrect) {
  NSView* platformView = [[NSView alloc] init];
  FlutterMutatorView* mutatorView = [[FlutterMutatorView alloc] initWithPlatformView:platformView];

  EXPECT_EQ(mutatorView.platformView, platformView);

  std::vector<FlutterPlatformViewMutation> mutations{
      {
          .type = kFlutterPlatformViewMutationTypeClipRect,
          .clip_rect = FlutterRect{80, 40, 200, 100},
      },
      {
          .type = kFlutterPlatformViewMutationTypeTransformation,
          .transformation =
              FlutterTransformation{
                  .scaleX = 1,
                  .transX = 100,
                  .scaleY = 1,
                  .transY = 50,
              },
      },
  };

  ApplyFlutterLayer(mutatorView, FlutterSize{30, 20}, mutations);

  EXPECT_TRUE(CGRectEqualToRect(mutatorView.frame, CGRectMake(100, 50, 30, 20)));
  EXPECT_TRUE(CGRectEqualToRect(platformView.frame, CGRectMake(0, 0, 30, 20)));
}

TEST(FlutterMutatorViewTest, FrameWithTightClipIsCorrect) {
  NSView* platformView = [[NSView alloc] init];
  FlutterMutatorView* mutatorView = [[FlutterMutatorView alloc] initWithPlatformView:platformView];

  EXPECT_EQ(mutatorView.platformView, platformView);

  std::vector<FlutterPlatformViewMutation> mutations{
      {
          .type = kFlutterPlatformViewMutationTypeClipRect,
          .clip_rect = FlutterRect{80, 40, 200, 100},
      },
      {
          .type = kFlutterPlatformViewMutationTypeClipRect,
          .clip_rect = FlutterRect{110, 55, 120, 65},
      },
      {
          .type = kFlutterPlatformViewMutationTypeTransformation,
          .transformation =
              FlutterTransformation{
                  .scaleX = 1,
                  .transX = 100,
                  .scaleY = 1,
                  .transY = 50,
              },
      },
  };

  ApplyFlutterLayer(mutatorView, FlutterSize{30, 20}, mutations);

  EXPECT_TRUE(CGRectEqualToRect(mutatorView.frame, CGRectMake(110, 55, 10, 10)));
  EXPECT_TRUE(
      CGRectEqualToRect(mutatorView.subviews.firstObject.frame, CGRectMake(-10, -5, 30, 20)));
  EXPECT_TRUE(CGRectEqualToRect(platformView.frame, CGRectMake(0, 0, 30, 20)));
}

TEST(FlutterMutatorViewTest, FrameWithTightClipAndTransformIsCorrect) {
  NSView* platformView = [[NSView alloc] init];
  FlutterMutatorView* mutatorView = [[FlutterMutatorView alloc] initWithPlatformView:platformView];
  NSView* mutatorViewParent = [[NSView alloc] init];
  mutatorViewParent.wantsLayer = YES;
  mutatorViewParent.layer.contentsScale = 2.0;
  [mutatorViewParent addSubview:mutatorView];

  std::vector<FlutterPlatformViewMutation> mutations{
      {
          .type = kFlutterPlatformViewMutationTypeTransformation,
          .transformation =
              FlutterTransformation{
                  .scaleX = 2,
                  .scaleY = 2,
              },
      },
      {
          .type = kFlutterPlatformViewMutationTypeClipRect,
          .clip_rect = FlutterRect{80, 40, 200, 100},
      },
      {
          .type = kFlutterPlatformViewMutationTypeClipRect,
          .clip_rect = FlutterRect{110, 55, 120, 65},
      },
      {
          .type = kFlutterPlatformViewMutationTypeTransformation,
          .transformation =
              FlutterTransformation{
                  .scaleX = 1,
                  .transX = 100,
                  .scaleY = 1,
                  .transY = 50,
              },
      },
      {
          .type = kFlutterPlatformViewMutationTypeTransformation,
          .transformation =
              FlutterTransformation{
                  .scaleX = 1.5,
                  .transX = -7.5,
                  .scaleY = 1.5,
                  .transY = -5,
              },
      },
  };

  ApplyFlutterLayer(mutatorView, FlutterSize{30 * 2, 20 * 2}, mutations);

  EXPECT_TRUE(CGRectEqualToRect(mutatorView.frame, CGRectMake(110, 55, 10, 10)));
  EXPECT_TRUE(
      CGRectEqualToRect(mutatorView.subviews.firstObject.frame, CGRectMake(-17.5, -10, 45, 30)));
  EXPECT_TRUE(CGRectEqualToRect(platformView.frame, CGRectMake(0, 0, 30, 20)));
}

// Rounded rectangle without hitting the corner
TEST(FlutterMutatorViewTest, RoundRectClipsToSimpleRectangle) {
  NSView* platformView = [[NSView alloc] init];
  FlutterMutatorView* mutatorView = [[FlutterMutatorView alloc] initWithPlatformView:platformView];

  std::vector<FlutterPlatformViewMutation> mutations{
      {
          .type = kFlutterPlatformViewMutationTypeClipRoundedRect,
          .clip_rounded_rect =
              FlutterRoundedRect{
                  .rect = FlutterRect{110, 30, 120, 90},
                  .upper_left_corner_radius = FlutterSize{10, 10},
                  .upper_right_corner_radius = FlutterSize{10, 10},
                  .lower_right_corner_radius = FlutterSize{10, 10},
                  .lower_left_corner_radius = FlutterSize{10, 10},
              },
      },
      {
          .type = kFlutterPlatformViewMutationTypeTransformation,
          .transformation =
              FlutterTransformation{
                  .scaleX = 1,
                  .transX = 100,
                  .scaleY = 1,
                  .transY = 50,
              },
      },
  };

  ApplyFlutterLayer(mutatorView, FlutterSize{30, 20}, mutations);

  EXPECT_TRUE(CGRectEqualToRect(mutatorView.frame, CGRectMake(110, 50, 10, 20)));
  EXPECT_TRUE(
      CGRectEqualToRect(mutatorView.subviews.firstObject.frame, CGRectMake(-10, 0, 30, 20)));
  EXPECT_TRUE(CGRectEqualToRect(platformView.frame, CGRectMake(0, 0, 30, 20)));
  EXPECT_EQ(mutatorView.pathClipViews.count, 0ul);
}

// Ensure that the mutator view, clip views, and container all use a flipped y axis. The transforms
// sent from the framework assume this, and so aside from the consistency with every other embedder,
// we can avoid a lot of extra math.
TEST(FlutterMutatorViewTest, ViewsSetIsFlipped) {
  NSView* platformView = [[NSView alloc] init];
  FlutterMutatorView* mutatorView = [[FlutterMutatorView alloc] initWithPlatformView:platformView];

  std::vector<FlutterPlatformViewMutation> mutations{
      {
          .type = kFlutterPlatformViewMutationTypeClipRoundedRect,
          .clip_rounded_rect =
              FlutterRoundedRect{
                  .rect = FlutterRect{110, 60, 150, 150},
                  .upper_left_corner_radius = FlutterSize{10, 10},
                  .upper_right_corner_radius = FlutterSize{10, 10},
                  .lower_right_corner_radius = FlutterSize{10, 10},
                  .lower_left_corner_radius = FlutterSize{10, 10},
              },
      },
      {
          .type = kFlutterPlatformViewMutationTypeTransformation,
          .transformation =
              FlutterTransformation{
                  .scaleX = 1,
                  .transX = 100,
                  .scaleY = 1,
                  .transY = 50,
              },
      },
  };

  ApplyFlutterLayer(mutatorView, FlutterSize{30, 20}, mutations);

  EXPECT_TRUE(mutatorView.isFlipped);
  ASSERT_EQ(mutatorView.pathClipViews.count, 1ul);
  EXPECT_TRUE(mutatorView.pathClipViews.firstObject.isFlipped);
  EXPECT_TRUE(mutatorView.platformViewContainer.isFlipped);
}

TEST(FlutterMutatorViewTest, RectsClipsToPathWhenRotated) {
  NSView* platformView = [[NSView alloc] init];
  FlutterMutatorView* mutatorView = [[FlutterMutatorView alloc] initWithPlatformView:platformView];
  std::vector<FlutterPlatformViewMutation> mutations{
      {
          .type = kFlutterPlatformViewMutationTypeTransformation,
          // Roation M_PI / 8
          .transformation =
              FlutterTransformation{
                  .scaleX = 0.9238795325112867,
                  .skewX = -0.3826834323650898,
                  .skewY = 0.3826834323650898,
                  .scaleY = 0.9238795325112867,
              },
      },
      {
          .type = kFlutterPlatformViewMutationTypeClipRect,
          .clip_rect = FlutterRect{110, 60, 150, 150},
      },
      {
          .type = kFlutterPlatformViewMutationTypeTransformation,
          .transformation =
              FlutterTransformation{
                  .scaleX = 1,
                  .transX = 100,
                  .scaleY = 1,
                  .transY = 50,
              },
      },
  };
  ApplyFlutterLayer(mutatorView, FlutterSize{30, 20}, mutations);
  EXPECT_EQ(mutatorView.pathClipViews.count, 1ul);
  EXPECT_NEAR(mutatorView.platformViewContainer.frame.size.width, 35.370054622640396, kMaxErr);
  EXPECT_NEAR(mutatorView.platformViewContainer.frame.size.height, 29.958093621178421, kMaxErr);
}

TEST(FlutterMutatorViewTest, RoundRectClipsToPath) {
  NSView* platformView = [[NSView alloc] init];
  FlutterMutatorView* mutatorView = [[FlutterMutatorView alloc] initWithPlatformView:platformView];

  std::vector<FlutterPlatformViewMutation> mutations{
      {
          .type = kFlutterPlatformViewMutationTypeClipRoundedRect,
          .clip_rounded_rect =
              FlutterRoundedRect{
                  .rect = FlutterRect{110, 60, 150, 150},
                  .upper_left_corner_radius = FlutterSize{10, 10},
                  .upper_right_corner_radius = FlutterSize{10, 10},
                  .lower_right_corner_radius = FlutterSize{10, 10},
                  .lower_left_corner_radius = FlutterSize{10, 10},
              },
      },
      {
          .type = kFlutterPlatformViewMutationTypeTransformation,
          .transformation =
              FlutterTransformation{
                  .scaleX = 1,
                  .transX = 100,
                  .scaleY = 1,
                  .transY = 50,
              },
      },
  };

  ApplyFlutterLayer(mutatorView, FlutterSize{30, 20}, mutations);

  EXPECT_TRUE(CGRectEqualToRect(mutatorView.frame, CGRectMake(110, 60, 20, 10)));
  EXPECT_TRUE(
      CGRectEqualToRect(mutatorView.subviews.firstObject.frame, CGRectMake(-10, -10, 30, 20)));
  EXPECT_TRUE(CGRectEqualToRect(platformView.frame, CGRectMake(0, 0, 30, 20)));
  EXPECT_EQ(mutatorView.pathClipViews.count, 1ul);
  ExpectTransform3DEqual(mutatorView.pathClipViews.firstObject.layer.mask.transform,
                         CATransform3DMakeTranslation(-100, -50, 0));
}

TEST(FlutterMutatorViewTest, PathClipViewsAreAddedAndRemoved) {
  NSView* platformView = [[NSView alloc] init];
  FlutterMutatorView* mutatorView = [[FlutterMutatorView alloc] initWithPlatformView:platformView];

  std::vector<FlutterPlatformViewMutation> mutations{
      {
          .type = kFlutterPlatformViewMutationTypeTransformation,
          .transformation =
              FlutterTransformation{
                  .scaleX = 1,
                  .transX = 100,
                  .scaleY = 1,
                  .transY = 50,
              },
      },
  };

  ApplyFlutterLayer(mutatorView, FlutterSize{30, 20}, mutations);

  EXPECT_TRUE(CGRectEqualToRect(mutatorView.frame, CGRectMake(100, 50, 30, 20)));
  EXPECT_EQ(mutatorView.pathClipViews.count, 0ull);

  std::vector<FlutterPlatformViewMutation> mutations2{
      {
          .type = kFlutterPlatformViewMutationTypeClipRoundedRect,
          .clip_rounded_rect =
              FlutterRoundedRect{
                  .rect = FlutterRect{110, 60, 150, 150},
                  .upper_left_corner_radius = FlutterSize{10, 10},
                  .upper_right_corner_radius = FlutterSize{10, 10},
                  .lower_right_corner_radius = FlutterSize{10, 10},
                  .lower_left_corner_radius = FlutterSize{10, 10},
              },
      },
      {
          .type = kFlutterPlatformViewMutationTypeTransformation,
          .transformation =
              FlutterTransformation{
                  .scaleX = 1,
                  .transX = 100,
                  .scaleY = 1,
                  .transY = 50,
              },
      },
  };

  ApplyFlutterLayer(mutatorView, FlutterSize{30, 20}, mutations2);

  EXPECT_TRUE(CGRectEqualToRect(mutatorView.frame, CGRectMake(110, 60, 20, 10)));
  EXPECT_TRUE(
      CGRectEqualToRect(mutatorView.subviews.firstObject.frame, CGRectMake(-10, -10, 30, 20)));
  EXPECT_TRUE(CGRectEqualToRect(platformView.frame, CGRectMake(0, 0, 30, 20)));
  EXPECT_EQ(mutatorView.pathClipViews.count, 1ul);

  EXPECT_EQ(platformView.superview, mutatorView.platformViewContainer);
  EXPECT_EQ(mutatorView.platformViewContainer.superview, mutatorView.pathClipViews[0]);
  EXPECT_EQ(mutatorView.pathClipViews[0].superview, mutatorView);

  std::vector<FlutterPlatformViewMutation> mutations3{
      {
          .type = kFlutterPlatformViewMutationTypeClipRoundedRect,
          .clip_rounded_rect =
              FlutterRoundedRect{
                  .rect = FlutterRect{110, 55, 150, 150},
                  .upper_left_corner_radius = FlutterSize{10, 10},
                  .upper_right_corner_radius = FlutterSize{10, 10},
                  .lower_right_corner_radius = FlutterSize{10, 10},
                  .lower_left_corner_radius = FlutterSize{10, 10},
              },
      },
      {
          .type = kFlutterPlatformViewMutationTypeClipRoundedRect,
          .clip_rounded_rect =
              FlutterRoundedRect{
                  .rect = FlutterRect{30, 30, 120, 65},
                  .upper_left_corner_radius = FlutterSize{10, 10},
                  .upper_right_corner_radius = FlutterSize{10, 10},
                  .lower_right_corner_radius = FlutterSize{10, 10},
                  .lower_left_corner_radius = FlutterSize{10, 10},
              },
      },
      {
          .type = kFlutterPlatformViewMutationTypeTransformation,
          .transformation =
              FlutterTransformation{
                  .scaleX = 1,
                  .transX = 100,
                  .scaleY = 1,
                  .transY = 50,
              },
      },
  };

  ApplyFlutterLayer(mutatorView, FlutterSize{30, 20}, mutations3);

  EXPECT_TRUE(CGRectEqualToRect(mutatorView.frame, CGRectMake(110, 55, 10, 10)));
  EXPECT_TRUE(
      CGRectEqualToRect(mutatorView.subviews.firstObject.frame, CGRectMake(-10, -5, 30, 20)));
  EXPECT_EQ(mutatorView.pathClipViews.count, 2ul);

  EXPECT_EQ(platformView.superview, mutatorView.platformViewContainer);
  EXPECT_EQ(mutatorView.platformViewContainer.superview, mutatorView.pathClipViews[1]);
  EXPECT_EQ(mutatorView.pathClipViews[1].superview, mutatorView.pathClipViews[0]);
  EXPECT_EQ(mutatorView.pathClipViews[0].superview, mutatorView);

  ApplyFlutterLayer(mutatorView, FlutterSize{30, 20}, mutations2);

  EXPECT_TRUE(CGRectEqualToRect(mutatorView.frame, CGRectMake(110, 60, 20, 10)));
  EXPECT_TRUE(
      CGRectEqualToRect(mutatorView.subviews.firstObject.frame, CGRectMake(-10, -10, 30, 20)));
  EXPECT_TRUE(CGRectEqualToRect(platformView.frame, CGRectMake(0, 0, 30, 20)));
  EXPECT_EQ(mutatorView.pathClipViews.count, 1ul);

  EXPECT_EQ(platformView.superview, mutatorView.platformViewContainer);
  EXPECT_EQ(mutatorView.platformViewContainer.superview, mutatorView.pathClipViews[0]);
  EXPECT_EQ(mutatorView.pathClipViews[0].superview, mutatorView);

  ApplyFlutterLayer(mutatorView, FlutterSize{30, 20}, mutations);

  EXPECT_TRUE(CGRectEqualToRect(mutatorView.frame, CGRectMake(100, 50, 30, 20)));
  EXPECT_EQ(mutatorView.pathClipViews.count, 0ull);
}

TEST(FlutterMutatorViewTest, HitTestIgnoreRegion) {
  NSView* platformView = [[NSView alloc] init];
  FlutterMutatorView* mutatorView = [[FlutterMutatorView alloc] initWithPlatformView:platformView];
  ApplyFlutterLayer(mutatorView, FlutterSize{100, 100}, {});
  EXPECT_EQ([mutatorView hitTest:NSMakePoint(10, 10)], platformView);
  EXPECT_EQ([mutatorView hitTest:NSMakePoint(50, 10)], platformView);

  [mutatorView resetHitTestRegion];
  [mutatorView addHitTestIgnoreRegion:CGRectMake(0, 0, 50, 50)];
  [mutatorView addHitTestIgnoreRegion:CGRectMake(50, 50, 50, 50)];

  EXPECT_EQ([mutatorView hitTest:NSMakePoint(10, 10)], nil);
  EXPECT_EQ([mutatorView hitTest:NSMakePoint(49, 10)], nil);
  EXPECT_EQ([mutatorView hitTest:NSMakePoint(10, 49)], nil);
  EXPECT_EQ([mutatorView hitTest:NSMakePoint(50, 50)], nil);
  EXPECT_EQ([mutatorView hitTest:NSMakePoint(50, 10)], platformView);
  EXPECT_EQ([mutatorView hitTest:NSMakePoint(10, 50)], platformView);

  [mutatorView resetHitTestRegion];
  EXPECT_EQ([mutatorView hitTest:NSMakePoint(10, 10)], platformView);
  EXPECT_EQ([mutatorView hitTest:NSMakePoint(49, 10)], platformView);
  EXPECT_EQ([mutatorView hitTest:NSMakePoint(10, 49)], platformView);
  EXPECT_EQ([mutatorView hitTest:NSMakePoint(50, 50)], platformView);
  EXPECT_EQ([mutatorView hitTest:NSMakePoint(50, 10)], platformView);
  EXPECT_EQ([mutatorView hitTest:NSMakePoint(10, 50)], platformView);
}

TEST(FlutterMutatorViewTest, ReparentingPlatformView) {
  NSView* platformView = [[NSView alloc] init];
  FlutterMutatorView* mutatorView = [[FlutterMutatorView alloc] initWithPlatformView:platformView];
  ApplyFlutterLayer(mutatorView, FlutterSize{100, 100}, {});
  EXPECT_TRUE(platformView.superview == mutatorView.platformViewContainer);
  EXPECT_TRUE(CGRectEqualToRect(platformView.frame, CGRectMake(0, 0, 100, 100)));

  // Reparent platform view and replace it with placeholder (mimicking WKWebKit going full screen)
  NSView* newParent = [[NSView alloc] init];
  [newParent addSubview:platformView];
  platformView.frame = CGRectMake(10, 10, 200, 200);

  NSView* placeholderView = [[NSView alloc] init];
  [mutatorView.platformViewContainer addSubview:placeholderView];
  ApplyFlutterLayer(mutatorView, FlutterSize{100, 100}, {});

  // Platform view should not be touched but the replacement view should be properly positioned.
  EXPECT_TRUE(platformView.superview == newParent);
  EXPECT_TRUE(CGRectEqualToRect(platformView.frame, CGRectMake(10, 10, 200, 200)));
  EXPECT_TRUE(CGRectEqualToRect(placeholderView.frame, CGRectMake(0, 0, 100, 100)));
}

@interface FlutterCursorCoordinatorTest : NSObject

@end

@implementation FlutterCursorCoordinatorTest
- (void)testCoordinatorEventWithinFlutterContent {
  id flutterView = OCMClassMock([FlutterView class]);
  FlutterCursorCoordinator* coordinator =
      [[FlutterCursorCoordinator alloc] initWithFlutterView:flutterView];
  {
    id platformView = OCMClassMock([NSView class]);
    OCMStub([flutterView cursorUpdate:[OCMArg any]]);
    id mutatorView = OCMStrictClassMock([FlutterMutatorView class]);
    OCMStub([mutatorView platformView]).andReturn(platformView);
    CGPoint location = NSMakePoint(50, 50);
    OCMStub([mutatorView convertPoint:location fromView:[OCMArg any]]).andReturn(location);
    NSEvent* event = [NSEvent mouseEventWithType:NSEventTypeMouseMoved
                                        location:location
                                   modifierFlags:0
                                       timestamp:0
                                    windowNumber:0
                                         context:nil
                                     eventNumber:0
                                      clickCount:0
                                        pressure:0];
    [coordinator processMouseMoveEvent:event
                        forMutatorView:mutatorView
                         overlayRegion:{CGRectMake(0, 0, 100, 100)}];
    OCMVerify([flutterView cursorUpdate:event]);
  }
  {
    id platformView = OCMClassMock([NSView class]);
    // Make sure once event is handled the coordinator will not send cursorUpdate again.
    OCMReject([flutterView cursorUpdate:[OCMArg any]]);
    id mutatorView = OCMStrictClassMock([FlutterMutatorView class]);
    OCMStub([mutatorView platformView]).andReturn(platformView);
    CGPoint location = NSMakePoint(50, 50);
    OCMStub([mutatorView convertPoint:location fromView:[OCMArg any]]).andReturn(location);
    NSEvent* event = [NSEvent mouseEventWithType:NSEventTypeMouseMoved
                                        location:location
                                   modifierFlags:0
                                       timestamp:0
                                    windowNumber:0
                                         context:nil
                                     eventNumber:0
                                      clickCount:0
                                        pressure:0];
    [coordinator processMouseMoveEvent:event
                        forMutatorView:mutatorView
                         overlayRegion:{CGRectMake(0, 0, 100, 100)}];
  }
  EXPECT_TRUE(coordinator.cleanupScheduled);
  [[NSRunLoop mainRunLoop] runUntilDate:[NSDate dateWithTimeIntervalSinceNow:0.1]];
  EXPECT_FALSE(coordinator.cleanupScheduled);
}

- (void)testCoordinatorEventOutsideFlutterContent {
  id flutterView = OCMClassMock([FlutterView class]);
  OCMReject([flutterView cursorUpdate:[OCMArg any]]);
  FlutterCursorCoordinator* coordinator =
      [[FlutterCursorCoordinator alloc] initWithFlutterView:flutterView];
  id platformViewWindow = OCMClassMock([NSWindow class]);
  {
    id platformView = OCMClassMock([NSView class]);
    OCMStub([platformViewWindow invalidateCursorRectsForView:platformView]);
    OCMStub([platformView window]).andReturn(platformViewWindow);
    OCMStub([flutterView cursorUpdate:[OCMArg any]]);
    id mutatorView = OCMStrictClassMock([FlutterMutatorView class]);
    OCMStub([mutatorView platformView]).andReturn(platformView);
    CGPoint location = NSMakePoint(150, 150);
    OCMStub([mutatorView convertPoint:location fromView:[OCMArg any]]).andReturn(location);
    NSEvent* event = [NSEvent mouseEventWithType:NSEventTypeMouseMoved
                                        location:location
                                   modifierFlags:0
                                       timestamp:0
                                    windowNumber:0
                                         context:nil
                                     eventNumber:0
                                      clickCount:0
                                        pressure:0];
    [coordinator processMouseMoveEvent:event
                        forMutatorView:mutatorView
                         overlayRegion:{CGRectMake(0, 0, 100, 100)}];
    OCMVerify([platformViewWindow invalidateCursorRectsForView:platformView]);
  }
  {
    // Make sure this is not called again for subsequent invocation during same run loop turn.
    OCMReject([platformViewWindow invalidateCursorRectsForView:[OCMArg any]]);

    id platformView = OCMClassMock([NSView class]);
    OCMStub([platformViewWindow invalidateCursorRectsForView:platformView]);
    OCMStub([platformView window]).andReturn(platformViewWindow);
    OCMStub([flutterView cursorUpdate:[OCMArg any]]);
    id mutatorView = OCMStrictClassMock([FlutterMutatorView class]);
    OCMStub([mutatorView platformView]).andReturn(platformView);
    CGPoint location = NSMakePoint(150, 150);
    OCMStub([mutatorView convertPoint:location fromView:[OCMArg any]]).andReturn(location);
    NSEvent* event = [NSEvent mouseEventWithType:NSEventTypeMouseMoved
                                        location:location
                                   modifierFlags:0
                                       timestamp:0
                                    windowNumber:0
                                         context:nil
                                     eventNumber:0
                                      clickCount:0
                                        pressure:0];
    [coordinator processMouseMoveEvent:event
                        forMutatorView:mutatorView
                         overlayRegion:{CGRectMake(0, 0, 100, 100)}];
  }
  EXPECT_TRUE(coordinator.cleanupScheduled);
  [[NSRunLoop mainRunLoop] runUntilDate:[NSDate dateWithTimeIntervalSinceNow:0.1]];
  EXPECT_FALSE(coordinator.cleanupScheduled);

  // Check that invalidateCursorRectsForView is called again
  platformViewWindow = OCMClassMock([NSWindow class]);
  {
    id platformView = OCMClassMock([NSView class]);
    OCMStub([platformViewWindow invalidateCursorRectsForView:platformView]);
    OCMStub([platformView window]).andReturn(platformViewWindow);
    OCMStub([flutterView cursorUpdate:[OCMArg any]]);
    id mutatorView = OCMStrictClassMock([FlutterMutatorView class]);
    OCMStub([mutatorView platformView]).andReturn(platformView);
    CGPoint location = NSMakePoint(150, 150);
    OCMStub([mutatorView convertPoint:location fromView:[OCMArg any]]).andReturn(location);
    NSEvent* event = [NSEvent mouseEventWithType:NSEventTypeMouseMoved
                                        location:location
                                   modifierFlags:0
                                       timestamp:0
                                    windowNumber:0
                                         context:nil
                                     eventNumber:0
                                      clickCount:0
                                        pressure:0];
    [coordinator processMouseMoveEvent:event
                        forMutatorView:mutatorView
                         overlayRegion:{CGRectMake(0, 0, 100, 100)}];
    OCMVerify([platformViewWindow invalidateCursorRectsForView:platformView]);
  }
  [[NSRunLoop mainRunLoop] runUntilDate:[NSDate dateWithTimeIntervalSinceNow:0.1]];
}
@end

TEST(FlutterMutatorViewTest, CursorCoordinator) {
  [[[FlutterCursorCoordinatorTest alloc] init] testCoordinatorEventWithinFlutterContent];
  [[[FlutterCursorCoordinatorTest alloc] init] testCoordinatorEventOutsideFlutterContent];
}