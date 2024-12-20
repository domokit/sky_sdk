// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import <Cocoa/Cocoa.h>
#import <Metal/Metal.h>

#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterSurface.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterSurfaceManager.h"

#include "flutter/testing/testing.h"
#include "gtest/gtest.h"

@interface TestView : NSView <FlutterSurfaceManagerDelegate>

@property(readwrite, nonatomic) CGSize presentedFrameSize;
- (nonnull instancetype)init;

@end

@implementation TestView

- (instancetype)init {
  self = [super initWithFrame:NSZeroRect];
  if (self) {
    [self setWantsLayer:YES];
    self.layer.contentsScale = 2.0;
  }
  return self;
}

- (void)onPresent:(CGSize)frameSize withBlock:(nonnull dispatch_block_t)block {
  self.presentedFrameSize = frameSize;
  block();
}

@end

namespace flutter::testing {

static FlutterSurfaceManager* CreateSurfaceManager(TestView* testView) {
  id<MTLDevice> device = MTLCreateSystemDefaultDevice();
  id<MTLCommandQueue> commandQueue = [device newCommandQueue];
  CALayer* layer = reinterpret_cast<CALayer*>(testView.layer);
  return [[FlutterSurfaceManager alloc] initWithDevice:device
                                          commandQueue:commandQueue
                                                 layer:layer
                                              delegate:testView];
}

static FlutterSurfacePresentInfo* CreatePresentInfo(
    FlutterSurface* surface,
    CGPoint offset = CGPointZero,
    size_t index = 0,
    const std::vector<FlutterRect>& paintRegion = {}) {
  FlutterSurfacePresentInfo* res = [[FlutterSurfacePresentInfo alloc] init];
  res.surface = surface;
  res.offset = offset;
  res.zIndex = index;
  res.paintRegion = paintRegion;
  return res;
}

TEST(FlutterSurfaceManager, MetalTextureSizeMatchesSurfaceSize) {
  TestView* testView = [[TestView alloc] init];
  FlutterSurfaceManager* surfaceManager = CreateSurfaceManager(testView);

  // Get back buffer, lookup should work for borrowed surfaces util present.
  auto surface = [surfaceManager surfaceForSize:CGSizeMake(100, 50)];
  auto texture = surface.asFlutterMetalTexture;
  id<MTLTexture> metalTexture = (__bridge id)texture.texture;
  EXPECT_EQ(metalTexture.width, 100ul);
  EXPECT_EQ(metalTexture.height, 50ul);
  texture.destruction_callback(texture.user_data);
}

TEST(FlutterSurfaceManager, TestSurfaceLookupFromTexture) {
  TestView* testView = [[TestView alloc] init];
  FlutterSurfaceManager* surfaceManager = CreateSurfaceManager(testView);

  // Get back buffer, lookup should work for borrowed surfaces util present.
  auto surface = [surfaceManager surfaceForSize:CGSizeMake(100, 50)];

  // SurfaceManager should keep texture alive while borrowed.
  auto texture = surface.asFlutterMetalTexture;
  texture.destruction_callback(texture.user_data);

  FlutterMetalTexture dummyTexture{.texture_id = 1, .texture = nullptr, .user_data = nullptr};
  auto surface1 = [FlutterSurface fromFlutterMetalTexture:&dummyTexture];
  EXPECT_EQ(surface1, nil);

  auto surface2 = [FlutterSurface fromFlutterMetalTexture:&texture];
  EXPECT_EQ(surface2, surface);
}

TEST(FlutterSurfaceManager, BackBufferCacheDoesNotLeak) {
  TestView* testView = [[TestView alloc] init];
  FlutterSurfaceManager* surfaceManager = CreateSurfaceManager(testView);
  EXPECT_EQ(surfaceManager.backBufferCache.count, 0ul);

  auto surface1 = [surfaceManager surfaceForSize:CGSizeMake(100, 100)];
  [surfaceManager presentSurfaces:@[ CreatePresentInfo(surface1) ] atTime:0 notify:nil];

  EXPECT_EQ(surfaceManager.backBufferCache.count, 0ul);

  auto surface2 = [surfaceManager surfaceForSize:CGSizeMake(110, 110)];
  [surfaceManager presentSurfaces:@[ CreatePresentInfo(surface2) ] atTime:0 notify:nil];

  EXPECT_EQ(surfaceManager.backBufferCache.count, 1ul);

  auto surface3 = [surfaceManager surfaceForSize:CGSizeMake(120, 120)];
  [surfaceManager presentSurfaces:@[ CreatePresentInfo(surface3) ] atTime:0 notify:nil];

  // Cache should be cleaned during present and only contain the last visible
  // surface(s).
  EXPECT_EQ(surfaceManager.backBufferCache.count, 1ul);
  auto surfaceFromCache = [surfaceManager surfaceForSize:CGSizeMake(110, 110)];
  EXPECT_EQ(surfaceFromCache, surface2);

  // Submit empty surfaces until the one in cache gets to age >= kSurfaceEvictionAge, in which case
  // it should be removed.

  for (int i = 0; i < 30 /* kSurfaceEvictionAge */; ++i) {
    [surfaceManager presentSurfaces:@[] atTime:0 notify:nil];
    EXPECT_EQ(surfaceManager.backBufferCache.count, 1ul);
  }

  [surfaceManager presentSurfaces:@[] atTime:0 notify:nil];
  EXPECT_EQ(surfaceManager.backBufferCache.count, 0ul);
}

TEST(FlutterSurfaceManager, SurfacesAreRecycled) {
  TestView* testView = [[TestView alloc] init];
  FlutterSurfaceManager* surfaceManager = CreateSurfaceManager(testView);

  EXPECT_EQ(surfaceManager.frontSurfaces.count, 0ul);

  // Get first surface and present it.

  auto surface1 = [surfaceManager surfaceForSize:CGSizeMake(100, 100)];
  EXPECT_TRUE(CGSizeEqualToSize(surface1.size, CGSizeMake(100, 100)));

  EXPECT_EQ(surfaceManager.backBufferCache.count, 0ul);
  EXPECT_EQ(surfaceManager.frontSurfaces.count, 0ul);

  [surfaceManager presentSurfaces:@[ CreatePresentInfo(surface1) ] atTime:0 notify:nil];

  EXPECT_EQ(surfaceManager.backBufferCache.count, 0ul);
  EXPECT_EQ(surfaceManager.frontSurfaces.count, 1ul);
  EXPECT_EQ(testView.layer.sublayers.count, 1ul);

  // Get second surface and present it.

  auto surface2 = [surfaceManager surfaceForSize:CGSizeMake(100, 100)];
  EXPECT_TRUE(CGSizeEqualToSize(surface2.size, CGSizeMake(100, 100)));

  EXPECT_EQ(surfaceManager.backBufferCache.count, 0ul);

  [surfaceManager presentSurfaces:@[ CreatePresentInfo(surface2) ] atTime:0 notify:nil];

  // Check that current front surface returns to cache.
  EXPECT_EQ(surfaceManager.backBufferCache.count, 1ul);
  EXPECT_EQ(surfaceManager.frontSurfaces.count, 1ul);
  EXPECT_EQ(testView.layer.sublayers.count, 1ull);

  // Check that surface is properly reused.
  auto surface3 = [surfaceManager surfaceForSize:CGSizeMake(100, 100)];
  EXPECT_EQ(surface3, surface1);
}

TEST(FlutterSurfaceManager, BackingStoreCacheSurfaceStuckInUse) {
  TestView* testView = [[TestView alloc] init];
  FlutterSurfaceManager* surfaceManager = CreateSurfaceManager(testView);

  auto surface1 = [surfaceManager surfaceForSize:CGSizeMake(100, 100)];

  [surfaceManager presentSurfaces:@[ CreatePresentInfo(surface1) ] atTime:0 notify:nil];
  // Pretend that compositor is holding on to the surface. The surface will be kept
  // in cache until the age of kSurfaceEvictionAge is reached, and then evicted.
  surface1.isInUseOverride = YES;

  auto surface2 = [surfaceManager surfaceForSize:CGSizeMake(100, 100)];
  [surfaceManager presentSurfaces:@[ CreatePresentInfo(surface2) ] atTime:0 notify:nil];
  EXPECT_EQ(surfaceManager.backBufferCache.count, 1ul);

  for (int i = 0; i < 30 /* kSurfaceEvictionAge */ - 1; ++i) {
    auto surface3 = [surfaceManager surfaceForSize:CGSizeMake(100, 100)];
    [surfaceManager presentSurfaces:@[ CreatePresentInfo(surface3) ] atTime:0 notify:nil];
    EXPECT_EQ(surfaceManager.backBufferCache.count, 2ul);
  }

  auto surface4 = [surfaceManager surfaceForSize:CGSizeMake(100, 100)];
  [surfaceManager presentSurfaces:@[ CreatePresentInfo(surface4) ] atTime:0 notify:nil];
  // Surface in use should bet old enough at this point to be evicted.
  EXPECT_EQ(surfaceManager.backBufferCache.count, 1ul);
}

inline bool operator==(const CGRect& lhs, const CGRect& rhs) {
  return CGRectEqualToRect(lhs, rhs);
}

TEST(FlutterSurfaceManager, LayerManagement) {
  TestView* testView = [[TestView alloc] init];
  FlutterSurfaceManager* surfaceManager = CreateSurfaceManager(testView);

  EXPECT_EQ(testView.layer.sublayers.count, 0ul);

  auto surface1_1 = [surfaceManager surfaceForSize:CGSizeMake(50, 30)];
  [surfaceManager presentSurfaces:@[ CreatePresentInfo(surface1_1, CGPointMake(20, 10)) ]
                           atTime:0
                           notify:nil];

  EXPECT_EQ(testView.layer.sublayers.count, 1ul);
  EXPECT_TRUE(CGSizeEqualToSize(testView.presentedFrameSize, CGSizeMake(70, 40)));

  auto surface2_1 = [surfaceManager surfaceForSize:CGSizeMake(50, 30)];
  auto surface2_2 = [surfaceManager surfaceForSize:CGSizeMake(20, 20)];
  [surfaceManager presentSurfaces:@[
    CreatePresentInfo(surface2_1, CGPointMake(20, 10), 1),
    CreatePresentInfo(surface2_2, CGPointMake(40, 50), 2,
                      {
                          FlutterRect{0, 0, 20, 20},
                          FlutterRect{40, 0, 60, 20},
                      })
  ]
                           atTime:0
                           notify:nil];

  EXPECT_EQ(testView.layer.sublayers.count, 2ul);
  EXPECT_EQ(testView.layer.sublayers[0].zPosition, 1.0);
  EXPECT_EQ(testView.layer.sublayers[1].zPosition, 2.0);
  CALayer* firstOverlaySublayer;
  {
    NSArray<CALayer*>* sublayers = testView.layer.sublayers[1].sublayers;
    EXPECT_EQ(sublayers.count, 2ul);
    EXPECT_TRUE(CGRectEqualToRect(sublayers[0].frame, CGRectMake(0, 0, 10, 10)));
    EXPECT_TRUE(CGRectEqualToRect(sublayers[1].frame, CGRectMake(20, 0, 10, 10)));
    EXPECT_TRUE(CGRectEqualToRect(sublayers[0].contentsRect, CGRectMake(0, 0, 1, 1)));
    EXPECT_TRUE(CGRectEqualToRect(sublayers[1].contentsRect, CGRectMake(2, 0, 1, 1)));
    EXPECT_EQ(sublayers[0].contents, sublayers[1].contents);
    firstOverlaySublayer = sublayers[0];
  }
  EXPECT_TRUE(CGSizeEqualToSize(testView.presentedFrameSize, CGSizeMake(70, 70)));

  // Check second overlay sublayer is removed while first is reused and updated
  [surfaceManager presentSurfaces:@[
    CreatePresentInfo(surface2_1, CGPointMake(20, 10), 1),
    CreatePresentInfo(surface2_2, CGPointMake(40, 50), 2,
                      {
                          FlutterRect{0, 10, 20, 20},
                      })
  ]
                           atTime:0
                           notify:nil];
  EXPECT_EQ(testView.layer.sublayers.count, 2ul);
  {
    NSArray<CALayer*>* sublayers = testView.layer.sublayers[1].sublayers;
    EXPECT_EQ(sublayers.count, 1ul);
    EXPECT_EQ(sublayers[0], firstOverlaySublayer);
    EXPECT_TRUE(CGRectEqualToRect(sublayers[0].frame, CGRectMake(0, 5, 10, 5)));
  }

  // Check that second overlay sublayer is added back while first is reused and updated
  [surfaceManager presentSurfaces:@[
    CreatePresentInfo(surface2_1, CGPointMake(20, 10), 1),
    CreatePresentInfo(surface2_2, CGPointMake(40, 50), 2,
                      {
                          FlutterRect{0, 0, 20, 20},
                          FlutterRect{40, 0, 60, 20},
                      })
  ]
                           atTime:0
                           notify:nil];

  EXPECT_EQ(testView.layer.sublayers.count, 2ul);
  {
    NSArray<CALayer*>* sublayers = testView.layer.sublayers[1].sublayers;
    EXPECT_EQ(sublayers.count, 2ul);
    EXPECT_EQ(sublayers[0], firstOverlaySublayer);
    EXPECT_TRUE(CGRectEqualToRect(sublayers[0].frame, CGRectMake(0, 0, 10, 10)));
    EXPECT_TRUE(CGRectEqualToRect(sublayers[1].frame, CGRectMake(20, 0, 10, 10)));
    EXPECT_EQ(sublayers[0].contents, sublayers[1].contents);
  }

  auto surface3_1 = [surfaceManager surfaceForSize:CGSizeMake(50, 30)];
  [surfaceManager presentSurfaces:@[ CreatePresentInfo(surface3_1, CGPointMake(20, 10)) ]
                           atTime:0
                           notify:nil];

  EXPECT_EQ(testView.layer.sublayers.count, 1ul);
  EXPECT_TRUE(CGSizeEqualToSize(testView.presentedFrameSize, CGSizeMake(70, 40)));

  // Check removal of all surfaces.
  [surfaceManager presentSurfaces:@[] atTime:0 notify:nil];
  EXPECT_EQ(testView.layer.sublayers.count, 0ul);
  EXPECT_TRUE(CGSizeEqualToSize(testView.presentedFrameSize, CGSizeMake(0, 0)));
}

}  // namespace flutter::testing
