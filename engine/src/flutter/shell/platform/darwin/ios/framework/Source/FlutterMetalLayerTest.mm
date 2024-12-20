// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import <Metal/Metal.h>
#import <OCMock/OCMock.h>
#import <QuartzCore/QuartzCore.h>
#import <XCTest/XCTest.h>

#include "flutter/fml/logging.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterMetalLayer.h"

@interface FlutterMetalLayerTest : XCTestCase
@end

@interface TestFlutterMetalLayerView : UIView
@end

@implementation TestFlutterMetalLayerView

+ (Class)layerClass {
  return [FlutterMetalLayer class];
}

@end

/// A fake compositor that simulates presenting layer surface by increasing
/// and decreasing IOSurface use count.
@interface TestCompositor : NSObject {
  FlutterMetalLayer* _layer;
  IOSurfaceRef _presentedSurface;
}
@end

@implementation TestCompositor

- (instancetype)initWithLayer:(FlutterMetalLayer*)layer {
  self = [super init];
  if (self) {
    self->_layer = layer;
  }
  return self;
}

/// Increment use count of currently presented surface and decrement use count
/// of previously presented surface.
- (void)commitTransaction {
  IOSurfaceRef surface = (__bridge IOSurfaceRef)self->_layer.contents;
  if (self->_presentedSurface) {
    IOSurfaceDecrementUseCount(self->_presentedSurface);
  }
  IOSurfaceIncrementUseCount(surface);
  self->_presentedSurface = surface;
}

- (void)dealloc {
  if (self->_presentedSurface) {
    IOSurfaceDecrementUseCount(self->_presentedSurface);
  }
}

@end

@implementation FlutterMetalLayerTest

- (FlutterMetalLayer*)addMetalLayer {
  TestFlutterMetalLayerView* view =
      [[TestFlutterMetalLayerView alloc] initWithFrame:CGRectMake(0, 0, 100, 100)];
  FlutterMetalLayer* layer = (FlutterMetalLayer*)view.layer;
  layer.drawableSize = CGSizeMake(100, 100);
  return layer;
}

- (void)removeMetalLayer:(FlutterMetalLayer*)layer {
}

// For unknown reason sometimes CI fails to create IOSurface. Bail out
// to prevent flakiness.
#define BAIL_IF_NO_DRAWABLE(drawable)                \
  if (drawable == nil) {                             \
    FML_LOG(ERROR) << "Could not allocate drawable"; \
    return;                                          \
  }

- (void)testFlip {
  FlutterMetalLayer* layer = [self addMetalLayer];
  TestCompositor* compositor = [[TestCompositor alloc] initWithLayer:layer];

  id<MTLTexture> t1, t2, t3;

  id<CAMetalDrawable> drawable = [layer nextDrawable];
  BAIL_IF_NO_DRAWABLE(drawable);
  t1 = drawable.texture;
  [drawable present];
  [compositor commitTransaction];

  drawable = [layer nextDrawable];
  BAIL_IF_NO_DRAWABLE(drawable);
  t2 = drawable.texture;
  [drawable present];
  [compositor commitTransaction];

  drawable = [layer nextDrawable];
  BAIL_IF_NO_DRAWABLE(drawable);
  t3 = drawable.texture;
  [drawable present];
  [compositor commitTransaction];

  // If there was no frame drop, layer should return oldest presented
  // texture.

  drawable = [layer nextDrawable];
  XCTAssertEqual(drawable.texture, t1);

  [drawable present];
  [compositor commitTransaction];

  drawable = [layer nextDrawable];
  XCTAssertEqual(drawable.texture, t2);
  [drawable present];
  [compositor commitTransaction];

  drawable = [layer nextDrawable];
  XCTAssertEqual(drawable.texture, t3);
  [drawable present];
  [compositor commitTransaction];

  drawable = [layer nextDrawable];
  XCTAssertEqual(drawable.texture, t1);
  [drawable present];

  [self removeMetalLayer:layer];
}

- (void)testFlipWithDroppedFrame {
  FlutterMetalLayer* layer = [self addMetalLayer];
  TestCompositor* compositor = [[TestCompositor alloc] initWithLayer:layer];

  id<MTLTexture> t1, t2, t3;

  id<CAMetalDrawable> drawable = [layer nextDrawable];
  BAIL_IF_NO_DRAWABLE(drawable);
  t1 = drawable.texture;
  [drawable present];
  [compositor commitTransaction];
  XCTAssertTrue(IOSurfaceIsInUse(t1.iosurface));

  drawable = [layer nextDrawable];
  BAIL_IF_NO_DRAWABLE(drawable);
  t2 = drawable.texture;
  [drawable present];
  [compositor commitTransaction];

  drawable = [layer nextDrawable];
  BAIL_IF_NO_DRAWABLE(drawable);
  t3 = drawable.texture;
  [drawable present];
  [compositor commitTransaction];

  // Simulate compositor holding on to t3 for a while.
  IOSurfaceIncrementUseCount(t3.iosurface);

  // Here the drawable is presented, but immediately replaced by another drawable
  // (before the compositor has a chance to pick it up). This should result
  // in same drawable returned in next call to nextDrawable.
  drawable = [layer nextDrawable];
  XCTAssertEqual(drawable.texture, t1);
  XCTAssertFalse(IOSurfaceIsInUse(drawable.texture.iosurface));
  [drawable present];

  drawable = [layer nextDrawable];
  XCTAssertEqual(drawable.texture, t2);
  [drawable present];
  [compositor commitTransaction];

  // Next drawable should be t1, since it was never picked up by compositor.
  drawable = [layer nextDrawable];
  XCTAssertEqual(drawable.texture, t1);

  IOSurfaceDecrementUseCount(t3.iosurface);

  [self removeMetalLayer:layer];
}

- (void)testDroppedDrawableReturnsTextureToPool {
  FlutterMetalLayer* layer = [self addMetalLayer];
  // FlutterMetalLayer will keep creating new textures until it has 3.
  @autoreleasepool {
    for (int i = 0; i < 3; ++i) {
      id<CAMetalDrawable> drawable = [layer nextDrawable];
      BAIL_IF_NO_DRAWABLE(drawable);
    }
  }
  id<MTLTexture> texture;
  {
    @autoreleasepool {
      id<CAMetalDrawable> drawable = [layer nextDrawable];
      XCTAssertNotNil(drawable);
      texture = drawable.texture;
      // Dropping the drawable must return texture to pool, so
      // next drawable should return the same texture.
    }
  }
  {
    id<CAMetalDrawable> drawable = [layer nextDrawable];
    XCTAssertEqual(texture, drawable.texture);
  }

  [self removeMetalLayer:layer];
}

- (void)testLayerLimitsDrawableCount {
  FlutterMetalLayer* layer = [self addMetalLayer];

  id<CAMetalDrawable> d1 = [layer nextDrawable];
  BAIL_IF_NO_DRAWABLE(d1);
  id<CAMetalDrawable> d2 = [layer nextDrawable];
  BAIL_IF_NO_DRAWABLE(d2);
  id<CAMetalDrawable> d3 = [layer nextDrawable];
  BAIL_IF_NO_DRAWABLE(d3);
  XCTAssertNotNil(d3);

  // Layer should not return more than 3 drawables.
  id<CAMetalDrawable> d4 = [layer nextDrawable];
  XCTAssertNil(d4);

  [d1 present];

  // Still no drawable, until the front buffer returns to pool
  id<CAMetalDrawable> d5 = [layer nextDrawable];
  XCTAssertNil(d5);

  [d2 present];
  id<CAMetalDrawable> d6 = [layer nextDrawable];
  XCTAssertNotNil(d6);

  [self removeMetalLayer:layer];
}

- (void)testTimeout {
  FlutterMetalLayer* layer = [self addMetalLayer];
  TestCompositor* compositor = [[TestCompositor alloc] initWithLayer:layer];

  id<CAMetalDrawable> drawable = [layer nextDrawable];
  BAIL_IF_NO_DRAWABLE(drawable);

  __block MTLCommandBufferHandler handler;

  id<MTLCommandBuffer> mockCommandBuffer = OCMProtocolMock(@protocol(MTLCommandBuffer));
  OCMStub([mockCommandBuffer addCompletedHandler:OCMOCK_ANY]).andDo(^(NSInvocation* invocation) {
    MTLCommandBufferHandler handlerOnStack;
    [invocation getArgument:&handlerOnStack atIndex:2];
    // Required to copy stack block to heap.
    handler = handlerOnStack;
  });

  [(id<FlutterMetalDrawable>)drawable flutterPrepareForPresent:mockCommandBuffer];
  [drawable present];
  [compositor commitTransaction];

  // Drawable will not be available until the command buffer completes.
  drawable = [layer nextDrawable];
  XCTAssertNil(drawable);

  handler(mockCommandBuffer);

  drawable = [layer nextDrawable];
  XCTAssertNotNil(drawable);

  [self removeMetalLayer:layer];
}

- (void)testDealloc {
  __weak FlutterMetalLayer* weakLayer;
  @autoreleasepool {
    FlutterMetalLayer* layer = [self addMetalLayer];
    weakLayer = layer;
    TestCompositor* compositor = [[TestCompositor alloc] initWithLayer:layer];

    id<CAMetalDrawable> drawable = [layer nextDrawable];
    BAIL_IF_NO_DRAWABLE(drawable);
    [drawable present];
    [compositor commitTransaction];

    [self removeMetalLayer:layer];
  }
  CFTimeInterval start = CACurrentMediaTime();
  while (weakLayer != nil && CACurrentMediaTime() - start < 1) {
    // Deallocating the layer after removing is not synchronous.
    CFRunLoopRunInMode(kCFRunLoopDefaultMode, 0.01, YES);
  }

  XCTAssertNil(weakLayer);
}

@end
