// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterView.h"

#import <Metal/Metal.h>

#import "flutter/testing/testing.h"

constexpr int64_t kImplicitViewId = 0ll;

@interface TestFlutterViewDelegate : NSObject <FlutterViewDelegate>

@end

@implementation TestFlutterViewDelegate

- (void)viewDidReshape:(nonnull NSView*)view {
}

- (BOOL)viewShouldAcceptFirstResponder:(NSView*)view {
  return YES;
}

@end

TEST(FlutterView, ShouldInheritContentsScaleReturnsYes) {
  id<MTLDevice> device = MTLCreateSystemDefaultDevice();
  id<MTLCommandQueue> queue = [device newCommandQueue];
  TestFlutterViewDelegate* delegate = [[TestFlutterViewDelegate alloc] init];
  FlutterView* view = [[FlutterView alloc] initWithMTLDevice:device
                                                commandQueue:queue
                                                    delegate:delegate
                                              viewIdentifier:kImplicitViewId];
  EXPECT_EQ([view layer:view.layer shouldInheritContentsScale:3.0 fromWindow:view.window], YES);
}

@interface TestFlutterView : FlutterView

@property(readwrite, nonatomic) NSView* (^onHitTest)(NSPoint point);

@end

@implementation TestFlutterView

@synthesize onHitTest;

- (NSView*)hitTest:(NSPoint)point {
  return self.onHitTest(point);
}

- (void)reshaped {
  // Disable resize synchronization for testing.
}

@end

@interface TestCursor : NSCursor
@property(readwrite, nonatomic) BOOL setCalled;
@end

@implementation TestCursor

- (void)set {
  self.setCalled = YES;
}

@end

TEST(FlutterView, CursorUpdateDoesHitTest) {
  id<MTLDevice> device = MTLCreateSystemDefaultDevice();
  id<MTLCommandQueue> queue = [device newCommandQueue];
  TestFlutterViewDelegate* delegate = [[TestFlutterViewDelegate alloc] init];
  TestFlutterView* view = [[TestFlutterView alloc] initWithMTLDevice:device
                                                        commandQueue:queue
                                                            delegate:delegate
                                                      viewIdentifier:kImplicitViewId];
  NSWindow* window = [[NSWindow alloc] initWithContentRect:NSMakeRect(0, 0, 800, 600)
                                                 styleMask:NSBorderlessWindowMask
                                                   backing:NSBackingStoreBuffered
                                                     defer:NO];

  TestCursor* cursor = [[TestCursor alloc] init];

  window.contentView = view;
  __weak NSView* weakView = view;
  __block BOOL hitTestCalled = NO;
  __block NSPoint hitTestCoordinate = NSZeroPoint;
  view.onHitTest = ^NSView*(NSPoint point) {
    hitTestCalled = YES;
    hitTestCoordinate = point;
    return weakView;
  };
  NSEvent* mouseEvent = [NSEvent mouseEventWithType:NSEventTypeMouseMoved
                                           location:NSMakePoint(100, 100)
                                      modifierFlags:0
                                          timestamp:0
                                       windowNumber:0
                                            context:nil
                                        eventNumber:0
                                         clickCount:0
                                           pressure:0];
  [view didUpdateMouseCursor:cursor];
  [view cursorUpdate:mouseEvent];

  EXPECT_TRUE(hitTestCalled);
  // The hit test coordinate should be in the window coordinate system.
  EXPECT_TRUE(CGPointEqualToPoint(hitTestCoordinate, CGPointMake(100, 100)));
  EXPECT_TRUE(cursor.setCalled);
}

TEST(FlutterView, CursorUpdateDoesNotOverridePlatformView) {
  id<MTLDevice> device = MTLCreateSystemDefaultDevice();
  id<MTLCommandQueue> queue = [device newCommandQueue];
  TestFlutterViewDelegate* delegate = [[TestFlutterViewDelegate alloc] init];
  TestFlutterView* view = [[TestFlutterView alloc] initWithMTLDevice:device
                                                        commandQueue:queue
                                                            delegate:delegate
                                                      viewIdentifier:kImplicitViewId];
  NSWindow* window = [[NSWindow alloc] initWithContentRect:NSMakeRect(0, 0, 800, 600)
                                                 styleMask:NSBorderlessWindowMask
                                                   backing:NSBackingStoreBuffered
                                                     defer:NO];

  TestCursor* cursor = [[TestCursor alloc] init];

  NSView* platformView = [[NSView alloc] initWithFrame:NSMakeRect(0, 0, 100, 100)];

  window.contentView = view;
  __block BOOL hitTestCalled = NO;
  __block NSPoint hitTestCoordinate = NSZeroPoint;
  view.onHitTest = ^NSView*(NSPoint point) {
    hitTestCalled = YES;
    hitTestCoordinate = point;
    return platformView;
  };
  NSEvent* mouseEvent = [NSEvent mouseEventWithType:NSEventTypeMouseMoved
                                           location:NSMakePoint(100, 100)
                                      modifierFlags:0
                                          timestamp:0
                                       windowNumber:0
                                            context:nil
                                        eventNumber:0
                                         clickCount:0
                                           pressure:0];
  [view didUpdateMouseCursor:cursor];
  [view cursorUpdate:mouseEvent];

  EXPECT_TRUE(hitTestCalled);
  // The hit test coordinate should be in the window coordinate system.
  EXPECT_TRUE(CGPointEqualToPoint(hitTestCoordinate, CGPointMake(100, 100)));
  EXPECT_FALSE(cursor.setCalled);
}
