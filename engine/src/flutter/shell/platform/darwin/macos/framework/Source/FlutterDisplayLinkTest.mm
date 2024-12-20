// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterDisplayLink.h"

#import <AppKit/AppKit.h>
#include <numeric>

#include "flutter/fml/synchronization/waitable_event.h"
#include "flutter/testing/testing.h"

@interface TestDisplayLinkDelegate : NSObject <FlutterDisplayLinkDelegate> {
  void (^_block)(CFTimeInterval timestamp, CFTimeInterval targetTimestamp);
}

- (instancetype)initWithBlock:(void (^)(CFTimeInterval timestamp,
                                        CFTimeInterval targetTimestamp))block;

@end

@implementation TestDisplayLinkDelegate
- (instancetype)initWithBlock:(void (^__strong)(CFTimeInterval, CFTimeInterval))block {
  if (self = [super init]) {
    _block = block;
  }
  return self;
}

- (void)onDisplayLink:(CFTimeInterval)timestamp targetTimestamp:(CFTimeInterval)targetTimestamp {
  _block(timestamp, targetTimestamp);
}

@end

TEST(FlutterDisplayLinkTest, ViewAddedToWindowFirst) {
  NSWindow* window = [[NSWindow alloc] initWithContentRect:NSMakeRect(0, 0, 100, 100)
                                                 styleMask:NSWindowStyleMaskTitled
                                                   backing:NSBackingStoreNonretained
                                                     defer:NO];
  NSView* view = [[NSView alloc] initWithFrame:NSMakeRect(0, 0, 100, 100)];
  [window setContentView:view];

  auto event = std::make_shared<fml::AutoResetWaitableEvent>();

  TestDisplayLinkDelegate* delegate = [[TestDisplayLinkDelegate alloc]
      initWithBlock:^(CFTimeInterval timestamp, CFTimeInterval targetTimestamp) {
        event->Signal();
      }];

  FlutterDisplayLink* displayLink = [FlutterDisplayLink displayLinkWithView:view];
  displayLink.delegate = delegate;
  displayLink.paused = NO;

  event->Wait();

  [displayLink invalidate];
}

TEST(FlutterDisplayLinkTest, ViewAddedToWindowLater) {
  NSView* view = [[NSView alloc] initWithFrame:NSMakeRect(0, 0, 100, 100)];

  auto event = std::make_shared<fml::AutoResetWaitableEvent>();

  TestDisplayLinkDelegate* delegate = [[TestDisplayLinkDelegate alloc]
      initWithBlock:^(CFTimeInterval timestamp, CFTimeInterval targetTimestamp) {
        event->Signal();
      }];

  FlutterDisplayLink* displayLink = [FlutterDisplayLink displayLinkWithView:view];
  displayLink.delegate = delegate;
  displayLink.paused = NO;

  NSWindow* window = [[NSWindow alloc] initWithContentRect:NSMakeRect(0, 0, 100, 100)
                                                 styleMask:NSWindowStyleMaskTitled
                                                   backing:NSBackingStoreNonretained
                                                     defer:NO];
  [window setContentView:view];

  event->Wait();

  [displayLink invalidate];
}

TEST(FlutterDisplayLinkTest, ViewRemovedFromWindow) {
  NSWindow* window = [[NSWindow alloc] initWithContentRect:NSMakeRect(0, 0, 100, 100)
                                                 styleMask:NSWindowStyleMaskTitled
                                                   backing:NSBackingStoreNonretained
                                                     defer:NO];
  NSView* view = [[NSView alloc] initWithFrame:NSMakeRect(0, 0, 100, 100)];
  [window setContentView:view];

  auto event = std::make_shared<fml::AutoResetWaitableEvent>();

  TestDisplayLinkDelegate* delegate = [[TestDisplayLinkDelegate alloc]
      initWithBlock:^(CFTimeInterval timestamp, CFTimeInterval targetTimestamp) {
        event->Signal();
      }];

  FlutterDisplayLink* displayLink = [FlutterDisplayLink displayLinkWithView:view];
  displayLink.delegate = delegate;
  displayLink.paused = NO;

  event->Wait();
  displayLink.paused = YES;

  event->Reset();

  displayLink.paused = NO;

  [window setContentView:nil];

  EXPECT_TRUE(event->WaitWithTimeout(fml::TimeDelta::FromMilliseconds(100)));
  EXPECT_FALSE(event->IsSignaledForTest());

  [displayLink invalidate];
}

TEST(FlutterDisplayLinkTest, WorkaroundForFB13482573) {
  NSWindow* window = [[NSWindow alloc] initWithContentRect:NSMakeRect(0, 0, 100, 100)
                                                 styleMask:NSWindowStyleMaskTitled
                                                   backing:NSBackingStoreNonretained
                                                     defer:NO];
  NSView* view = [[NSView alloc] initWithFrame:NSMakeRect(0, 0, 100, 100)];
  [window setContentView:view];

  auto event = std::make_shared<fml::AutoResetWaitableEvent>();

  TestDisplayLinkDelegate* delegate = [[TestDisplayLinkDelegate alloc]
      initWithBlock:^(CFTimeInterval timestamp, CFTimeInterval targetTimestamp) {
        event->Signal();
      }];

  FlutterDisplayLink* displayLink = [FlutterDisplayLink displayLinkWithView:view];
  displayLink.delegate = delegate;
  displayLink.paused = NO;

  event->Wait();
  displayLink.paused = YES;

  event->Reset();
  [NSThread detachNewThreadWithBlock:^{
    // Here pthread_self() will be same as pthread_self inside first invocation of
    // display link callback, causing CVDisplayLinkStart to return error.
    displayLink.paused = NO;
  }];

  event->Wait();

  [displayLink invalidate];
}

TEST(FlutterDisplayLinkTest, CVDisplayLinkInterval) {
  CVDisplayLinkRef link;
  CVDisplayLinkCreateWithCGDisplay(CGMainDisplayID(), &link);
  __block CFTimeInterval last = 0;
  auto intervals = std::make_shared<std::vector<CFTimeInterval>>();
  auto event = std::make_shared<fml::AutoResetWaitableEvent>();
  CVDisplayLinkSetOutputHandler(
      link, ^(CVDisplayLinkRef displayLink, const CVTimeStamp* inNow,
              const CVTimeStamp* inOutputTime, CVOptionFlags flagsIn, CVOptionFlags* flagsOut) {
        if (last != 0) {
          intervals->push_back(CACurrentMediaTime() - last);
        }
        last = CACurrentMediaTime();
        if (intervals->size() == 10) {
          event->Signal();
        }
        return 0;
      });

  CVDisplayLinkStart(link);
  event->Wait();
  CVDisplayLinkStop(link);
  CVDisplayLinkRelease(link);
  CFTimeInterval average = std::reduce(intervals->begin(), intervals->end()) / intervals->size();
  CFTimeInterval max = *std::max_element(intervals->begin(), intervals->end());
  CFTimeInterval min = *std::min_element(intervals->begin(), intervals->end());
  NSLog(@"CVDisplayLink Interval: Average: %fs, Max: %fs, Min: %fs", average, max, min);
}
