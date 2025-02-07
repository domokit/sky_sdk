// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/fml/synchronization/waitable_event.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterResizeSynchronizer.h"
#import "flutter/testing/testing.h"

TEST(FlutterThreadSynchronizerTest, NotBlocked) {
  [FlutterRunLoop ensureMainLoopInitialized];

  FlutterResizeSynchronizer* synchronizer = [[FlutterResizeSynchronizer alloc] init];
  __block BOOL performed = NO;

  [NSThread detachNewThreadWithBlock:^{
    [synchronizer performCommitForSize:CGSizeMake(100, 100)
                                notify:^{
                                  performed = YES;
                                }
                                 delay:0];
  }];

  CFTimeInterval start = CFAbsoluteTimeGetCurrent();

  while (!performed && CFAbsoluteTimeGetCurrent() - start < 1.0) {
    [FlutterRunLoop.mainRunLoop pollFlutterMessagesOnce];
  }
  EXPECT_EQ(performed, YES);
}

TEST(FlutterThreadSynchronizerTest, WaitForResize) {
  [FlutterRunLoop ensureMainLoopInitialized];

  FlutterResizeSynchronizer* synchronizer = [[FlutterResizeSynchronizer alloc] init];

  __block BOOL commit1 = NO;
  __block BOOL commit2 = NO;

  fml::AutoResetWaitableEvent latch_;
  fml::AutoResetWaitableEvent& latch = latch_;

  // Resize synchronizer must have at received one frame in order to block.
  __block BOOL didReceiveFrame = NO;
  [synchronizer performCommitForSize:CGSizeMake(10, 10)
                              notify:^{
                                didReceiveFrame = YES;
                              }
                               delay:0];

  CFTimeInterval start = CFAbsoluteTimeGetCurrent();
  while (!didReceiveFrame && CFAbsoluteTimeGetCurrent() - start < 1.0) {
    [FlutterRunLoop.mainRunLoop pollFlutterMessagesOnce];
  }

  // Now resize should block until it has received expected size.

  [NSThread detachNewThreadWithBlock:^{
    latch.Wait();

    [synchronizer performCommitForSize:CGSizeMake(50, 100)
                                notify:^{
                                  commit1 = YES;
                                }
                                 delay:0];

    [synchronizer performCommitForSize:CGSizeMake(100, 100)
                                notify:^{
                                  commit2 = YES;
                                }
                                 delay:0];
  }];

  [synchronizer beginResizeForSize:CGSizeMake(100, 100)
                            notify:^{
                              latch.Signal();
                            }];

  EXPECT_EQ(commit1, YES);
  EXPECT_EQ(commit2, YES);
}

TEST(FlutterThreadSynchronizerTest, UnblocksOnShutDown) {
  [FlutterRunLoop ensureMainLoopInitialized];

  FlutterResizeSynchronizer* synchronizer = [[FlutterResizeSynchronizer alloc] init];

  // Resize synchronizer must have at received one frame in order to block.
  __block BOOL didReceiveFrame = NO;
  [synchronizer performCommitForSize:CGSizeMake(10, 10)
                              notify:^{
                                didReceiveFrame = YES;
                              }
                               delay:0];

  CFTimeInterval start = CFAbsoluteTimeGetCurrent();
  while (!didReceiveFrame && CFAbsoluteTimeGetCurrent() - start < 1.0) {
    [FlutterRunLoop.mainRunLoop pollFlutterMessagesOnce];
  }

  fml::AutoResetWaitableEvent latch_;
  fml::AutoResetWaitableEvent& latch = latch_;

  [NSThread detachNewThreadWithBlock:^{
    latch.Wait();

    [synchronizer shutDown];
  }];

  [synchronizer beginResizeForSize:CGSizeMake(100, 100)
                            notify:^{
                              // Unblock resize
                              latch.Signal();
                            }];

  // Subsequent calls should not block.
  [synchronizer beginResizeForSize:CGSizeMake(100, 100)
                            notify:^{
                            }];
}
