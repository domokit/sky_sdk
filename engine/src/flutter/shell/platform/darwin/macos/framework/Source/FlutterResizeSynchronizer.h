// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTERRESIZESYNCHRONIZER_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTERRESIZESYNCHRONIZER_H_

#import <Cocoa/Cocoa.h>

#import "flutter/shell/platform/darwin/macos/framework/Headers/FlutterViewController.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterRunLoop.h"

@interface FlutterResizeSynchronizer : NSObject

/**
 * Begins a resize operation for the given size. Block the thread until
 * performCommitForSize: with the same size is called.
 * While the thread is blocked Flutter messages are being pumped.
 * See [FlutterRunLoop pollFlutterMessagesOnce].
 */
- (void)beginResizeForSize:(CGSize)size notify:(nonnull dispatch_block_t)notify;

/**
 * Called from raster thread. Schedules the given block on platform thread
 * at given delay and unblocks the platform thread if waiting for the surface
 * during resize.
 */
- (void)performCommitForSize:(CGSize)size
                      notify:(nonnull dispatch_block_t)notify
                       delay:(NSTimeInterval)delay;

/**
 * Called when the view is shut down. Unblocks platform thread if blocked
 * during resize.
 */
- (void)shutDown;

@end

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTERRESIZESYNCHRONIZER_H_
