// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTERTHREADSYNCHRONIZER_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTERTHREADSYNCHRONIZER_H_

#import <Cocoa/Cocoa.h>

#import "flutter/shell/platform/darwin/macos/framework/Headers/FlutterViewController.h"

/**
 * Takes care of synchronization between raster and platform thread.
 *
 * All methods of this class must be called from the platform thread,
 * except for performCommitForView:size:notify:.
 */
@interface FlutterThreadSynchronizer : NSObject

/**
 * Creates a FlutterThreadSynchronizer that uses the OS main thread as the
 * platform thread.
 */
- (nullable instancetype)init;

/**
 * Blocks until all views have a commit with their given sizes (or empty) is requested.
 */
- (void)beginResizeForView:(FlutterViewIdentifier)viewIdentifier
                      size:(CGSize)size
                    notify:(nonnull dispatch_block_t)notify;

/**
 * Called from raster thread. Schedules the given block on platform thread
 * and blocks until it is performed.
 *
 * If platform thread is blocked in `beginResize:` for given size (or size is empty),
 * unblocks platform thread.
 *
 * The notify block is guaranteed to be called within a core animation transaction.
 */
- (void)performCommitForView:(FlutterViewIdentifier)viewIdentifier
                        size:(CGSize)size
                      notify:(nonnull dispatch_block_t)notify;

/**
 * Schedules the given block to be performed on the platform thread.
 * The block will be performed even if the platform thread is blocked waiting
 * for a commit.
 */
- (void)performOnPlatformThread:(nonnull dispatch_block_t)block;

/**
 * Requests the synchronizer to track another view.
 *
 * A view must be registered before calling begineResizeForView: or
 * performCommitForView:. It is typically done when the view controller is
 * created.
 */
- (void)registerView:(FlutterViewIdentifier)viewIdentifier;

/**
 * Requests the synchronizer to no longer track a view.
 *
 * It is typically done when the view controller is destroyed.
 */
- (void)deregisterView:(FlutterViewIdentifier)viewIdentifier;

/**
 * Called when the engine shuts down.
 *
 * Prevents any further synchronization and no longer blocks any threads.
 */
- (void)shutdown;

@end

@interface FlutterThreadSynchronizer (TestUtils)

/**
 * Creates a FlutterThreadSynchronizer that uses the specified queue as the
 * platform thread.
 */
- (nullable instancetype)initWithMainQueue:(nonnull dispatch_queue_t)queue;

/**
 * Blocks current thread until the mutex is available, then return whether the
 * synchronizer is waiting for a correct commit during resizing.
 *
 * After calling an operation of the thread synchronizer, call this method,
 * and when it returns, the thread synchronizer can be at one of the following 3
 * states:
 *
 *  1. The operation has not started at all (with a return value FALSE.)
 *  2. The operation has ended (with a return value FALSE.)
 *  3. beginResizeForView: is in progress, waiting (with a return value TRUE.)
 *
 * By eliminating the 1st case (such as using the notify callback), we can use
 * this return value to decide whether the synchronizer is in case 2 or case 3,
 * that is whether the resizing is blocked by a mismatching commit.
 */
- (BOOL)isWaitingWhenMutexIsAvailable;

/**
 * Blocks current thread until there is frame available.
 * Used in FlutterEngineTest.
 */
- (void)blockUntilFrameAvailable;

@end

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTERTHREADSYNCHRONIZER_H_
