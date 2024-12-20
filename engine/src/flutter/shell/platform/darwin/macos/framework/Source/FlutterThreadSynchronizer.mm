// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterThreadSynchronizer.h"

#import <QuartzCore/QuartzCore.h>

#include <mutex>
#include <unordered_map>
#include <vector>

#import "flutter/fml/logging.h"
#import "flutter/fml/synchronization/waitable_event.h"

@interface FlutterThreadSynchronizer () {
  dispatch_queue_t _mainQueue;
  std::mutex _mutex;
  BOOL _shuttingDown;
  std::unordered_map<int64_t, CGSize> _contentSizes;
  std::vector<dispatch_block_t> _scheduledBlocks;

  BOOL _beginResizeWaiting;

  // Used to block [beginResize:].
  std::condition_variable _condBlockBeginResize;
}

/**
 * Returns true if all existing views have a non-zero size.
 *
 * If there are no views, still returns true.
 */
- (BOOL)allViewsHaveFrame;

/**
 * Returns true if there are any views that have a non-zero size.
 *
 * If there are no views, returns false.
 */
- (BOOL)someViewsHaveFrame;

@end

@implementation FlutterThreadSynchronizer

- (instancetype)init {
  return [self initWithMainQueue:dispatch_get_main_queue()];
}

- (instancetype)initWithMainQueue:(dispatch_queue_t)queue {
  self = [super init];
  if (self != nil) {
    _mainQueue = queue;
  }
  return self;
}

- (BOOL)allViewsHaveFrame {
  for (auto const& [viewIdentifier, contentSize] : _contentSizes) {
    if (CGSizeEqualToSize(contentSize, CGSizeZero)) {
      return NO;
    }
  }
  return YES;
}

- (BOOL)someViewsHaveFrame {
  for (auto const& [viewIdentifier, contentSize] : _contentSizes) {
    if (!CGSizeEqualToSize(contentSize, CGSizeZero)) {
      return YES;
    }
  }
  return NO;
}

- (void)drain {
  dispatch_assert_queue(_mainQueue);

  [CATransaction begin];
  [CATransaction setDisableActions:YES];
  for (dispatch_block_t block : _scheduledBlocks) {
    block();
  }
  [CATransaction commit];
  _scheduledBlocks.clear();
}

- (void)blockUntilFrameAvailable {
  std::unique_lock<std::mutex> lock(_mutex);
  [self drain];

  _beginResizeWaiting = YES;
  while (![self someViewsHaveFrame] && !_shuttingDown) {
    _condBlockBeginResize.wait(lock);
    [self drain];
  }

  _beginResizeWaiting = NO;
}

- (void)beginResizeForView:(FlutterViewIdentifier)viewIdentifier
                      size:(CGSize)size
                    notify:(nonnull dispatch_block_t)notify {
  dispatch_assert_queue(_mainQueue);
  std::unique_lock<std::mutex> lock(_mutex);

  if (![self allViewsHaveFrame] || _shuttingDown) {
    // No blocking until framework produces at least one frame
    notify();
    return;
  }

  [self drain];

  notify();

  _contentSizes[viewIdentifier] = CGSizeMake(-1, -1);

  _beginResizeWaiting = YES;

  while (true) {
    if (_shuttingDown) {
      break;
    }
    const CGSize& contentSize = _contentSizes[viewIdentifier];
    if (CGSizeEqualToSize(contentSize, size) || CGSizeEqualToSize(contentSize, CGSizeZero)) {
      break;
    }
    _condBlockBeginResize.wait(lock);
    [self drain];
  }

  _beginResizeWaiting = NO;
}

- (void)performCommitForView:(FlutterViewIdentifier)viewIdentifier
                        size:(CGSize)size
                      notify:(nonnull dispatch_block_t)notify {
  dispatch_assert_queue_not(_mainQueue);
  fml::AutoResetWaitableEvent event;
  {
    std::unique_lock<std::mutex> lock(_mutex);
    if (_shuttingDown) {
      // Engine is shutting down, main thread may be blocked by the engine
      // waiting for raster thread to finish.
      return;
    }
    fml::AutoResetWaitableEvent& e = event;
    _scheduledBlocks.push_back(^{
      notify();
      _contentSizes[viewIdentifier] = size;
      e.Signal();
    });
    if (_beginResizeWaiting) {
      _condBlockBeginResize.notify_all();
    } else {
      dispatch_async(_mainQueue, ^{
        std::unique_lock<std::mutex> lock(_mutex);
        [self drain];
      });
    }
  }
  event.Wait();
}

- (void)performOnPlatformThread:(nonnull dispatch_block_t)block {
  std::unique_lock<std::mutex> lock(_mutex);
  _scheduledBlocks.push_back(block);
  if (_beginResizeWaiting) {
    _condBlockBeginResize.notify_all();
  } else {
    dispatch_async(_mainQueue, ^{
      std::unique_lock<std::mutex> lock(_mutex);
      [self drain];
    });
  }
}

- (void)registerView:(FlutterViewIdentifier)viewIdentifier {
  dispatch_assert_queue(_mainQueue);
  std::unique_lock<std::mutex> lock(_mutex);
  _contentSizes[viewIdentifier] = CGSizeZero;
}

- (void)deregisterView:(FlutterViewIdentifier)viewIdentifier {
  dispatch_assert_queue(_mainQueue);
  std::unique_lock<std::mutex> lock(_mutex);
  _contentSizes.erase(viewIdentifier);
}

- (void)shutdown {
  dispatch_assert_queue(_mainQueue);
  std::unique_lock<std::mutex> lock(_mutex);
  _shuttingDown = YES;
  _condBlockBeginResize.notify_all();
  [self drain];
}

- (BOOL)isWaitingWhenMutexIsAvailable {
  std::unique_lock<std::mutex> lock(_mutex);
  return _beginResizeWaiting;
}

@end
