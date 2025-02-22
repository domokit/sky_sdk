// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterResizeSynchronizer.h"
#import "flutter/fml/logging.h"

@implementation FlutterResizeSynchronizer {
  std::atomic_bool _inResize;
  bool _shuttingDown;
  bool _didReceiveFrame;
  CGSize _contentSize;
}

- (void)beginResizeForSize:(CGSize)size notify:(nonnull dispatch_block_t)notify {
  if (!_didReceiveFrame || _shuttingDown) {
    notify();
    return;
  }

  _inResize = true;
  _contentSize = CGSizeMake(-1, -1);
  notify();
  CFAbsoluteTime start = CFAbsoluteTimeGetCurrent();
  while (true) {
    if (CGSizeEqualToSize(_contentSize, size) || _shuttingDown) {
      break;
    }
    if (CFAbsoluteTimeGetCurrent() - start > 1.0) {
      FML_LOG(ERROR) << "Resize timed out.";
      break;
    }
    [FlutterRunLoop.mainRunLoop pollFlutterMessagesOnce];
  }
  _inResize = false;
}

- (void)performCommitForSize:(CGSize)size
                      notify:(nonnull dispatch_block_t)notify
                       delay:(NSTimeInterval)delay {
  if (_inResize) {
    delay = 0;
  }
  [FlutterRunLoop.mainRunLoop
      performBlock:^{
        _didReceiveFrame = YES;
        _contentSize = size;
        notify();
      }
        afterDelay:delay];
}

- (void)shutDown {
  [FlutterRunLoop.mainRunLoop performBlock:^{
    _shuttingDown = YES;
  }];
}

@end
