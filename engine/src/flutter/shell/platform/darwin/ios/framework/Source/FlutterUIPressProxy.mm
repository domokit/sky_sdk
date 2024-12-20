// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterUIPressProxy.h"

#import <UIKit/UIKit.h>

#import "flutter/shell/platform/darwin/common/framework/Headers/FlutterMacros.h"

FLUTTER_ASSERT_ARC

@interface FlutterUIPressProxy ()
@property(nonatomic, readonly) UIPress* press;
@property(nonatomic, readonly) UIEvent* event;
@end

@implementation FlutterUIPressProxy

- (instancetype)initWithPress:(UIPress*)press withEvent:(UIEvent*)event API_AVAILABLE(ios(13.4)) {
  self = [super init];
  if (self) {
    _press = press;
    _event = event;
  }
  return self;
}

- (UIPressPhase)phase API_AVAILABLE(ios(13.4)) {
  return _press.phase;
}

- (UIKey*)key API_AVAILABLE(ios(13.4)) {
  return _press.key;
}

- (UIEventType)type API_AVAILABLE(ios(13.4)) {
  return _event.type;
}

- (NSTimeInterval)timestamp API_AVAILABLE(ios(13.4)) {
  return _event.timestamp;
}

@end
