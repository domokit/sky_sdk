// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_FLUTTERUIPRESSPROXY_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_FLUTTERUIPRESSPROXY_H_

#import <UIKit/UIKit.h>
#include <functional>

/**
 * A event class that is a wrapper around a UIPress and a UIEvent to allow
 * overidding for testing purposes, since UIKit doesn't allow creation of
 * UIEvent or UIPress directly.
 */
API_AVAILABLE(ios(13.4))
@interface FlutterUIPressProxy : NSObject

- (instancetype)initWithPress:(UIPress*)press withEvent:(UIEvent*)event API_AVAILABLE(ios(13.4));

- (UIPressPhase)phase API_AVAILABLE(ios(13.4));
- (UIKey*)key API_AVAILABLE(ios(13.4));
- (UIEventType)type API_AVAILABLE(ios(13.4));
- (NSTimeInterval)timestamp API_AVAILABLE(ios(13.4));

@end

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_FLUTTERUIPRESSPROXY_H_
