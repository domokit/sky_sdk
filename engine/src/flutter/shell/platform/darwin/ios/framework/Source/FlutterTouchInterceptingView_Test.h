// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterPlatformViews_Internal.h"

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_FLUTTERTOUCHINTERCEPTINGVIEW_TEST_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_FLUTTERTOUCHINTERCEPTINGVIEW_TEST_H_

@interface FlutterTouchInterceptingView (Tests)
- (id)accessibilityContainer;
@end

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_FLUTTERTOUCHINTERCEPTINGVIEW_TEST_H_
