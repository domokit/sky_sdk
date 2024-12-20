// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_FLUTTERAPPDELEGATE_TEST_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_FLUTTERAPPDELEGATE_TEST_H_

@class FlutterViewController;

@interface FlutterAppDelegate (Test)
@property(nonatomic, copy) FlutterViewController* (^rootFlutterViewControllerGetter)(void);

@end

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_FLUTTERAPPDELEGATE_TEST_H_
