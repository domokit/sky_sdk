// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTERMENUPLUGIN_INTERNAL_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTERMENUPLUGIN_INTERNAL_H_

#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterMenuPlugin.h"

/**
 * Internal methods used in testing.
 */
@interface FlutterMenuPlugin ()

// Handles method calls received from the framework.
- (void)handleMethodCall:(FlutterMethodCall*)call result:(FlutterResult)result;

@end

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTERMENUPLUGIN_INTERNAL_H_
