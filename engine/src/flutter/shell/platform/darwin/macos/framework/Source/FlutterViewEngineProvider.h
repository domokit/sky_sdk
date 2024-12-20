// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTERVIEWENGINEPROVIDER_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTERVIEWENGINEPROVIDER_H_

#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterViewProvider.h"

@class FlutterEngine;

/**
 * A facade over FlutterEngine that allows FlutterEngine's children components
 * to query FlutterView.
 *
 * FlutterViewProvider only holds a weak reference to FlutterEngine.
 */
@interface FlutterViewEngineProvider : NSObject <FlutterViewProvider>

/**
 * Create a FlutterViewProvider with the underlying engine.
 */
- (nonnull instancetype)initWithEngine:(nonnull __weak FlutterEngine*)engine;

@end

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTERVIEWENGINEPROVIDER_H_
