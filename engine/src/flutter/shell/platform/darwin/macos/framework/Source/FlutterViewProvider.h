// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTERVIEWPROVIDER_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTERVIEWPROVIDER_H_

#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterView.h"

/**
 * An interface to query FlutterView.
 *
 * See also:
 *
 *  * FlutterViewEngineProvider, a typical implementation.
 */
@protocol FlutterViewProvider

/**
 * Get the FlutterView with the given view ID.
 *
 * Returns nil if the ID is invalid.
 */
- (nullable FlutterView*)viewForIdentifier:(FlutterViewIdentifier)id;

@end

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTERVIEWPROVIDER_H_
