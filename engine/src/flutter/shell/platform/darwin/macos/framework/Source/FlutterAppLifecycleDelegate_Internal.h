// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTERAPPLIFECYCLEDELEGATE_INTERNAL_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTERAPPLIFECYCLEDELEGATE_INTERNAL_H_

#import "flutter/shell/platform/darwin/macos/framework/Headers/FlutterAppLifecycleDelegate.h"

@interface FlutterAppLifecycleRegistrar ()
/**
 * Registered delegates. Exposed to allow FlutterAppDelegate to share the delegate list for
 * handling non-notification delegation.
 */
@property(nonatomic, strong) NSPointerArray* delegates;
@end

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTERAPPLIFECYCLEDELEGATE_INTERNAL_H_
