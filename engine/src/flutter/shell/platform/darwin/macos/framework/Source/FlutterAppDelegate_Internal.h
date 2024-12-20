// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTERAPPDELEGATE_INTERNAL_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTERAPPDELEGATE_INTERNAL_H_

#import "flutter/shell/platform/darwin/macos/framework/Headers/FlutterAppDelegate.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterEngine_Internal.h"

@interface FlutterAppDelegate ()

/**
 * Holds a weak reference to the termination handler owned by the engine.
 * Called by the |FlutterApplication| when termination is requested by the OS.
 */
@property(readwrite, nullable, weak) FlutterEngineTerminationHandler* terminationHandler;

@end

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTERAPPDELEGATE_INTERNAL_H_
