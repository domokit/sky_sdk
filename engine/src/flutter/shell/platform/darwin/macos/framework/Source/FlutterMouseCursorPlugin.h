// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTERMOUSECURSORPLUGIN_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTERMOUSECURSORPLUGIN_H_

#import <Cocoa/Cocoa.h>

#import "flutter/shell/platform/darwin/common/framework/Headers/FlutterBinaryMessenger.h"
#import "flutter/shell/platform/darwin/macos/framework/Headers/FlutterViewController.h"

@protocol FlutterMouseCursorPluginDelegate <NSObject>
- (void)didUpdateMouseCursor:(nonnull NSCursor*)cursor;
@end

/**
 * A plugin to handle mouse cursor.
 *
 * Responsible for bridging the native macOS mouse cursor system with the
 * Flutter framework mouse cursor classes, via system channels.
 */
@interface FlutterMouseCursorPlugin : NSObject <FlutterPlugin>

+ (void)registerWithRegistrar:(nonnull id<FlutterPluginRegistrar>)registrar
                     delegate:(nullable id<FlutterMouseCursorPluginDelegate>)delegate;

@end

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTERMOUSECURSORPLUGIN_H_
