// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTERMENUPLUGIN_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTERMENUPLUGIN_H_

#import <AppKit/AppKit.h>

#import "flutter/shell/platform/darwin/common/framework/Headers/FlutterBinaryMessenger.h"
#import "flutter/shell/platform/darwin/macos/framework/Headers/FlutterPluginMacOS.h"
#import "flutter/shell/platform/darwin/macos/framework/Headers/FlutterViewController.h"

/**
 * A plugin to configure and control the native system menu.
 *
 * Responsible for bridging the native macOS menu system with the Flutter
 * framework's PlatformMenuBar class, via method channels.
 */
@interface FlutterMenuPlugin : NSObject <FlutterPlugin>

/**
 * Registers a FlutterMenuPlugin with the given registrar.
 */
+ (void)registerWithRegistrar:(nonnull id<FlutterPluginRegistrar>)registrar;
@end

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTERMENUPLUGIN_H_
