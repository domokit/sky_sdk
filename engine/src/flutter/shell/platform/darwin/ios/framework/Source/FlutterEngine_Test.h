// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_FLUTTERENGINE_TEST_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_FLUTTERENGINE_TEST_H_

#import "flutter/shell/common/shell.h"
#import "flutter/shell/platform/darwin/ios/framework/Headers/FlutterEngine.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterTextInputDelegate.h"
#include "flutter/shell/platform/darwin/ios/platform_view_ios.h"
#import "flutter/shell/platform/darwin/ios/rendering_api_selection.h"
#include "flutter/shell/platform/embedder/embedder.h"

@class FlutterBinaryMessengerRelay;

namespace flutter {
class ThreadHost;
}

// Category to add test-only visibility.
@interface FlutterEngine (Test) <FlutterBinaryMessenger>

@property(readonly, nonatomic) FlutterEngineProcTable& embedderAPI;
@property(readonly, nonatomic) BOOL enableEmbedderAPI;

- (flutter::Shell&)shell;
- (flutter::PlatformViewIOS*)platformView;

- (void)setBinaryMessenger:(FlutterBinaryMessengerRelay*)binaryMessenger;
- (flutter::IOSRenderingAPI)platformViewsRenderingAPI;
- (void)waitForFirstFrame:(NSTimeInterval)timeout callback:(void (^)(BOOL didTimeout))callback;
- (FlutterEngine*)spawnWithEntrypoint:(/*nullable*/ NSString*)entrypoint
                           libraryURI:(/*nullable*/ NSString*)libraryURI
                         initialRoute:(/*nullable*/ NSString*)initialRoute
                       entrypointArgs:(/*nullable*/ NSArray<NSString*>*)entrypointArgs;
- (const flutter::ThreadHost&)threadHost;
- (void)updateDisplays;
- (void)flutterTextInputView:(FlutterTextInputView*)textInputView
               performAction:(FlutterTextInputAction)action
                  withClient:(int)client;
- (void)sceneWillEnterForeground:(NSNotification*)notification API_AVAILABLE(ios(13.0));
- (void)sceneDidEnterBackground:(NSNotification*)notification API_AVAILABLE(ios(13.0));
- (void)applicationWillEnterForeground:(NSNotification*)notification;
- (void)applicationDidEnterBackground:(NSNotification*)notification;

@end

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_FLUTTERENGINE_TEST_H_
