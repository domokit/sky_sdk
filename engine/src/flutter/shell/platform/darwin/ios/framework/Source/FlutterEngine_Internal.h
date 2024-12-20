// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_FLUTTERENGINE_INTERNAL_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_FLUTTERENGINE_INTERNAL_H_

#import "flutter/shell/platform/darwin/ios/framework/Headers/FlutterEngine.h"

#include "flutter/fml/memory/weak_ptr.h"
#include "flutter/fml/task_runner.h"
#include "flutter/lib/ui/window/pointer_data_packet.h"
#include "flutter/lib/ui/window/viewport_metrics.h"
#include "flutter/shell/common/platform_view.h"
#include "flutter/shell/common/rasterizer.h"
#include "flutter/shell/common/shell.h"

#include "flutter/shell/platform/embedder/embedder.h"

#import "flutter/shell/platform/darwin/ios/framework/Headers/FlutterEngine.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterDartProject_Internal.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterIndirectScribbleDelegate.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterPlatformPlugin.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterPlatformViews_Internal.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterRestorationPlugin.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterTextInputDelegate.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterTextInputPlugin.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterView.h"

NS_ASSUME_NONNULL_BEGIN

@interface FlutterEngine () <FlutterViewEngineDelegate>

- (void)updateViewportMetrics:(flutter::ViewportMetrics)viewportMetrics;
- (void)dispatchPointerDataPacket:(std::unique_ptr<flutter::PointerDataPacket>)packet;

- (fml::RefPtr<fml::TaskRunner>)platformTaskRunner;
- (fml::RefPtr<fml::TaskRunner>)uiTaskRunner;
- (fml::RefPtr<fml::TaskRunner>)rasterTaskRunner;

- (void)installFirstFrameCallback:(void (^)(void))block;
- (void)enableSemantics:(BOOL)enabled withFlags:(int64_t)flags;
- (void)notifyViewCreated;
- (void)notifyViewDestroyed;

- (flutter::Rasterizer::Screenshot)screenshot:(flutter::Rasterizer::ScreenshotType)type
                                 base64Encode:(bool)base64Encode;

- (FlutterPlatformPlugin*)platformPlugin;
- (FlutterTextInputPlugin*)textInputPlugin;
- (FlutterRestorationPlugin*)restorationPlugin;
- (void)launchEngine:(nullable NSString*)entrypoint
          libraryURI:(nullable NSString*)libraryOrNil
      entrypointArgs:(nullable NSArray<NSString*>*)entrypointArgs;
- (BOOL)createShell:(nullable NSString*)entrypoint
         libraryURI:(nullable NSString*)libraryOrNil
       initialRoute:(nullable NSString*)initialRoute;
- (void)attachView;
- (void)notifyLowMemory;

/// Blocks until the first frame is presented or the timeout is exceeded, then invokes callback.
- (void)waitForFirstFrameSync:(NSTimeInterval)timeout
                     callback:(NS_NOESCAPE void (^)(BOOL didTimeout))callback;

/// Asynchronously waits until the first frame is presented or the timeout is exceeded, then invokes
/// callback.
- (void)waitForFirstFrame:(NSTimeInterval)timeout callback:(void (^)(BOOL didTimeout))callback;

/**
 * Creates one running FlutterEngine from another, sharing components between them.
 *
 * This results in a faster creation time and a smaller memory footprint engine.
 * This should only be called on a FlutterEngine that is running.
 */
- (FlutterEngine*)spawnWithEntrypoint:(nullable NSString*)entrypoint
                           libraryURI:(nullable NSString*)libraryURI
                         initialRoute:(nullable NSString*)initialRoute
                       entrypointArgs:(nullable NSArray<NSString*>*)entrypointArgs;

/**
 * Dispatches the given key event data to the framework through the engine.
 * The callback is called once the response from the framework is received.
 */
- (void)sendKeyEvent:(const FlutterKeyEvent&)event
            callback:(nullable FlutterKeyEventCallback)callback
            userData:(nullable void*)userData;

@property(nonatomic, readonly) FlutterDartProject* project;

@end

NS_ASSUME_NONNULL_END

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_FLUTTERENGINE_INTERNAL_H_
