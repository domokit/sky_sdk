// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTERPLATFORMVIEWCONTROLLER_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTERPLATFORMVIEWCONTROLLER_H_

#import <Cocoa/Cocoa.h>

#import "FlutterChannels.h"

#import "flutter/shell/platform/darwin/macos/framework/Headers/FlutterPlatformViews.h"

#include <map>
#include <unordered_set>

@interface FlutterPlatformViewController : NSViewController
@end

@interface FlutterPlatformViewController ()

/**
 * Creates a platform view of viewType with viewId and arguments passed from
 * the framework's creationParams constructor parameter.
 * FlutterResult is updated to contain nil for success or to contain
 * a FlutterError if there is an error.
 */
- (void)onCreateWithViewIdentifier:(int64_t)viewId
                          viewType:(nonnull NSString*)viewType
                         arguments:(nullable id)args
                            result:(nonnull FlutterResult)result;

/**
 * Disposes the platform view with `viewId`.
 * FlutterResult is updated to contain nil for success or a FlutterError if there is an error.
 */
- (void)onDisposeWithViewID:(int64_t)viewId result:(nonnull FlutterResult)result;

/**
 * Returns the platform view associated with the viewId.
 */
- (nullable NSView*)platformViewWithID:(int64_t)viewId;

/**
 * Register a view factory by adding an entry into the platformViewFactories map with key factoryId
 * and value factory.
 */
- (void)registerViewFactory:(nonnull NSObject<FlutterPlatformViewFactory>*)factory
                     withId:(nonnull NSString*)factoryId;

/**
 * Handles platform view related method calls, for example create, dispose, etc.
 */
- (void)handleMethodCall:(nonnull FlutterMethodCall*)call result:(nonnull FlutterResult)result;

/**
 * Removes platform views slated to be disposed via method handler calls.
 */
- (void)disposePlatformViews;

/**
 * Removes all platform views.
 */
- (void)reset;

@end

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTERPLATFORMVIEWCONTROLLER_H_
