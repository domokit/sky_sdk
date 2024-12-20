// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTERVIEWCONTROLLER_INTERNAL_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTERVIEWCONTROLLER_INTERNAL_H_

#import "flutter/shell/platform/darwin/macos/framework/Headers/FlutterViewController.h"

#include <memory>

#import "flutter/shell/platform/darwin/macos/framework/Source/AccessibilityBridgeMac.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterKeyboardViewDelegate.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterTextInputPlugin.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterView.h"

@interface FlutterViewController () <FlutterKeyboardViewDelegate>

// The FlutterView for this view controller.
@property(nonatomic, readonly, nullable) FlutterView* flutterView;

/**
 * The text input plugin that handles text editing state for text fields.
 */
@property(nonatomic, readonly, nonnull) FlutterTextInputPlugin* textInputPlugin;

@property(nonatomic, readonly) std::weak_ptr<flutter::AccessibilityBridgeMac> accessibilityBridge;

/**
 * Returns YES if provided event is being currently redispatched by keyboard manager.
 */
- (BOOL)isDispatchingKeyEvent:(nonnull NSEvent*)event;

/**
 * Set up the controller with `engine` and `id`, and other engine-level classes.
 *
 * This method is called by FlutterEngine. A view controller must be set up
 * before being used, and must be set up only once until detachFromEngine:.
 */
- (void)setUpWithEngine:(nonnull FlutterEngine*)engine
         viewIdentifier:(FlutterViewIdentifier)viewIdentifier
     threadSynchronizer:(nonnull FlutterThreadSynchronizer*)threadSynchronizer;

/**
 * Reset the `engine` and `id` of this controller.
 *
 * This method is called by FlutterEngine.
 */
- (void)detachFromEngine;

/**
 * Called by the associated FlutterEngine when FlutterEngine#semanticsEnabled
 * has changed.
 */
- (void)notifySemanticsEnabledChanged;

/**
 * Notify from the framework that the semantics for this view needs to be
 * updated.
 */
- (void)updateSemantics:(nonnull const FlutterSemanticsUpdate2*)update;

@end

// Private methods made visible for testing
@interface FlutterViewController (TestMethods)
- (void)onAccessibilityStatusChanged:(BOOL)enabled;

/* Creates an accessibility bridge with the provided parameters.
 *
 * By default this method calls AccessibilityBridgeMac's initializer. Exposing
 * this method allows unit tests to override.
 */
- (std::shared_ptr<flutter::AccessibilityBridgeMac>)createAccessibilityBridgeWithEngine:
    (nonnull FlutterEngine*)engine;

- (nonnull FlutterView*)createFlutterViewWithMTLDevice:(nonnull id<MTLDevice>)device
                                          commandQueue:(nonnull id<MTLCommandQueue>)commandQueue;

@end

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTERVIEWCONTROLLER_INTERNAL_H_
