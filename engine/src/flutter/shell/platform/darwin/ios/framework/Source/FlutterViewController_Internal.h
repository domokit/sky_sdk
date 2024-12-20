// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_FLUTTERVIEWCONTROLLER_INTERNAL_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_FLUTTERVIEWCONTROLLER_INTERNAL_H_

#import "flutter/shell/platform/darwin/ios/framework/Headers/FlutterViewController.h"

#include "flutter/fml/time/time_point.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterKeySecondaryResponder.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterKeyboardManager.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterPlatformViewsController.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterRestorationPlugin.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterUIPressProxy.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterViewResponder.h"

namespace flutter {
class PlatformViewsController;
}

FLUTTER_DARWIN_EXPORT
// NOLINTNEXTLINE(readability-identifier-naming)
extern NSNotificationName const FlutterViewControllerWillDealloc;

FLUTTER_DARWIN_EXPORT
// NOLINTNEXTLINE(readability-identifier-naming)
extern NSNotificationName const FlutterViewControllerHideHomeIndicator;

FLUTTER_DARWIN_EXPORT
// NOLINTNEXTLINE(readability-identifier-naming)
extern NSNotificationName const FlutterViewControllerShowHomeIndicator;

typedef NS_ENUM(NSInteger, FlutterKeyboardMode) {
  // NOLINTBEGIN(readability-identifier-naming)
  FlutterKeyboardModeHidden = 0,
  FlutterKeyboardModeDocked = 1,
  FlutterKeyboardModeFloating = 2,
  // NOLINTEND(readability-identifier-naming)
};

typedef void (^FlutterKeyboardAnimationCallback)(fml::TimePoint);

@interface FlutterViewController () <FlutterViewResponder>

@property(class, nonatomic, readonly) BOOL accessibilityIsOnOffSwitchLabelsEnabled;
@property(nonatomic, readonly) BOOL isPresentingViewController;
@property(nonatomic, readonly) BOOL isVoiceOverRunning;
@property(nonatomic, strong) FlutterKeyboardManager* keyboardManager;

/**
 * @brief Whether the status bar is preferred hidden.
 *
 *        This overrides the |UIViewController:prefersStatusBarHidden|.
 *        This is ignored when `UIViewControllerBasedStatusBarAppearance` in info.plist
 *        of the app project is `false`.
 */
@property(nonatomic, assign, readwrite) BOOL prefersStatusBarHidden;

@property(nonatomic, readonly) FlutterPlatformViewsController* platformViewsController;

- (FlutterRestorationPlugin*)restorationPlugin;

// Accepts keypress events, and then calls |nextAction| if the event was not
// handled.
- (void)handlePressEvent:(FlutterUIPressProxy*)press
              nextAction:(void (^)())nextAction API_AVAILABLE(ios(13.4));
- (void)sendDeepLinkToFramework:(NSURL*)url completionHandler:(void (^)(BOOL success))completion;
- (void)addInternalPlugins;
- (void)deregisterNotifications;
- (int32_t)accessibilityFlags;

- (BOOL)supportsShowingSystemContextMenu;
@end

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_FLUTTERVIEWCONTROLLER_INTERNAL_H_
