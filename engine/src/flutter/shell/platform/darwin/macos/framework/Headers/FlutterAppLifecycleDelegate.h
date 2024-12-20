// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_HEADERS_FLUTTERAPPLIFECYCLEDELEGATE_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_HEADERS_FLUTTERAPPLIFECYCLEDELEGATE_H_

#import <Cocoa/Cocoa.h>
#include <Foundation/Foundation.h>

#import "FlutterMacros.h"

NS_ASSUME_NONNULL_BEGIN

#pragma mark -
/**
 * Protocol for listener of lifecycle events from the NSApplication, typically a
 * FlutterPlugin.
 */
FLUTTER_DARWIN_EXPORT
@protocol FlutterAppLifecycleDelegate <NSObject>

@optional
/**
 * Called when the |FlutterAppDelegate| gets the applicationWillFinishLaunching
 * notification.
 */
- (void)handleWillFinishLaunching:(NSNotification*)notification;

/**
 * Called when the |FlutterAppDelegate| gets the applicationDidFinishLaunching
 * notification.
 */
- (void)handleDidFinishLaunching:(NSNotification*)notification;

/**
 * Called when the |FlutterAppDelegate| gets the applicationWillBecomeActive
 * notification.
 */
- (void)handleWillBecomeActive:(NSNotification*)notification;

/**
 * Called when the |FlutterAppDelegate| gets the applicationDidBecomeActive
 * notification.
 */
- (void)handleDidBecomeActive:(NSNotification*)notification;

/**
 * Called when the |FlutterAppDelegate| gets the applicationWillResignActive
 * notification.
 */
- (void)handleWillResignActive:(NSNotification*)notification;

/**
 * Called when the |FlutterAppDelegate| gets the applicationWillResignActive
 * notification.
 */
- (void)handleDidResignActive:(NSNotification*)notification;

/**
 * Called when the |FlutterAppDelegate| gets the applicationWillHide
 * notification.
 */
- (void)handleWillHide:(NSNotification*)notification;

/**
 * Called when the |FlutterAppDelegate| gets the applicationDidHide
 * notification.
 */
- (void)handleDidHide:(NSNotification*)notification;

/**
 * Called when the |FlutterAppDelegate| gets the applicationWillUnhide
 * notification.
 */
- (void)handleWillUnhide:(NSNotification*)notification;

/**
 * Called when the |FlutterAppDelegate| gets the applicationDidUnhide
 * notification.
 */
- (void)handleDidUnhide:(NSNotification*)notification;

/**
 * Called when the |FlutterAppDelegate| gets the applicationDidUnhide
 * notification.
 */
- (void)handleDidChangeScreenParameters:(NSNotification*)notification;

/**
 * Called when the |FlutterAppDelegate| gets the applicationDidUnhide
 * notification.
 */
- (void)handleDidChangeOcclusionState:(NSNotification*)notification;

/**
 * Called when the |FlutterAppDelegate| gets the application:openURLs:
 * callback.
 *
 * Implementers should return YES if they handle the URLs, otherwise NO.
 * Delegates will be called in order of registration, and once a delegate
 * returns YES, no further delegates will reiceve this callback.
 */
- (BOOL)handleOpenURLs:(NSArray<NSURL*>*)urls;

/**
 * Called when the |FlutterAppDelegate| gets the applicationWillTerminate
 * notification.
 *
 * Applications should not rely on always receiving all possible notifications.
 *
 * For example, if the application is killed with a task manager, a kill signal,
 * the user pulls the power from the device, or there is a rapid unscheduled
 * disassembly of the device, no notification will be sent before the
 * application is suddenly terminated, and this notification may be skipped.
 */
- (void)handleWillTerminate:(NSNotification*)notification;
@end

#pragma mark -

/**
 * Propagates `NSAppDelegate` callbacks to registered delegates.
 */
FLUTTER_DARWIN_EXPORT
@interface FlutterAppLifecycleRegistrar : NSObject <FlutterAppLifecycleDelegate>

/**
 * Registers `delegate` to receive lifecycle callbacks via this
 * FlutterAppLifecycleDelegate as long as it is alive.
 *
 * `delegate` will only be referenced weakly.
 */
- (void)addDelegate:(NSObject<FlutterAppLifecycleDelegate>*)delegate;

/**
 * Unregisters `delegate` so that it will no longer receive life cycle callbacks
 * via this FlutterAppLifecycleDelegate.
 *
 * `delegate` will only be referenced weakly.
 */
- (void)removeDelegate:(NSObject<FlutterAppLifecycleDelegate>*)delegate;
@end

NS_ASSUME_NONNULL_END

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_HEADERS_FLUTTERAPPLIFECYCLEDELEGATE_H_
