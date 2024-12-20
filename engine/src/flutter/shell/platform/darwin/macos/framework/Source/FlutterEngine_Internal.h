// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTERENGINE_INTERNAL_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTERENGINE_INTERNAL_H_

#import "flutter/shell/platform/darwin/macos/framework/Headers/FlutterEngine.h"

#import <Cocoa/Cocoa.h>

#include <memory>

#include "flutter/shell/platform/common/app_lifecycle_state.h"

#import "flutter/shell/platform/darwin/macos/framework/Source/AccessibilityBridgeMac.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterPlatformViewController.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterRenderer.h"

NS_ASSUME_NONNULL_BEGIN

#pragma mark - Typedefs

typedef void (^FlutterTerminationCallback)(id _Nullable sender);

#pragma mark - Enumerations

/**
 * An enum for defining the different request types allowed when requesting an
 * application exit.
 *
 * Must match the entries in the `AppExitType` enum in the Dart code.
 */
typedef NS_ENUM(NSInteger, FlutterAppExitType) {
  kFlutterAppExitTypeCancelable = 0,
  kFlutterAppExitTypeRequired = 1,
};

/**
 * An enum for defining the different responses the framework can give to an
 * application exit request from the engine.
 *
 * Must match the entries in the `AppExitResponse` enum in the Dart code.
 */
typedef NS_ENUM(NSInteger, FlutterAppExitResponse) {
  kFlutterAppExitResponseCancel = 0,
  kFlutterAppExitResponseExit = 1,
};

#pragma mark - FlutterEngineTerminationHandler

/**
 * A handler interface for handling application termination that the
 * FlutterAppDelegate can use to coordinate an application exit by sending
 * messages through the platform channel managed by the engine.
 */
@interface FlutterEngineTerminationHandler : NSObject

@property(nonatomic, readonly) BOOL shouldTerminate;
@property(nonatomic, readwrite) BOOL acceptingRequests;

- (instancetype)initWithEngine:(FlutterEngine*)engine
                    terminator:(nullable FlutterTerminationCallback)terminator;
- (void)handleRequestAppExitMethodCall:(NSDictionary<NSString*, id>*)data
                                result:(FlutterResult)result;
- (void)requestApplicationTermination:(NSApplication*)sender
                             exitType:(FlutterAppExitType)type
                               result:(nullable FlutterResult)result;
@end

/**
 * An NSPasteboard wrapper object to allow for substitution of a fake in unit tests.
 */
@interface FlutterPasteboard : NSObject
- (NSInteger)clearContents;
- (NSString*)stringForType:(NSPasteboardType)dataType;
- (BOOL)setString:(NSString*)string forType:(NSPasteboardType)dataType;
@end

@interface FlutterEngine ()

/**
 * True if the engine is currently running.
 */
@property(nonatomic, readonly) BOOL running;

/**
 * Provides the renderer config needed to initialize the engine and also handles external
 * texture management.
 */
@property(nonatomic, readonly, nullable) FlutterRenderer* renderer;

/**
 * Function pointers for interacting with the embedder.h API.
 */
@property(nonatomic) FlutterEngineProcTable& embedderAPI;

/**
 * True if the semantics is enabled. The Flutter framework starts sending
 * semantics update through the embedder as soon as it is set to YES.
 */
@property(nonatomic) BOOL semanticsEnabled;

/**
 * The executable name for the current process.
 */
@property(nonatomic, readonly, nonnull) NSString* executableName;

/**
 * This just returns the NSPasteboard so that it can be mocked in the tests.
 */
@property(nonatomic, nonnull) FlutterPasteboard* pasteboard;

/**
 * The command line arguments array for the engine.
 */
@property(nonatomic, readonly) std::vector<std::string> switches;

/**
 * Provides the |FlutterEngineTerminationHandler| to be used for this engine.
 */
@property(nonatomic, readonly) FlutterEngineTerminationHandler* terminationHandler;

/**
 * Attach a view controller to the engine as its default controller.
 *
 * Since FlutterEngine can only handle the implicit view for now, the given
 * controller will always be assigned to the implicit view, if there isn't an
 * implicit view yet. If the engine already has an implicit view, this call
 * throws an assertion.
 *
 * The engine holds a weak reference to the attached view controller.
 *
 * If the given view controller is already attached to an engine, this call
 * throws an assertion.
 */
- (void)addViewController:(FlutterViewController*)viewController;

/**
 * Notify the engine that a view for the given view controller has been loaded.
 */
- (void)viewControllerViewDidLoad:(FlutterViewController*)viewController;

/**
 * Dissociate the given view controller from this engine.
 *
 * If the view controller is not associated with this engine, this call throws an
 * assertion.
 */
- (void)removeViewController:(FlutterViewController*)viewController;

/**
 * The |FlutterViewController| associated with the given view ID, if any.
 */
- (nullable FlutterViewController*)viewControllerForIdentifier:
    (FlutterViewIdentifier)viewIdentifier;

/**
 * Informs the engine that the specified view controller's window metrics have changed.
 */
- (void)updateWindowMetricsForViewController:(FlutterViewController*)viewController;

/**
 * Dispatches the given pointer event data to engine.
 */
- (void)sendPointerEvent:(const FlutterPointerEvent&)event;

/**
 * Dispatches the given pointer event data to engine.
 */
- (void)sendKeyEvent:(const FlutterKeyEvent&)event
            callback:(nullable FlutterKeyEventCallback)callback
            userData:(nullable void*)userData;

/**
 * Registers an external texture with the given id. Returns YES on success.
 */
- (BOOL)registerTextureWithID:(int64_t)textureId;

/**
 * Marks texture with the given id as available. Returns YES on success.
 */
- (BOOL)markTextureFrameAvailable:(int64_t)textureID;

/**
 * Unregisters an external texture with the given id. Returns YES on success.
 */
- (BOOL)unregisterTextureWithID:(int64_t)textureID;

- (nonnull FlutterPlatformViewController*)platformViewController;

/**
 * Handles changes to the application state, sending them to the framework.
 *
 * @param state One of the lifecycle constants in app_lifecycle_state.h,
 *              corresponding to the Dart enum AppLifecycleState.
 */
- (void)setApplicationState:(flutter::AppLifecycleState)state;

// Accessibility API.

/**
 * Dispatches semantics action back to the framework. The semantics must be enabled by calling
 * the updateSemanticsEnabled before dispatching semantics actions.
 */
- (void)dispatchSemanticsAction:(FlutterSemanticsAction)action
                       toTarget:(uint16_t)target
                       withData:(fml::MallocMapping)data;

/**
 * Handles accessibility events.
 */
- (void)handleAccessibilityEvent:(NSDictionary<NSString*, id>*)annotatedEvent;

/**
 * Announces accessibility messages.
 */
- (void)announceAccessibilityMessage:(NSString*)message
                        withPriority:(NSAccessibilityPriorityLevel)priority;

/**
 * Returns an array of screen objects representing all of the screens available on the system.
 */
- (NSArray<NSScreen*>*)screens;
@end

@interface FlutterEngine (Tests)
- (nonnull FlutterThreadSynchronizer*)testThreadSynchronizer;
@end

NS_ASSUME_NONNULL_END

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTERENGINE_INTERNAL_H_
