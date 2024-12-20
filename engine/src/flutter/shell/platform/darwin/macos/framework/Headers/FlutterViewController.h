// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_HEADERS_FLUTTERVIEWCONTROLLER_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_HEADERS_FLUTTERVIEWCONTROLLER_H_

#import <Cocoa/Cocoa.h>

#import "FlutterEngine.h"
#import "FlutterMacros.h"
#import "FlutterPlatformViews.h"
#import "FlutterPluginRegistrarMacOS.h"

/**
 * A unique identifier for a view within which Flutter content is hosted.
 *
 * Identifiers are guaranteed to be unique for views owned by a given engine but
 * may collide for views owned by different engines.
 */
typedef int64_t FlutterViewIdentifier;

/**
 * Values for the `mouseTrackingMode` property.
 */
typedef NS_ENUM(NSInteger, FlutterMouseTrackingMode) {
  // Hover events will never be sent to Flutter.
  kFlutterMouseTrackingModeNone = 0,
  // NOLINTNEXTLINE(readability-identifier-naming)
  FlutterMouseTrackingModeNone __attribute__((deprecated)) = kFlutterMouseTrackingModeNone,

  // Hover events will be sent to Flutter when the view is in the key window.
  kFlutterMouseTrackingModeInKeyWindow = 1,
  // NOLINTNEXTLINE(readability-identifier-naming)
  FlutterMouseTrackingModeInKeyWindow
  __attribute__((deprecated)) = kFlutterMouseTrackingModeInKeyWindow,

  // Hover events will be sent to Flutter when the view is in the active app.
  kFlutterMouseTrackingModeInActiveApp = 2,
  // NOLINTNEXTLINE(readability-identifier-naming)
  FlutterMouseTrackingModeInActiveApp
  __attribute__((deprecated)) = kFlutterMouseTrackingModeInActiveApp,

  // Hover events will be sent to Flutter regardless of window and app focus.
  kFlutterMouseTrackingModeAlways = 3,
  // NOLINTNEXTLINE(readability-identifier-naming)
  FlutterMouseTrackingModeAlways __attribute__((deprecated)) = kFlutterMouseTrackingModeAlways,
};

/**
 * Controls a view that displays Flutter content and manages input.
 *
 * A FlutterViewController works with a FlutterEngine. Upon creation, the view
 * controller is always added to an engine, either a given engine, or it implicitly
 * creates an engine and add itself to that engine.
 *
 * The FlutterEngine assigns each view controller attached to it a unique ID.
 * Each view controller corresponds to a view, and the ID is used by the framework
 * to specify which view to operate.
 *
 * A FlutterViewController can also be unattached to an engine after it is manually
 * unset from the engine, or transiently during the initialization process.
 * An unattached view controller is invalid. Whether the view controller is attached
 * can be queried using FlutterViewController#attached.
 *
 * The FlutterViewController strongly references the FlutterEngine, while
 * the engine weakly the view controller. When a FlutterViewController is deallocated,
 * it automatically removes itself from its attached engine. When a FlutterEngine
 * has no FlutterViewControllers attached, it might shut down itself or not depending
 * on its configuration.
 */
FLUTTER_DARWIN_EXPORT
@interface FlutterViewController : NSViewController <FlutterPluginRegistry>

/**
 * The Flutter engine associated with this view controller.
 */
@property(nonatomic, nonnull, readonly) FlutterEngine* engine;

/**
 * The style of mouse tracking to use for the view. Defaults to
 * FlutterMouseTrackingModeInKeyWindow.
 */
@property(nonatomic) FlutterMouseTrackingMode mouseTrackingMode;

/**
 * Initializes a controller that will run the given project.
 *
 * In this initializer, this controller creates an engine, and is attached to
 * that engine as the default controller. In this way, this controller can not
 * be set to other engines. This initializer is suitable for the first Flutter
 * view controller of the app. To use the controller with an existing engine,
 * use initWithEngine:nibName:bundle: instead.
 *
 * @param project The project to run in this view controller. If nil, a default `FlutterDartProject`
 *                will be used.
 */
- (nonnull instancetype)initWithProject:(nullable FlutterDartProject*)project
    NS_DESIGNATED_INITIALIZER;

- (nonnull instancetype)initWithNibName:(nullable NSString*)nibNameOrNil
                                 bundle:(nullable NSBundle*)nibBundleOrNil
    NS_DESIGNATED_INITIALIZER;
- (nonnull instancetype)initWithCoder:(nonnull NSCoder*)nibNameOrNil NS_DESIGNATED_INITIALIZER;
/**
 * Initializes this FlutterViewController with an existing `FlutterEngine`.
 *
 * The initialized view controller will add itself to the engine as part of this process.
 *
 * This initializer is suitable for both the first Flutter view controller and
 * the following ones of the app.
 *
 * @param engine The `FlutterEngine` instance to attach to. Cannot be nil.
 * @param nibName The NIB name to initialize this controller with.
 * @param nibBundle The NIB bundle.
 */
- (nonnull instancetype)initWithEngine:(nonnull FlutterEngine*)engine
                               nibName:(nullable NSString*)nibName
                                bundle:(nullable NSBundle*)nibBundle NS_DESIGNATED_INITIALIZER;

/**
 * The identifier for this view controller, if it is attached.
 *
 * The identifier is assigned when the view controller is attached to a
 * `FlutterEngine`.
 *
 * If the view controller is detached (see `FlutterViewController#attached`),
 * reading this property throws an assertion.
 */
@property(nonatomic, readonly) FlutterViewIdentifier viewIdentifier;

/**
 * Return YES if the view controller is attached to an engine.
 */
- (BOOL)attached;

/**
 * Invoked by the engine right before the engine is restarted.
 *
 * This should reset states to as if the application has just started.  It
 * usually indicates a hot restart (Shift-R in Flutter CLI.)
 */
- (void)onPreEngineRestart;

/**
 * Returns the file name for the given asset.
 * The returned file name can be used to access the asset in the application's
 * main bundle.
 *
 * @param asset The name of the asset. The name can be hierarchical.
 * @return The file name to be used for lookup in the main bundle.
 */
- (nonnull NSString*)lookupKeyForAsset:(nonnull NSString*)asset;

/**
 * Returns the file name for the given asset which originates from the specified
 * package.
 * The returned file name can be used to access the asset in the application's
 * main bundle.
 *
 * @param asset The name of the asset. The name can be hierarchical.
 * @param package The name of the package from which the asset originates.
 * @return The file name to be used for lookup in the main bundle.
 */
- (nonnull NSString*)lookupKeyForAsset:(nonnull NSString*)asset
                           fromPackage:(nonnull NSString*)package;

/**
 * The contentView (FlutterView)'s background color is set to black during
 * its instantiation.
 *
 * The containing layer's color can be set to the NSColor provided to this method.
 *
 * For example, the background may be set after the FlutterViewController
 * is instantiated in MainFlutterWindow.swift in the Flutter project.
 * ```swift
 * import Cocoa
 * import FlutterMacOS
 *
 * class MainFlutterWindow: NSWindow {
 *   override func awakeFromNib() {
 *     let flutterViewController = FlutterViewController()
 *
 *     // The background color of the window and `FlutterViewController`
 *     // are retained separately.
 *     //
 *     // In this example, both the MainFlutterWindow and FlutterViewController's
 *     // FlutterView's backgroundColor are set to clear to achieve a fully
 *     // transparent effect.
 *     //
 *     // If the window's background color is not set, it will use the system
 *     // default.
 *     //
 *     // If the `FlutterView`'s color is not set via `FlutterViewController.setBackgroundColor`
 *     // it's default will be black.
 *     self.backgroundColor = NSColor.clear
 *     flutterViewController.backgroundColor = NSColor.clear
 *
 *     let windowFrame = self.frame
 *     self.contentViewController = flutterViewController
 *     self.setFrame(windowFrame, display: true)
 *
 *     RegisterGeneratedPlugins(registry: flutterViewController)
 *
 *     super.awakeFromNib()
 *   }
 * }
 * ```
 */
@property(readwrite, nonatomic, nullable, copy) NSColor* backgroundColor;

@end

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_HEADERS_FLUTTERVIEWCONTROLLER_H_
