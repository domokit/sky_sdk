// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_HEADERS_FLUTTERVIEWCONTROLLER_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_HEADERS_FLUTTERVIEWCONTROLLER_H_

#import <UIKit/UIKit.h>
#include <sys/cdefs.h>

#import "FlutterBinaryMessenger.h"
#import "FlutterDartProject.h"
#import "FlutterEngine.h"
#import "FlutterHourFormat.h"
#import "FlutterMacros.h"
#import "FlutterPlugin.h"
#import "FlutterTexture.h"

NS_ASSUME_NONNULL_BEGIN

@class FlutterEngine;

/**
 * The name used for semantic update notifications via `NSNotificationCenter`.
 *
 * The object passed as the sender is the `FlutterViewController` associated
 * with the update.
 */
FLUTTER_DARWIN_EXPORT
// NOLINTNEXTLINE(readability-identifier-naming)
extern NSNotificationName const FlutterSemanticsUpdateNotification;

/**
 * A `UIViewController` implementation for Flutter views.
 *
 * Dart execution, channel communication, texture registration, and plugin registration are all
 * handled by `FlutterEngine`. Calls on this class to those members all proxy through to the
 * `FlutterEngine` attached FlutterViewController.
 *
 * A FlutterViewController can be initialized either with an already-running `FlutterEngine` via the
 * `initWithEngine:` initializer, or it can be initialized with a `FlutterDartProject` that will be
 * used to implicitly spin up a new `FlutterEngine`. Creating a `FlutterEngine` before showing a
 * FlutterViewController can be used to pre-initialize the Dart VM and to prepare the isolate in
 * order to reduce the latency to the first rendered frame. See
 * https://flutter.dev/docs/development/add-to-app/performance for more details on loading
 * latency.
 *
 * Holding a `FlutterEngine` independently of FlutterViewControllers can also be used to not to lose
 * Dart-related state and asynchronous tasks when navigating back and forth between a
 * FlutterViewController and other `UIViewController`s.
 */
FLUTTER_DARWIN_EXPORT
#ifdef __IPHONE_13_4
@interface FlutterViewController
    : UIViewController <FlutterTextureRegistry, FlutterPluginRegistry, UIGestureRecognizerDelegate>
#else
@interface FlutterViewController : UIViewController <FlutterTextureRegistry, FlutterPluginRegistry>
#endif

/**
 * Initializes this FlutterViewController with the specified `FlutterEngine`.
 *
 * The initialized viewcontroller will attach itself to the engine as part of this process.
 *
 * @param engine The `FlutterEngine` instance to attach to. Cannot be nil.
 * @param nibName The NIB name to initialize this UIViewController with.
 * @param nibBundle The NIB bundle.
 */
- (instancetype)initWithEngine:(FlutterEngine*)engine
                       nibName:(nullable NSString*)nibName
                        bundle:(nullable NSBundle*)nibBundle NS_DESIGNATED_INITIALIZER;

/**
 * Initializes a new FlutterViewController and `FlutterEngine` with the specified
 * `FlutterDartProject`.
 *
 * This will implicitly create a new `FlutterEngine` which is retrievable via the `engine` property
 * after initialization.
 *
 * @param project The `FlutterDartProject` to initialize the `FlutterEngine` with.
 * @param nibName The NIB name to initialize this UIViewController with.
 * @param nibBundle The NIB bundle.
 */
- (instancetype)initWithProject:(nullable FlutterDartProject*)project
                        nibName:(nullable NSString*)nibName
                         bundle:(nullable NSBundle*)nibBundle NS_DESIGNATED_INITIALIZER;

/**
 * Initializes a new FlutterViewController and `FlutterEngine` with the specified
 * `FlutterDartProject` and `initialRoute`.
 *
 * This will implicitly create a new `FlutterEngine` which is retrievable via the `engine` property
 * after initialization.
 *
 * @param project The `FlutterDartProject` to initialize the `FlutterEngine` with.
 * @param initialRoute The initial `Navigator` route to load.
 * @param nibName The NIB name to initialize this UIViewController with.
 * @param nibBundle The NIB bundle.
 */
- (instancetype)initWithProject:(nullable FlutterDartProject*)project
                   initialRoute:(nullable NSString*)initialRoute
                        nibName:(nullable NSString*)nibName
                         bundle:(nullable NSBundle*)nibBundle NS_DESIGNATED_INITIALIZER;

/**
 * Initializer that is called from loading a FlutterViewController from a XIB.
 *
 * See also:
 * https://developer.apple.com/documentation/foundation/nscoding/1416145-initwithcoder?language=objc
 */
- (instancetype)initWithCoder:(NSCoder*)aDecoder NS_DESIGNATED_INITIALIZER;

/**
 * Registers a callback that will be invoked when the Flutter view has been rendered.
 * The callback will be fired only once.
 *
 * Replaces an existing callback. Use a `nil` callback to unregister the existing one.
 */
- (void)setFlutterViewDidRenderCallback:(void (^)(void))callback;

/**
 * Returns the file name for the given asset.
 * The returned file name can be used to access the asset in the application's
 * main bundle.
 *
 * @param asset The name of the asset. The name can be hierarchical.
 * @return The file name to be used for lookup in the main bundle.
 */
- (NSString*)lookupKeyForAsset:(NSString*)asset;

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
- (NSString*)lookupKeyForAsset:(NSString*)asset fromPackage:(NSString*)package;

/**
 * Deprecated API to set initial route.
 *
 * Attempts to set the first route that the Flutter app shows if the Flutter
 * runtime hasn't yet started. The default is "/".
 *
 * This method must be called immediately after `initWithProject` and has no
 * effect when using `initWithEngine` if the `FlutterEngine` has already been
 * run.
 *
 * Setting this after the Flutter started running has no effect. See `pushRoute`
 * and `popRoute` to change the route after Flutter started running.
 *
 * This is deprecated because it needs to be called at the time of initialization
 * and thus should just be in the `initWithProject` initializer. If using
 * `initWithEngine`, the initial route should be set on the engine's
 * initializer.
 *
 * @param route The name of the first route to show.
 */
- (void)setInitialRoute:(NSString*)route
    FLUTTER_DEPRECATED("Use FlutterViewController initializer to specify initial route");

/**
 * Instructs the Flutter Navigator (if any) to go back.
 */
- (void)popRoute;

/**
 * Instructs the Flutter Navigator (if any) to push a route on to the navigation
 * stack.
 *
 * @param route The name of the route to push to the navigation stack.
 */
- (void)pushRoute:(NSString*)route;

/**
 * The `FlutterPluginRegistry` used by this FlutterViewController.
 */
- (id<FlutterPluginRegistry>)pluginRegistry;

/**
 * A wrapper around UIAccessibilityIsVoiceOverRunning().
 *
 * As a C function, UIAccessibilityIsVoiceOverRunning() cannot be mocked in testing. Mock
 * this class method to testing features depends on UIAccessibilityIsVoiceOverRunning().
 */
+ (BOOL)isUIAccessibilityIsVoiceOverRunning;

/**
 * True if at least one frame has rendered and the ViewController has appeared.
 *
 * This property is reset to false when the ViewController disappears. It is
 * guaranteed to only alternate between true and false for observers.
 */
@property(nonatomic, readonly, getter=isDisplayingFlutterUI) BOOL displayingFlutterUI;

/**
 * Specifies the view to use as a splash screen. Flutter's rendering is asynchronous, so the first
 * frame rendered by the Flutter application might not immediately appear when the Flutter view is
 * initially placed in the view hierarchy. The splash screen view will be used as
 * a replacement until the first frame is rendered.
 *
 * The view used should be appropriate for multiple sizes; an autoresizing mask to
 * have a flexible width and height will be applied automatically.
 *
 * Set to nil to remove the splash screen view.
 */
@property(strong, nonatomic, nullable) UIView* splashScreenView;

/**
 * Attempts to set the `splashScreenView` property from the `UILaunchStoryboardName` from the
 * main bundle's `Info.plist` file.  This method will not change the value of `splashScreenView`
 * if it cannot find a default one from a storyboard or nib.
 *
 * @return `YES` if successful, `NO` otherwise.
 */
- (BOOL)loadDefaultSplashScreenView;

/**
 * Controls whether the created view will be opaque or not.
 *
 * Default is `YES`.  Note that setting this to `NO` may negatively impact performance
 * when using hardware acceleration, and toggling this will trigger a re-layout of the
 * view.
 */
@property(nonatomic, getter=isViewOpaque) BOOL viewOpaque;

/**
 * The `FlutterEngine` instance for this view controller. This could be the engine this
 * `FlutterViewController` is initialized with or a new `FlutterEngine` implicitly created if
 * no engine was supplied during initialization.
 */
@property(nonatomic, readonly) FlutterEngine* engine;

/**
 * The `FlutterBinaryMessenger` associated with this FlutterViewController (used for communicating
 * with channels).
 *
 * This is just a convenient way to get the |FlutterEngine|'s binary messenger.
 */
@property(nonatomic, readonly) NSObject<FlutterBinaryMessenger>* binaryMessenger;

/**
 * If the `FlutterViewController` creates a `FlutterEngine`, this property
 * determines if that `FlutterEngine` has `allowHeadlessExecution` set.
 *
 * The intention is that this is used with the XIB.  Otherwise, a
 * `FlutterEngine` can just be sent to the init methods.
 *
 * See also: `-[FlutterEngine initWithName:project:allowHeadlessExecution:]`
 */
@property(nonatomic, readonly) BOOL engineAllowHeadlessExecution;

@end

NS_ASSUME_NONNULL_END

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_HEADERS_FLUTTERVIEWCONTROLLER_H_
