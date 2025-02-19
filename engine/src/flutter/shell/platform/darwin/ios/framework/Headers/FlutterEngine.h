// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_HEADERS_FLUTTERENGINE_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_HEADERS_FLUTTERENGINE_H_

#import <Foundation/Foundation.h>
#import <UIKit/UIKit.h>

#import "FlutterBinaryMessenger.h"
#import "FlutterDartProject.h"
#import "FlutterMacros.h"
#import "FlutterPlugin.h"
#import "FlutterTexture.h"

@class FlutterViewController;

NS_ASSUME_NONNULL_BEGIN

/**
 * The dart entrypoint that is associated with `main()`.  This is to be used as an argument to the
 * `runWithEntrypoint*` methods.
 */
// NOLINTNEXTLINE(readability-identifier-naming)
extern NSString* const FlutterDefaultDartEntrypoint;

/**
 * The default Flutter initial route ("/").
 */
// NOLINTNEXTLINE(readability-identifier-naming)
extern NSString* const FlutterDefaultInitialRoute;

/**
 * The FlutterEngine class coordinates a single instance of execution for a
 * `FlutterDartProject`.  It may have zero or one `FlutterViewController` at a
 * time, which can be specified via `-setViewController:`.
 * `FlutterViewController`'s `initWithEngine` initializer will automatically call
 * `-setViewController:` for itself.
 *
 * A FlutterEngine can be created independently of a `FlutterViewController` for
 * headless execution.  It can also persist across the lifespan of multiple
 * `FlutterViewController` instances to maintain state and/or asynchronous tasks
 * (such as downloading a large file).
 *
 * A FlutterEngine can also be used to prewarm the Dart execution environment and reduce the
 * latency of showing the Flutter screen when a `FlutterViewController` is created and presented.
 * See http://flutter.dev/docs/development/add-to-app/performance for more details on loading
 * performance.
 *
 * Alternatively, you can simply create a new `FlutterViewController` with only a
 * `FlutterDartProject`. That `FlutterViewController` will internally manage its
 * own instance of a FlutterEngine, but will not guarantee survival of the engine
 * beyond the life of the ViewController.
 *
 * A newly initialized FlutterEngine will not actually run a Dart Isolate until
 * either `-runWithEntrypoint:` or `-runWithEntrypoint:libraryURI` is invoked.
 * One of these methods must be invoked before calling `-setViewController:`.
 */
FLUTTER_DARWIN_EXPORT
@interface FlutterEngine : NSObject <FlutterPluginRegistry>

/**
 * Default initializer for a FlutterEngine.
 *
 * Threads created by this FlutterEngine will appear as "FlutterEngine #" in
 * Instruments. The prefix can be customized using `initWithName`.
 *
 * The engine will execute the project located in the bundle with the identifier
 * "io.flutter.flutter.app" (the default for Flutter projects).
 *
 * A newly initialized engine will not run until either `-runWithEntrypoint:` or
 * `-runWithEntrypoint:libraryURI:` is called.
 *
 * FlutterEngine created with this method will have allowHeadlessExecution set to `YES`.
 * This means that the engine will continue to run regardless of whether a `FlutterViewController`
 * is attached to it or not, until `-destroyContext:` is called or the process finishes.
 */
- (instancetype)init;

/**
 * Initialize this FlutterEngine.
 *
 * The engine will execute the project located in the bundle with the identifier
 * "io.flutter.flutter.app" (the default for Flutter projects).
 *
 * A newly initialized engine will not run until either `-runWithEntrypoint:` or
 * `-runWithEntrypoint:libraryURI:` is called.
 *
 * FlutterEngine created with this method will have allowHeadlessExecution set to `YES`.
 * This means that the engine will continue to run regardless of whether a `FlutterViewController`
 * is attached to it or not, until `-destroyContext:` is called or the process finishes.
 *
 * @param labelPrefix The label prefix used to identify threads for this instance. Should
 *   be unique across FlutterEngine instances, and is used in instrumentation to label
 *   the threads used by this FlutterEngine.
 */
- (instancetype)initWithName:(NSString*)labelPrefix;

/**
 * Initialize this FlutterEngine with a `FlutterDartProject`.
 *
 * If the FlutterDartProject is not specified, the FlutterEngine will attempt to locate
 * the project in a default location (the flutter_assets folder in the iOS application
 * bundle).
 *
 * A newly initialized engine will not run the `FlutterDartProject` until either
 * `-runWithEntrypoint:` or `-runWithEntrypoint:libraryURI:` is called.
 *
 * FlutterEngine created with this method will have allowHeadlessExecution set to `YES`.
 * This means that the engine will continue to run regardless of whether a `FlutterViewController`
 * is attached to it or not, until `-destroyContext:` is called or the process finishes.
 *
 * @param labelPrefix The label prefix used to identify threads for this instance. Should
 *   be unique across FlutterEngine instances, and is used in instrumentation to label
 *   the threads used by this FlutterEngine.
 * @param project The `FlutterDartProject` to run.
 */
- (instancetype)initWithName:(NSString*)labelPrefix project:(nullable FlutterDartProject*)project;

/**
 * Initialize this FlutterEngine with a `FlutterDartProject`.
 *
 * If the FlutterDartProject is not specified, the FlutterEngine will attempt to locate
 * the project in a default location (the flutter_assets folder in the iOS application
 * bundle).
 *
 * A newly initialized engine will not run the `FlutterDartProject` until either
 * `-runWithEntrypoint:` or `-runWithEntrypoint:libraryURI:` is called.
 *
 * @param labelPrefix The label prefix used to identify threads for this instance. Should
 *   be unique across FlutterEngine instances, and is used in instrumentation to label
 *   the threads used by this FlutterEngine.
 * @param project The `FlutterDartProject` to run.
 * @param allowHeadlessExecution Whether or not to allow this instance to continue
 *   running after passing a nil `FlutterViewController` to `-setViewController:`.
 */
- (instancetype)initWithName:(NSString*)labelPrefix
                     project:(nullable FlutterDartProject*)project
      allowHeadlessExecution:(BOOL)allowHeadlessExecution;

/**
 * Initialize this FlutterEngine with a `FlutterDartProject`.
 *
 * If the FlutterDartProject is not specified, the FlutterEngine will attempt to locate
 * the project in a default location (the flutter_assets folder in the iOS application
 * bundle).
 *
 * A newly initialized engine will not run the `FlutterDartProject` until either
 * `-runWithEntrypoint:` or `-runWithEntrypoint:libraryURI:` is called.
 *
 * @param labelPrefix The label prefix used to identify threads for this instance. Should
 *   be unique across FlutterEngine instances, and is used in instrumentation to label
 *   the threads used by this FlutterEngine.
 * @param project The `FlutterDartProject` to run.
 * @param allowHeadlessExecution Whether or not to allow this instance to continue
 *   running after passing a nil `FlutterViewController` to `-setViewController:`.
 * @param restorationEnabled Whether state restoration is enabled. When true, the framework will
 *   wait for the attached view controller to provide restoration data.
 */
- (instancetype)initWithName:(NSString*)labelPrefix
                     project:(nullable FlutterDartProject*)project
      allowHeadlessExecution:(BOOL)allowHeadlessExecution
          restorationEnabled:(BOOL)restorationEnabled NS_DESIGNATED_INITIALIZER;

/**
 * Runs a Dart program on an Isolate from the main Dart library (i.e. the library that
 * contains `main()`), using `main()` as the entrypoint (the default for Flutter projects),
 * and using "/" (the default route) as the initial route.
 *
 * The first call to this method will create a new Isolate. Subsequent calls will return
 * immediately and have no effect.
 *
 * @return YES if the call succeeds in creating and running a Flutter Engine instance; NO otherwise.
 */
- (BOOL)run;

/**
 * Runs a Dart program on an Isolate from the main Dart library (i.e. the library that
 * contains `main()`), using "/" (the default route) as the initial route.
 *
 * The first call to this method will create a new Isolate. Subsequent calls will return
 * immediately and have no effect.
 *
 * @param entrypoint The name of a top-level function from the same Dart
 *   library that contains the app's main() function.  If this is FlutterDefaultDartEntrypoint (or
 *   nil) it will default to `main()`.  If it is not the app's main() function, that function must
 *   be decorated with `@pragma(vm:entry-point)` to ensure the method is not tree-shaken by the Dart
 *   compiler.
 * @return YES if the call succeeds in creating and running a Flutter Engine instance; NO otherwise.
 */
- (BOOL)runWithEntrypoint:(nullable NSString*)entrypoint;

/**
 * Runs a Dart program on an Isolate from the main Dart library (i.e. the library that
 * contains `main()`).
 *
 * The first call to this method will create a new Isolate. Subsequent calls will return
 * immediately and have no effect.
 *
 * @param entrypoint The name of a top-level function from the same Dart
 *   library that contains the app's main() function.  If this is FlutterDefaultDartEntrypoint (or
 *   nil), it will default to `main()`.  If it is not the app's main() function, that function must
 *   be decorated with `@pragma(vm:entry-point)` to ensure the method is not tree-shaken by the Dart
 *   compiler.
 * @param initialRoute The name of the initial Flutter `Navigator` `Route` to load. If this is
 *   FlutterDefaultInitialRoute (or nil), it will default to the "/" route.
 * @return YES if the call succeeds in creating and running a Flutter Engine instance; NO otherwise.
 */
- (BOOL)runWithEntrypoint:(nullable NSString*)entrypoint
             initialRoute:(nullable NSString*)initialRoute;

/**
 * Runs a Dart program on an Isolate using the specified entrypoint and Dart library,
 * which may not be the same as the library containing the Dart program's `main()` function.
 *
 * The first call to this method will create a new Isolate. Subsequent calls will return
 * immediately and have no effect.
 *
 * @param entrypoint The name of a top-level function from a Dart library.  If this is
 *   FlutterDefaultDartEntrypoint (or nil); this will default to `main()`.  If it is not the app's
 *   main() function, that function must be decorated with `@pragma(vm:entry-point)` to ensure the
 *   method is not tree-shaken by the Dart compiler.
 * @param uri The URI of the Dart library which contains the entrypoint method
 *   (example "package:foo_package/main.dart").  If nil, this will default to
 *   the same library as the `main()` function in the Dart program.
 * @return YES if the call succeeds in creating and running a Flutter Engine instance; NO otherwise.
 */
- (BOOL)runWithEntrypoint:(nullable NSString*)entrypoint libraryURI:(nullable NSString*)uri;

/**
 * Runs a Dart program on an Isolate using the specified entrypoint and Dart library,
 * which may not be the same as the library containing the Dart program's `main()` function.
 *
 * The first call to this method will create a new Isolate. Subsequent calls will return
 * immediately and have no effect.
 *
 * @param entrypoint The name of a top-level function from a Dart library.  If this is
 *   FlutterDefaultDartEntrypoint (or nil); this will default to `main()`.  If it is not the app's
 *   main() function, that function must be decorated with `@pragma(vm:entry-point)` to ensure the
 *   method is not tree-shaken by the Dart compiler.
 * @param libraryURI The URI of the Dart library which contains the entrypoint
 *   method (example "package:foo_package/main.dart").  If nil, this will
 *   default to the same library as the `main()` function in the Dart program.
 * @param initialRoute The name of the initial Flutter `Navigator` `Route` to load. If this is
 *   FlutterDefaultInitialRoute (or nil), it will default to the "/" route.
 * @return YES if the call succeeds in creating and running a Flutter Engine instance; NO otherwise.
 */
- (BOOL)runWithEntrypoint:(nullable NSString*)entrypoint
               libraryURI:(nullable NSString*)libraryURI
             initialRoute:(nullable NSString*)initialRoute;

/**
 * Runs a Dart program on an Isolate using the specified entrypoint and Dart library,
 * which may not be the same as the library containing the Dart program's `main()` function.
 *
 * The first call to this method will create a new Isolate. Subsequent calls will return
 * immediately and have no effect.
 *
 * @param entrypoint The name of a top-level function from a Dart library.  If this is
 *   FlutterDefaultDartEntrypoint (or nil); this will default to `main()`.  If it is not the app's
 *   main() function, that function must be decorated with `@pragma(vm:entry-point)` to ensure the
 *   method is not tree-shaken by the Dart compiler.
 * @param libraryURI The URI of the Dart library which contains the entrypoint
 *   method (example "package:foo_package/main.dart").  If nil, this will
 *   default to the same library as the `main()` function in the Dart program.
 * @param initialRoute The name of the initial Flutter `Navigator` `Route` to load. If this is
 *   FlutterDefaultInitialRoute (or nil), it will default to the "/" route.
 * @param entrypointArgs Arguments passed as a list of string to Dart's entrypoint function.
 * @return YES if the call succeeds in creating and running a Flutter Engine instance; NO otherwise.
 */
- (BOOL)runWithEntrypoint:(nullable NSString*)entrypoint
               libraryURI:(nullable NSString*)libraryURI
             initialRoute:(nullable NSString*)initialRoute
           entrypointArgs:(nullable NSArray<NSString*>*)entrypointArgs;

/**
 * Destroy running context for an engine.
 *
 * This method can be used to force the FlutterEngine object to release all resources.
 * After sending this message, the object will be in an unusable state until it is deallocated.
 * Accessing properties or sending messages to it will result in undefined behavior or runtime
 * errors.
 */
- (void)destroyContext;

/**
 * Ensures that Flutter will generate a semantics tree.
 *
 * This is enabled by default if certain accessibility services are turned on by
 * the user, or when using a Simulator. This method allows a user to turn
 * semantics on when they would not ordinarily be generated and the performance
 * overhead is not a concern, e.g. for UI testing. Note that semantics should
 * never be programmatically turned off, as it would potentially disable
 * accessibility services an end user has requested.
 *
 * This method must only be called after launching the engine via
 * `-runWithEntrypoint:` or `-runWithEntryPoint:libraryURI`.
 *
 * Although this method returns synchronously, it does not guarantee that a
 * semantics tree is actually available when the method returns. It
 * synchronously ensures that the next frame the Flutter framework creates will
 * have a semantics tree.
 *
 * You can subscribe to semantics updates via `NSNotificationCenter` by adding
 * an observer for the name `FlutterSemanticsUpdateNotification`.  The `object`
 * parameter will be the `FlutterViewController` associated with the semantics
 * update.  This will asynchronously fire after a semantics tree has actually
 * built (which may be some time after the frame has been rendered).
 */
- (void)ensureSemanticsEnabled;

/**
 * Sets the `FlutterViewController` for this instance.  The FlutterEngine must be
 * running (e.g. a successful call to `-runWithEntrypoint:` or `-runWithEntrypoint:libraryURI`)
 * before calling this method. Callers may pass nil to remove the viewController
 * and have the engine run headless in the current process.
 *
 * A FlutterEngine can only have one `FlutterViewController` at a time. If there is
 * already a `FlutterViewController` associated with this instance, this method will replace
 * the engine's current viewController with the newly specified one.
 *
 * Setting the viewController will signal the engine to start animations and drawing, and unsetting
 * it will signal the engine to stop animations and drawing.  However, neither will impact the state
 * of the Dart program's execution.
 */
@property(nonatomic, weak) FlutterViewController* viewController;

/**
 * The `FlutterMethodChannel` used for localization related platform messages, such as
 * setting the locale.
 *
 * Can be nil after `destroyContext` is called.
 */
@property(nonatomic, readonly, nullable) FlutterMethodChannel* localizationChannel;
/**
 * The `FlutterMethodChannel` used for navigation related platform messages.
 *
 * Can be nil after `destroyContext` is called.
 *
 * @see [Navigation
 * Channel](https://api.flutter.dev/flutter/services/SystemChannels/navigation-constant.html)
 * @see [Navigator Widget](https://api.flutter.dev/flutter/widgets/Navigator-class.html)
 */
@property(nonatomic, readonly) FlutterMethodChannel* navigationChannel;

/**
 * The `FlutterMethodChannel` used for restoration related platform messages.
 *
 * Can be nil after `destroyContext` is called.
 *
 * @see [Restoration
 * Channel](https://api.flutter.dev/flutter/services/SystemChannels/restoration-constant.html)
 */
@property(nonatomic, readonly) FlutterMethodChannel* restorationChannel;

/**
 * The `FlutterMethodChannel` used for core platform messages, such as
 * information about the screen orientation.
 *
 * Can be nil after `destroyContext` is called.
 */
@property(nonatomic, readonly) FlutterMethodChannel* platformChannel;

/**
 * The `FlutterMethodChannel` used to communicate text input events to the
 * Dart Isolate.
 *
 * Can be nil after `destroyContext` is called.
 *
 * @see [Text Input
 * Channel](https://api.flutter.dev/flutter/services/SystemChannels/textInput-constant.html)
 */
@property(nonatomic, readonly) FlutterMethodChannel* textInputChannel;

/**
 * The `FlutterBasicMessageChannel` used to communicate app lifecycle events
 * to the Dart Isolate.
 *
 * Can be nil after `destroyContext` is called.
 *
 * @see [Lifecycle
 * Channel](https://api.flutter.dev/flutter/services/SystemChannels/lifecycle-constant.html)
 */
@property(nonatomic, readonly) FlutterBasicMessageChannel* lifecycleChannel;

/**
 * The `FlutterBasicMessageChannel` used for communicating system events, such as
 * memory pressure events.
 *
 * Can be nil after `destroyContext` is called.
 *
 * @see [System
 * Channel](https://api.flutter.dev/flutter/services/SystemChannels/system-constant.html)
 */
@property(nonatomic, readonly) FlutterBasicMessageChannel* systemChannel;

/**
 * The `FlutterBasicMessageChannel` used for communicating user settings such as
 * clock format and text scale.
 *
 * Can be nil after `destroyContext` is called.
 */
@property(nonatomic, readonly) FlutterBasicMessageChannel* settingsChannel;

/**
 * The `FlutterBasicMessageChannel` used for communicating key events
 * from physical keyboards
 *
 * Can be nil after `destroyContext` is called.
 */
@property(nonatomic, readonly) FlutterBasicMessageChannel* keyEventChannel;

/**
 * The depcreated `NSURL` of the Dart VM Service for the service isolate.
 *
 * This is only set in debug and profile runtime modes, and only after the
 * Dart VM Service is ready. In release mode or before the Dart VM Service has
 * started, it returns `nil`.
 */
@property(nonatomic, readonly, nullable)
    NSURL* observatoryUrl FLUTTER_DEPRECATED("Use vmServiceUrl instead");

/**
 * The `NSURL` of the Dart VM Service for the service isolate.
 *
 * This is only set in debug and profile runtime modes, and only after the
 * Dart VM Service is ready. In release mode or before the Dart VM Service has
 * started, it returns `nil`.
 */
@property(nonatomic, readonly, nullable) NSURL* vmServiceUrl;

/**
 * The `FlutterBinaryMessenger` associated with this FlutterEngine (used for communicating with
 * channels).
 */
@property(nonatomic, readonly) NSObject<FlutterBinaryMessenger>* binaryMessenger;

/**
 * The `FlutterTextureRegistry` associated with this FlutterEngine (used to register textures).
 */
@property(nonatomic, readonly) NSObject<FlutterTextureRegistry>* textureRegistry;

/**
 * The UI Isolate ID of the engine.
 *
 * This property will be nil if the engine is not running.
 */
@property(nonatomic, readonly, copy, nullable) NSString* isolateId;

/**
 * Whether or not GPU calls are allowed.
 *
 * Typically this is set when the app is backgrounded and foregrounded.
 */
@property(nonatomic, assign) BOOL isGpuDisabled;

@end

NS_ASSUME_NONNULL_END

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_HEADERS_FLUTTERENGINE_H_
