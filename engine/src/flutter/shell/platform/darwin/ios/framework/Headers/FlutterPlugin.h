// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_HEADERS_FLUTTERPLUGIN_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_HEADERS_FLUTTERPLUGIN_H_

#import <UIKit/UIKit.h>
#import <UserNotifications/UNUserNotificationCenter.h>

#import "FlutterBinaryMessenger.h"
#import "FlutterChannels.h"
#import "FlutterCodecs.h"
#import "FlutterPlatformViews.h"
#import "FlutterTexture.h"

NS_ASSUME_NONNULL_BEGIN
@protocol FlutterPluginRegistrar;
@protocol FlutterPluginRegistry;

#pragma mark -
/**
 * Protocol for listener of events from the UIApplication, typically a FlutterPlugin.
 */
@protocol FlutterApplicationLifeCycleDelegate <UNUserNotificationCenterDelegate>

@optional
/**
 * Called if this has been registered for `UIApplicationDelegate` callbacks.
 *
 * @return `NO` if this vetos application launch.
 */
- (BOOL)application:(UIApplication*)application
    didFinishLaunchingWithOptions:(NSDictionary*)launchOptions;

/**
 * Called if this has been registered for `UIApplicationDelegate` callbacks.
 *
 * @return `NO` if this vetos application launch.
 */
- (BOOL)application:(UIApplication*)application
    willFinishLaunchingWithOptions:(NSDictionary*)launchOptions;

/**
 * Called if this has been registered for `UIApplicationDelegate` callbacks.
 */
- (void)applicationDidBecomeActive:(UIApplication*)application;

/**
 * Called if this has been registered for `UIApplicationDelegate` callbacks.
 */
- (void)applicationWillResignActive:(UIApplication*)application;

/**
 * Called if this has been registered for `UIApplicationDelegate` callbacks.
 */
- (void)applicationDidEnterBackground:(UIApplication*)application;

/**
 * Called if this has been registered for `UIApplicationDelegate` callbacks.
 */
- (void)applicationWillEnterForeground:(UIApplication*)application;

/**
 Called if this has been registered for `UIApplicationDelegate` callbacks.
 */
- (void)applicationWillTerminate:(UIApplication*)application;

/**
 * Called if this has been registered for `UIApplicationDelegate` callbacks.
 */
- (void)application:(UIApplication*)application
    didRegisterUserNotificationSettings:(UIUserNotificationSettings*)notificationSettings
    API_DEPRECATED(
        "See -[UIApplicationDelegate application:didRegisterUserNotificationSettings:] deprecation",
        ios(8.0, 10.0));

/**
 * Called if this has been registered for `UIApplicationDelegate` callbacks.
 */
- (void)application:(UIApplication*)application
    didRegisterForRemoteNotificationsWithDeviceToken:(NSData*)deviceToken;

/**
 * Called if this has been registered for `UIApplicationDelegate` callbacks.
 */
- (void)application:(UIApplication*)application
    didFailToRegisterForRemoteNotificationsWithError:(NSError*)error;

/**
 * Called if this has been registered for `UIApplicationDelegate` callbacks.
 *
 * @return `YES` if this handles the request.
 */
- (BOOL)application:(UIApplication*)application
    didReceiveRemoteNotification:(NSDictionary*)userInfo
          fetchCompletionHandler:(void (^)(UIBackgroundFetchResult result))completionHandler;

/**
 * Calls all plugins registered for `UIApplicationDelegate` callbacks.
 */
- (void)application:(UIApplication*)application
    didReceiveLocalNotification:(UILocalNotification*)notification
    API_DEPRECATED(
        "See -[UIApplicationDelegate application:didReceiveLocalNotification:] deprecation",
        ios(4.0, 10.0));

/**
 * Called if this has been registered for `UIApplicationDelegate` callbacks.
 *
 * @return `YES` if this handles the request.
 */
- (BOOL)application:(UIApplication*)application
            openURL:(NSURL*)url
            options:(NSDictionary<UIApplicationOpenURLOptionsKey, id>*)options;

/**
 * Called if this has been registered for `UIApplicationDelegate` callbacks.
 *
 * @return `YES` if this handles the request.
 */
- (BOOL)application:(UIApplication*)application handleOpenURL:(NSURL*)url;

/**
 * Called if this has been registered for `UIApplicationDelegate` callbacks.
 *
 * @return `YES` if this handles the request.
 */
- (BOOL)application:(UIApplication*)application
              openURL:(NSURL*)url
    sourceApplication:(NSString*)sourceApplication
           annotation:(id)annotation;

/**
 * Called if this has been registered for `UIApplicationDelegate` callbacks.
 *
 * @return `YES` if this handles the request.
 */
- (BOOL)application:(UIApplication*)application
    performActionForShortcutItem:(UIApplicationShortcutItem*)shortcutItem
               completionHandler:(void (^)(BOOL succeeded))completionHandler
    API_AVAILABLE(ios(9.0));

/**
 * Called if this has been registered for `UIApplicationDelegate` callbacks.
 *
 * @return `YES` if this handles the request.
 */
- (BOOL)application:(UIApplication*)application
    handleEventsForBackgroundURLSession:(nonnull NSString*)identifier
                      completionHandler:(nonnull void (^)(void))completionHandler;

/**
 * Called if this has been registered for `UIApplicationDelegate` callbacks.
 *
 * @return `YES` if this handles the request.
 */
- (BOOL)application:(UIApplication*)application
    performFetchWithCompletionHandler:(void (^)(UIBackgroundFetchResult result))completionHandler;

/**
 * Called if this has been registered for `UIApplicationDelegate` callbacks.
 *
 * @return `YES` if this handles the request.
 */
- (BOOL)application:(UIApplication*)application
    continueUserActivity:(NSUserActivity*)userActivity
      restorationHandler:(void (^)(NSArray*))restorationHandler;
@end

#pragma mark -
/**
 * A plugin registration callback.
 *
 * Used for registering plugins with additional instances of
 * `FlutterPluginRegistry`.
 *
 * @param registry The registry to register plugins with.
 */
typedef void (*FlutterPluginRegistrantCallback)(NSObject<FlutterPluginRegistry>* registry);

#pragma mark -
/**
 * Implemented by the iOS part of a Flutter plugin.
 *
 * Defines a set of optional callback methods and a method to set up the plugin
 * and register it to be called by other application components.
 */
@protocol FlutterPlugin <NSObject, FlutterApplicationLifeCycleDelegate>
@required
/**
 * Registers this plugin using the context information and callback registration
 * methods exposed by the given registrar.
 *
 * The registrar is obtained from a `FlutterPluginRegistry` which keeps track of
 * the identity of registered plugins and provides basic support for cross-plugin
 * coordination.
 *
 * The caller of this method, a plugin registrant, is usually autogenerated by
 * Flutter tooling based on declared plugin dependencies. The generated registrant
 * asks the registry for a registrar for each plugin, and calls this method to
 * allow the plugin to initialize itself and register callbacks with application
 * objects available through the registrar protocol.
 *
 * @param registrar A helper providing application context and methods for
 *     registering callbacks.
 */
+ (void)registerWithRegistrar:(NSObject<FlutterPluginRegistrar>*)registrar;
@optional
/**
 * Set a callback for registering plugins to an additional `FlutterPluginRegistry`,
 * including headless `FlutterEngine` instances.
 *
 * This method is typically called from within an application's `AppDelegate` at
 * startup to allow for plugins which create additional `FlutterEngine` instances
 * to register the application's plugins.
 *
 * @param callback A callback for registering some set of plugins with a
 *     `FlutterPluginRegistry`.
 */
+ (void)setPluginRegistrantCallback:(FlutterPluginRegistrantCallback)callback;
@optional
/**
 * Called if this plugin has been registered to receive `FlutterMethodCall`s.
 *
 * @param call The method call command object.
 * @param result A callback for submitting the result of the call.
 */
- (void)handleMethodCall:(FlutterMethodCall*)call result:(FlutterResult)result;
@optional
/**
 * Called when a plugin is being removed from a `FlutterEngine`, which is
 * usually the result of the `FlutterEngine` being deallocated.  This method
 * provides the opportunity to do necessary cleanup.
 *
 * You will only receive this method if you registered your plugin instance with
 * the `FlutterEngine` via `-[FlutterPluginRegistry publish:]`.
 *
 * @param registrar The registrar that was used to publish the plugin.
 *
 */
- (void)detachFromEngineForRegistrar:(NSObject<FlutterPluginRegistrar>*)registrar;
@end

#pragma mark -
/**
 * How the UIGestureRecognizers of a platform view are blocked.
 *
 * UIGestureRecognizers of platform views can be blocked based on decisions made by the
 * Flutter Framework (e.g. When an interact-able widget is covering the platform view).
 */
typedef enum {
  // NOLINTBEGIN(readability-identifier-naming)
  /**
   * Flutter blocks all the UIGestureRecognizers on the platform view as soon as it
   * decides they should be blocked.
   *
   * With this policy, only the `touchesBegan` method for all the UIGestureRecognizers is guaranteed
   * to be called.
   */
  FlutterPlatformViewGestureRecognizersBlockingPolicyEager,
  /**
   * Flutter blocks the platform view's UIGestureRecognizers from recognizing only after
   * touchesEnded was invoked.
   *
   * This results in the platform view's UIGestureRecognizers seeing the entire touch sequence,
   * but never recognizing the gesture (and never invoking actions).
   */
  FlutterPlatformViewGestureRecognizersBlockingPolicyWaitUntilTouchesEnded,
  // NOLINTEND(readability-identifier-naming)
} FlutterPlatformViewGestureRecognizersBlockingPolicy;

#pragma mark -
/**
 * Registration context for a single `FlutterPlugin`, providing a one stop shop
 * for the plugin to access contextual information and register callbacks for
 * various application events.
 *
 * Registrars are obtained from a `FlutterPluginRegistry` which keeps track of
 * the identity of registered plugins and provides basic support for cross-plugin
 * coordination.
 */
@protocol FlutterPluginRegistrar <NSObject>
/**
 * Returns a `FlutterBinaryMessenger` for creating Dart/iOS communication
 * channels to be used by the plugin.
 *
 * @return The messenger.
 */
- (NSObject<FlutterBinaryMessenger>*)messenger;

/**
 * Returns a `FlutterTextureRegistry` for registering textures
 * provided by the plugin.
 *
 * @return The texture registry.
 */
- (NSObject<FlutterTextureRegistry>*)textures;

/**
 * Registers a `FlutterPlatformViewFactory` for creation of platform views.
 *
 * Plugins expose `UIView` for embedding in Flutter apps by registering a view factory.
 *
 * @param factory The view factory that will be registered.
 * @param factoryId A unique identifier for the factory, the Dart code of the Flutter app can use
 *   this identifier to request creation of a `UIView` by the registered factory.
 */
- (void)registerViewFactory:(NSObject<FlutterPlatformViewFactory>*)factory
                     withId:(NSString*)factoryId;

/**
 * Registers a `FlutterPlatformViewFactory` for creation of platform views.
 *
 * Plugins can expose a `UIView` for embedding in Flutter apps by registering a view factory.
 *
 * @param factory The view factory that will be registered.
 * @param factoryId A unique identifier for the factory, the Dart code of the Flutter app can use
 *   this identifier to request creation of a `UIView` by the registered factory.
 * @param gestureRecognizersBlockingPolicy How UIGestureRecognizers on the platform views are
 * blocked.
 *
 */
- (void)registerViewFactory:(NSObject<FlutterPlatformViewFactory>*)factory
                              withId:(NSString*)factoryId
    gestureRecognizersBlockingPolicy:
        (FlutterPlatformViewGestureRecognizersBlockingPolicy)gestureRecognizersBlockingPolicy;

/**
 * Publishes a value for external use of the plugin.
 *
 * Plugins may publish a single value, such as an instance of the
 * plugin's main class, for situations where external control or
 * interaction is needed.
 *
 * The published value will be available from the `FlutterPluginRegistry`.
 * Repeated calls overwrite any previous publication.
 *
 * @param value The value to be published.
 */
- (void)publish:(NSObject*)value;

/**
 * Registers the plugin as a receiver of incoming method calls from the Dart side
 * on the specified `FlutterMethodChannel`.
 *
 * @param delegate The receiving object, such as the plugin's main class.
 * @param channel The channel
 */
- (void)addMethodCallDelegate:(NSObject<FlutterPlugin>*)delegate
                      channel:(FlutterMethodChannel*)channel;

/**
 * Registers the plugin as a receiver of `UIApplicationDelegate` calls.
 *
 * @param delegate The receiving object, such as the plugin's main class.
 */
- (void)addApplicationDelegate:(NSObject<FlutterPlugin>*)delegate
    NS_EXTENSION_UNAVAILABLE_IOS("Disallowed in plugins used in app extensions");

/**
 * Returns the file name for the given asset.
 * The returned file name can be used to access the asset in the application's main bundle.
 *
 * @param asset The name of the asset. The name can be hierarchical.
 * @return the file name to be used for lookup in the main bundle.
 */
- (NSString*)lookupKeyForAsset:(NSString*)asset;

/**
 * Returns the file name for the given asset which originates from the specified package.
 * The returned file name can be used to access the asset in the application's main bundle.
 *
 *
 * @param asset The name of the asset. The name can be hierarchical.
 * @param package The name of the package from which the asset originates.
 * @return the file name to be used for lookup in the main bundle.
 */
- (NSString*)lookupKeyForAsset:(NSString*)asset fromPackage:(NSString*)package;
@end

#pragma mark -
/**
 * A registry of Flutter iOS plugins.
 *
 * Plugins are identified by unique string keys, typically the name of the
 * plugin's main class. The registry tracks plugins by this key, mapping it to
 * a value published by the plugin during registration, if any. This provides a
 * very basic means of cross-plugin coordination with loose coupling between
 * unrelated plugins.
 *
 * Plugins typically need contextual information and the ability to register
 * callbacks for various application events. To keep the API of the registry
 * focused, these facilities are not provided directly by the registry, but by
 * a `FlutterPluginRegistrar`, created by the registry in exchange for the unique
 * key of the plugin.
 *
 * There is no implied connection between the registry and the registrar.
 * Specifically, callbacks registered by the plugin via the registrar may be
 * relayed directly to the underlying iOS application objects.
 */
@protocol FlutterPluginRegistry <NSObject>
/**
 * Returns a registrar for registering a plugin.
 *
 * @param pluginKey The unique key identifying the plugin.
 */
- (nullable NSObject<FlutterPluginRegistrar>*)registrarForPlugin:(NSString*)pluginKey;
/**
 * Returns whether the specified plugin has been registered.
 *
 * @param pluginKey The unique key identifying the plugin.
 * @return `YES` if `registrarForPlugin` has been called with `pluginKey`.
 */
- (BOOL)hasPlugin:(NSString*)pluginKey;

/**
 * Returns a value published by the specified plugin.
 *
 * @param pluginKey The unique key identifying the plugin.
 * @return An object published by the plugin, if any. Will be `NSNull` if
 *   nothing has been published. Will be `nil` if the plugin has not been
 *   registered.
 */
- (nullable NSObject*)valuePublishedByPlugin:(NSString*)pluginKey;
@end

#pragma mark -
/**
 * Implement this in the `UIAppDelegate` of your app to enable Flutter plugins to register
 * themselves to the application life cycle events.
 *
 * For plugins to receive events from `UNUserNotificationCenter`, register this as the
 * `UNUserNotificationCenterDelegate`.
 */
@protocol FlutterAppLifeCycleProvider <UNUserNotificationCenterDelegate>

/**
 * Called when registering a new `FlutterApplicaitonLifeCycleDelegate`.
 *
 * See also: `-[FlutterAppDelegate addApplicationLifeCycleDelegate:]`
 */
- (void)addApplicationLifeCycleDelegate:(NSObject<FlutterApplicationLifeCycleDelegate>*)delegate;
@end

NS_ASSUME_NONNULL_END

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_HEADERS_FLUTTERPLUGIN_H_
