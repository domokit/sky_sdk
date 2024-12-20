// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_HEADERS_FLUTTERENGINEGROUP_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_HEADERS_FLUTTERENGINEGROUP_H_

#import <Foundation/Foundation.h>

#import "FlutterEngine.h"

NS_ASSUME_NONNULL_BEGIN

/** Options that control how a FlutterEngine should be created. */
FLUTTER_DARWIN_EXPORT
@interface FlutterEngineGroupOptions : NSObject

/**
 * The name of a top-level function from a Dart library.  If this is FlutterDefaultDartEntrypoint
 * (or nil); this will default to `main()`.  If it is not the app's main() function, that function
 * must be decorated with `@pragma(vm:entry-point)` to ensure themethod is not tree-shaken by the
 * Dart compiler.
 */
@property(nonatomic, copy, nullable) NSString* entrypoint;

/**
 * The URI of the Dart library which contains the entrypoint method.  If nil, this will default to
 * the same library as the `main()` function in the Dart program.
 */
@property(nonatomic, copy, nullable) NSString* libraryURI;

/**
 * The name of the initial Flutter `Navigator` `Route` to load. If this is
 * FlutterDefaultInitialRoute (or nil), it will default to the "/" route.
 */
@property(nonatomic, copy, nullable) NSString* initialRoute;

/**
 * Arguments passed as a list of string to Dart's entrypoint function.
 */
@property(nonatomic, copy, nullable) NSArray<NSString*>* entrypointArgs;
@end

/**
 * Represents a collection of FlutterEngines who share resources which allows
 * them to be created with less time const and occupy less memory than just
 * creating multiple FlutterEngines.
 *
 * Deleting a FlutterEngineGroup doesn't invalidate existing FlutterEngines, but
 * it eliminates the possibility to create more FlutterEngines in that group.
 *
 * @warning This class is a work-in-progress and may change.
 * @see https://github.com/flutter/flutter/issues/72009
 */
FLUTTER_DARWIN_EXPORT
@interface FlutterEngineGroup : NSObject
- (instancetype)init NS_UNAVAILABLE;

/**
 * Initialize a new FlutterEngineGroup.
 *
 * @param name The name that will present in the threads shared across the
 * engines in this group.
 * @param project The `FlutterDartProject` that all FlutterEngines in this group
 * will be executing.
 */
- (instancetype)initWithName:(NSString*)name
                     project:(nullable FlutterDartProject*)project NS_DESIGNATED_INITIALIZER;

/**
 * Creates a running `FlutterEngine` that shares components with this group.
 *
 * @param entrypoint The name of a top-level function from a Dart library.  If this is
 *   FlutterDefaultDartEntrypoint (or nil); this will default to `main()`.  If it is not the app's
 *   main() function, that function must be decorated with `@pragma(vm:entry-point)` to ensure the
 *   method is not tree-shaken by the Dart compiler.
 * @param libraryURI The URI of the Dart library which contains the entrypoint method.  IF nil,
 *   this will default to the same library as the `main()` function in the Dart program.
 *
 * @see FlutterEngineGroup
 */
- (FlutterEngine*)makeEngineWithEntrypoint:(nullable NSString*)entrypoint
                                libraryURI:(nullable NSString*)libraryURI;

/**
 * Creates a running `FlutterEngine` that shares components with this group.
 *
 * @param entrypoint The name of a top-level function from a Dart library.  If this is
 *   FlutterDefaultDartEntrypoint (or nil); this will default to `main()`.  If it is not the app's
 *   main() function, that function must be decorated with `@pragma(vm:entry-point)` to ensure the
 *   method is not tree-shaken by the Dart compiler.
 * @param libraryURI The URI of the Dart library which contains the entrypoint method.  IF nil,
 *   this will default to the same library as the `main()` function in the Dart program.
 * @param initialRoute The name of the initial Flutter `Navigator` `Route` to load. If this is
 *   FlutterDefaultInitialRoute (or nil), it will default to the "/" route.
 *
 * @see FlutterEngineGroup
 */
- (FlutterEngine*)makeEngineWithEntrypoint:(nullable NSString*)entrypoint
                                libraryURI:(nullable NSString*)libraryURI
                              initialRoute:(nullable NSString*)initialRoute;

/**
 * Creates a running `FlutterEngine` that shares components with this group.
 *
 * @param options Options that control how a FlutterEngine should be created.
 *
 * @see FlutterEngineGroupOptions
 */
- (FlutterEngine*)makeEngineWithOptions:(nullable FlutterEngineGroupOptions*)options;
@end

NS_ASSUME_NONNULL_END

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_HEADERS_FLUTTERENGINEGROUP_H_
