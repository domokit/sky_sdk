// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_HEADERS_FLUTTERHEADLESSDARTRUNNER_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_HEADERS_FLUTTERHEADLESSDARTRUNNER_H_

#import <Foundation/Foundation.h>

#import "FlutterBinaryMessenger.h"
#import "FlutterDartProject.h"
#import "FlutterEngine.h"
#import "FlutterMacros.h"

/**
 * A callback for when FlutterHeadlessDartRunner has attempted to start a Dart
 * Isolate in the background.
 *
 * @param success YES if the Isolate was started and run successfully, NO
 *   otherwise.
 */
typedef void (^FlutterHeadlessDartRunnerCallback)(BOOL success);

/**
 * The deprecated FlutterHeadlessDartRunner runs Flutter Dart code with a null rasterizer,
 * and no native drawing surface. It is appropriate for use in running Dart
 * code e.g. in the background from a plugin.
 *
 * Most callers should prefer using `FlutterEngine` directly; this interface exists
 * for legacy support.
 */
FLUTTER_DARWIN_EXPORT
FLUTTER_DEPRECATED("FlutterEngine should be used rather than FlutterHeadlessDartRunner")
@interface FlutterHeadlessDartRunner : FlutterEngine

/**
 * Initialize this FlutterHeadlessDartRunner with a `FlutterDartProject`.
 *
 * If the FlutterDartProject is not specified, the FlutterHeadlessDartRunner will attempt to locate
 * the project in a default location.
 *
 * A newly initialized engine will not run the `FlutterDartProject` until either
 * `-runWithEntrypoint:` or `-runWithEntrypoint:libraryURI` is called.
 *
 * @param labelPrefix The label prefix used to identify threads for this instance. Should
 * be unique across FlutterEngine instances
 * @param projectOrNil The `FlutterDartProject` to run.
 */
- (instancetype)initWithName:(NSString*)labelPrefix project:(FlutterDartProject*)projectOrNil;

/**
 * Initialize this FlutterHeadlessDartRunner with a `FlutterDartProject`.
 *
 * If the FlutterDartProject is not specified, the FlutterHeadlessDartRunner will attempt to locate
 * the project in a default location.
 *
 * A newly initialized engine will not run the `FlutterDartProject` until either
 * `-runWithEntrypoint:` or `-runWithEntrypoint:libraryURI` is called.
 *
 * @param labelPrefix The label prefix used to identify threads for this instance. Should
 * be unique across FlutterEngine instances
 * @param projectOrNil The `FlutterDartProject` to run.
 * @param allowHeadlessExecution Must be set to `YES`.
 */
- (instancetype)initWithName:(NSString*)labelPrefix
                     project:(FlutterDartProject*)projectOrNil
      allowHeadlessExecution:(BOOL)allowHeadlessExecution;

/**
 * Initialize this FlutterHeadlessDartRunner with a `FlutterDartProject`.
 *
 * If the FlutterDartProject is not specified, the FlutterHeadlessDartRunner will attempt to locate
 * the project in a default location.
 *
 * A newly initialized engine will not run the `FlutterDartProject` until either
 * `-runWithEntrypoint:` or `-runWithEntrypoint:libraryURI` is called.
 *
 * @param labelPrefix The label prefix used to identify threads for this instance. Should
 * be unique across FlutterEngine instances
 * @param projectOrNil The `FlutterDartProject` to run.
 * @param allowHeadlessExecution Must be set to `YES`.
 * @param restorationEnabled Must be set to `NO`.
 */
- (instancetype)initWithName:(NSString*)labelPrefix
                     project:(FlutterDartProject*)projectOrNil
      allowHeadlessExecution:(BOOL)allowHeadlessExecution
          restorationEnabled:(BOOL)restorationEnabled NS_DESIGNATED_INITIALIZER;

/**
 * Not recommended for use - will initialize with a default label ("io.flutter.headless")
 * and the default FlutterDartProject.
 */
- (instancetype)init;

@end

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_HEADERS_FLUTTERHEADLESSDARTRUNNER_H_
