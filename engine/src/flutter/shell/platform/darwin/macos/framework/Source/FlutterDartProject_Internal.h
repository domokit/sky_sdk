// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTERDARTPROJECT_INTERNAL_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTERDARTPROJECT_INTERNAL_H_

#import "flutter/shell/platform/darwin/common/framework/Headers/FlutterDartProject.h"
#import "flutter/shell/platform/darwin/common/framework/Source/FlutterNSBundleUtils.h"

#include <string>
#include <vector>

/**
 * Provides access to data needed to construct a FlutterProjectArgs for the project.
 */
@interface FlutterDartProject ()

/**
 * The path to the Flutter assets directory.
 */
@property(nonatomic, readonly, nullable) NSString* assetsPath;

/**
 * The path to the ICU data file.
 */
@property(nonatomic, readonly, nullable) NSString* ICUDataPath;

/**
 * The callback invoked by the engine in root isolate scope.
 */
@property(nonatomic, nullable) void (*rootIsolateCreateCallback)(void* _Nullable);

/**
 * Whether the Impeller rendering backend is enabled
 */
@property(nonatomic, readonly) BOOL enableImpeller;

/**
 * Instead of looking up the assets and ICU data path in the application bundle, this initializer
 * allows callers to create a Dart project with custom locations specified for the both.
 */
- (nonnull instancetype)initWithAssetsPath:(nonnull NSString*)assets
                               ICUDataPath:(nonnull NSString*)icuPath NS_DESIGNATED_INITIALIZER;

@end

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTERDARTPROJECT_INTERNAL_H_
