// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_FLUTTERDARTPROJECT_INTERNAL_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_FLUTTERDARTPROJECT_INTERNAL_H_

#include "flutter/common/settings.h"
#include "flutter/runtime/platform_data.h"
#include "flutter/shell/common/engine.h"
#import "flutter/shell/platform/darwin/common/framework/Headers/FlutterDartProject.h"
#import "flutter/shell/platform/darwin/common/framework/Source/FlutterNSBundleUtils.h"

NS_ASSUME_NONNULL_BEGIN

flutter::Settings FLTDefaultSettingsForBundle(NSBundle* _Nullable bundle = nil,
                                              NSProcessInfo* _Nullable processInfoOrNil = nil);

@interface FlutterDartProject ()

@property(nonatomic, readonly) BOOL isWideGamutEnabled;
@property(nonatomic, readonly) BOOL isImpellerEnabled;

/**
 * This is currently used for *only for tests* to override settings.
 */
- (instancetype)initWithSettings:(const flutter::Settings&)settings;
- (const flutter::Settings&)settings;
- (const flutter::PlatformData)defaultPlatformData;

- (flutter::RunConfiguration)runConfiguration;
- (flutter::RunConfiguration)runConfigurationForEntrypoint:(nullable NSString*)entrypointOrNil;
- (flutter::RunConfiguration)runConfigurationForEntrypoint:(nullable NSString*)entrypointOrNil
                                              libraryOrNil:(nullable NSString*)dartLibraryOrNil;
- (flutter::RunConfiguration)runConfigurationForEntrypoint:(nullable NSString*)entrypointOrNil
                                              libraryOrNil:(nullable NSString*)dartLibraryOrNil
                                            entrypointArgs:
                                                (nullable NSArray<NSString*>*)entrypointArgs;

+ (NSString*)flutterAssetsName:(NSBundle*)bundle;
+ (NSString*)domainNetworkPolicy:(NSDictionary*)appTransportSecurity;
+ (bool)allowsArbitraryLoads:(NSDictionary*)appTransportSecurity;

@end

NS_ASSUME_NONNULL_END

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_FLUTTERDARTPROJECT_INTERNAL_H_
