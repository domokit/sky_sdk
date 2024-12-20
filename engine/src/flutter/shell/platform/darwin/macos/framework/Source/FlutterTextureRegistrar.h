// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTERTEXTUREREGISTRAR_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTERTEXTUREREGISTRAR_H_

#import <Cocoa/Cocoa.h>

#import "flutter/shell/platform/darwin/macos/framework/Headers/FlutterEngine.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterExternalTexture.h"

/*
 * Delegate methods for FlutterTextureRegistrar.
 */
@protocol FlutterTextureRegistrarDelegate

/*
 * Called by the FlutterTextureRegistrar when a texture is registered.
 */
- (nonnull FlutterExternalTexture*)onRegisterTexture:(nonnull id<FlutterTexture>)texture;

@end

/*
 * Holds the external textures and implements the FlutterTextureRegistry.
 */
@interface FlutterTextureRegistrar : NSObject <FlutterTextureRegistry>

/*
 * Use `initWithDelegate:engine:` instead.
 */
- (nullable instancetype)init NS_UNAVAILABLE;

/*
 * Use `initWithDelegate:engine:` instead.
 */
+ (nullable instancetype)new NS_UNAVAILABLE;

/*
 * Initialzes the texture registrar.
 */
- (nullable instancetype)initWithDelegate:(nonnull id<FlutterTextureRegistrarDelegate>)delegate
                                   engine:(nonnull FlutterEngine*)engine NS_DESIGNATED_INITIALIZER;

/*
 * Returns the registered texture with the provided `textureID`.
 */
- (nullable FlutterExternalTexture*)getTextureWithID:(int64_t)textureID;

@end

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTERTEXTUREREGISTRAR_H_
