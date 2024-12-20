// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTERRENDERER_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTERRENDERER_H_

#import <Cocoa/Cocoa.h>

#import "flutter/shell/platform/darwin/macos/framework/Headers/FlutterEngine.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterTextureRegistrar.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterView.h"
#import "flutter/shell/platform/embedder/embedder.h"

/**
 * Rendering backend agnostic FlutterRendererConfig provider to be used by the embedder API.
 */
@interface FlutterRenderer
    : FlutterTextureRegistrar <FlutterTextureRegistry, FlutterTextureRegistrarDelegate>

/**
 * Interface to the system GPU. Used to issue all the rendering commands.
 */
@property(nonatomic, readonly, nonnull) id<MTLDevice> device;

/**
 * Used to get the command buffers for the MTLDevice to render to.
 */
@property(nonatomic, readonly, nonnull) id<MTLCommandQueue> commandQueue;

/**
 * Intializes the renderer with the given FlutterEngine.
 */
- (nullable instancetype)initWithFlutterEngine:(nonnull FlutterEngine*)flutterEngine;

/**
 * Creates a FlutterRendererConfig that renders using the appropriate backend.
 */
- (FlutterRendererConfig)createRendererConfig;

/**
 * Populates the texture registry with the provided metalTexture.
 */
- (BOOL)populateTextureWithIdentifier:(int64_t)textureID
                         metalTexture:(nonnull FlutterMetalExternalTexture*)metalTexture;

@end

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTERRENDERER_H_
