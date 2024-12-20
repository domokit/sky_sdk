// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTERSURFACE_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTERSURFACE_H_

#import <Cocoa/Cocoa.h>

#import "flutter/shell/platform/embedder/embedder.h"

/**
 * Opaque surface type.
 * Can be represented as FlutterMetalTexture to cross the embedder API boundary.
 */
@interface FlutterSurface : NSObject

- (FlutterMetalTexture)asFlutterMetalTexture;

+ (nullable FlutterSurface*)fromFlutterMetalTexture:(nonnull const FlutterMetalTexture*)texture;

@end

/**
 * Internal FlutterSurface interface used by FlutterSurfaceManager.
 * Wraps an IOSurface framebuffer and metadata related to the surface.
 */
@interface FlutterSurface (Private)

- (nonnull instancetype)initWithSize:(CGSize)size device:(nonnull id<MTLDevice>)device;

@property(readonly, nonatomic, nonnull) IOSurfaceRef ioSurface;
@property(readonly, nonatomic) CGSize size;
@property(readonly, nonatomic) int64_t textureId;
// Whether the surface is currently in use by the compositor.
@property(readonly, nonatomic) BOOL isInUse;

@end

@interface FlutterSurface (Testing)
@property(readwrite, nonatomic) BOOL isInUseOverride;
@end

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTERSURFACE_H_
