// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_GRAPHICS_FLUTTERDARWINCONTEXTMETALSKIA_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_GRAPHICS_FLUTTERDARWINCONTEXTMETALSKIA_H_

#if !SLIMPELLER

#import <CoreVideo/CVMetalTextureCache.h>
#import <Foundation/Foundation.h>
#import <Metal/Metal.h>

#import "flutter/shell/platform/darwin/common/framework/Headers/FlutterTexture.h"
#import "flutter/shell/platform/darwin/graphics/FlutterDarwinExternalTextureMetal.h"
#include "third_party/skia/include/gpu/ganesh/GrDirectContext.h"

NS_ASSUME_NONNULL_BEGIN

/**
 * Provides skia GrContexts that are shared between iOS and macOS embeddings.
 */
@interface FlutterDarwinContextMetalSkia : NSObject

/**
 * Initializes a FlutterDarwinContextMetalSkia with the system default MTLDevice and a new
 * MTLCommandQueue.
 */
- (instancetype)initWithDefaultMTLDevice;

/**
 * Initializes a FlutterDarwinContextMetalSkia with provided MTLDevice and MTLCommandQueue.
 */
- (instancetype)initWithMTLDevice:(id<MTLDevice>)device
                     commandQueue:(id<MTLCommandQueue>)commandQueue;

/**
 * Creates an external texture with the specified ID and contents.
 */
- (FlutterDarwinExternalTextureMetal*)
    createExternalTextureWithIdentifier:(int64_t)textureID
                                texture:(NSObject<FlutterTexture>*)texture;

/**
 * Creates a GrDirectContext with the provided `MTLDevice` and `MTLCommandQueue`.
 */
+ (sk_sp<GrDirectContext>)createGrContext:(id<MTLDevice>)device
                             commandQueue:(id<MTLCommandQueue>)commandQueue;

/**
 * MTLDevice that is backing this context.s
 */
@property(nonatomic, readonly) id<MTLDevice> device;

/**
 * MTLCommandQueue that is acquired from the `device`. This queue is used both for rendering and
 * resource related commands.
 */
@property(nonatomic, readonly) id<MTLCommandQueue> commandQueue;

/**
 * Skia GrContext that is used for rendering.
 */
@property(nonatomic, readonly) sk_sp<GrDirectContext> mainContext;

/**
 * Skia GrContext that is used for resources (uploading textures etc).
 */
@property(nonatomic, readonly) sk_sp<GrDirectContext> resourceContext;

/*
 * Texture cache for external textures.
 */
@property(nonatomic, readonly) CVMetalTextureCacheRef textureCache;

@end

NS_ASSUME_NONNULL_END

#endif  //  !SLIMPELLER

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_GRAPHICS_FLUTTERDARWINCONTEXTMETALSKIA_H_
