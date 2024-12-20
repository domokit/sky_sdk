// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_GRAPHICS_FLUTTERDARWINCONTEXTMETALIMPELLER_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_GRAPHICS_FLUTTERDARWINCONTEXTMETALIMPELLER_H_

#import <CoreVideo/CVMetalTextureCache.h>
#import <Foundation/Foundation.h>
#import <Metal/Metal.h>

#include "flutter/fml/concurrent_message_loop.h"
#include "flutter/fml/platform/darwin/cf_utils.h"
#import "flutter/shell/platform/darwin/common/framework/Headers/FlutterTexture.h"
#import "flutter/shell/platform/darwin/graphics/FlutterDarwinExternalTextureMetal.h"
#include "impeller/renderer/backend/metal/context_mtl.h"

NS_ASSUME_NONNULL_BEGIN

/**
 * Provides skia GrContexts that are shared between iOS and macOS embeddings.
 */
@interface FlutterDarwinContextMetalImpeller : NSObject

/**
 * Initializes a FlutterDarwinContextMetalImpeller.
 */
- (instancetype)init:(const std::shared_ptr<const fml::SyncSwitch>&)is_gpu_disabled_sync_switch;

/**
 * Creates an external texture with the specified ID and contents.
 */
- (FlutterDarwinExternalTextureMetal*)
    createExternalTextureWithIdentifier:(int64_t)textureID
                                texture:(NSObject<FlutterTexture>*)texture;

/**
 * Impeller context.
 */
@property(nonatomic, readonly) std::shared_ptr<impeller::ContextMTL> context;

/*
 * Texture cache for external textures.
 */
@property(nonatomic, readonly) fml::CFRef<CVMetalTextureCacheRef> textureCache;

@end

NS_ASSUME_NONNULL_END

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_GRAPHICS_FLUTTERDARWINCONTEXTMETALIMPELLER_H_
