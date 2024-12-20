// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_GRAPHICS_FLUTTERDARWINEXTERNALTEXTUREMETAL_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_GRAPHICS_FLUTTERDARWINEXTERNALTEXTUREMETAL_H_

#import <Foundation/Foundation.h>
#import <Metal/Metal.h>

#include "flutter/common/graphics/texture.h"
#include "flutter/display_list/image/dl_image.h"
#include "flutter/impeller/display_list/aiks_context.h"
#import "flutter/shell/platform/darwin/common/framework/Headers/FlutterTexture.h"
#include "third_party/skia/include/core/SkCanvas.h"
#include "third_party/skia/include/core/SkImage.h"

@interface FlutterDarwinExternalTextureSkImageWrapper : NSObject

+ (sk_sp<SkImage>)wrapYUVATexture:(nonnull id<MTLTexture>)yTex
                            UVTex:(nonnull id<MTLTexture>)uvTex
                    YUVColorSpace:(SkYUVColorSpace)colorSpace
                        grContext:(nonnull GrDirectContext*)grContext
                            width:(size_t)width
                           height:(size_t)height;

+ (sk_sp<SkImage>)wrapRGBATexture:(nonnull id<MTLTexture>)rgbaTex
                        grContext:(nonnull GrDirectContext*)grContext
                            width:(size_t)width
                           height:(size_t)height;

@end

@interface FlutterDarwinExternalTextureImpellerImageWrapper : NSObject

+ (sk_sp<flutter::DlImage>)wrapYUVATexture:(nonnull id<MTLTexture>)yTex
                                     UVTex:(nonnull id<MTLTexture>)uvTex
                             YUVColorSpace:(impeller::YUVColorSpace)colorSpace
                               aiksContext:(nonnull impeller::AiksContext*)aiksContext;

+ (sk_sp<flutter::DlImage>)wrapRGBATexture:(nonnull id<MTLTexture>)rgbaTex
                               aiksContext:(nonnull impeller::AiksContext*)aiks_context;

@end

@interface FlutterDarwinExternalTextureMetal : NSObject

- (nullable instancetype)initWithTextureCache:(nonnull CVMetalTextureCacheRef)textureCache
                                    textureID:(int64_t)textureID
                                      texture:(nonnull NSObject<FlutterTexture>*)texture
                               enableImpeller:(BOOL)enableImpeller;

- (void)paintContext:(flutter::Texture::PaintContext&)context
              bounds:(const SkRect&)bounds
              freeze:(BOOL)freeze
            sampling:(const flutter::DlImageSampling)sampling;

- (void)onGrContextCreated;

- (void)onGrContextDestroyed;

- (void)markNewFrameAvailable;

- (void)onTextureUnregistered;

@property(nonatomic, readonly) int64_t textureID;

@end

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_GRAPHICS_FLUTTERDARWINEXTERNALTEXTUREMETAL_H_
