// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "flutter/shell/platform/embedder/embedder_external_texture_metal.h"

#include "flow/layers/layer.h"
#include "flutter/fml/logging.h"
#import "flutter/shell/platform/darwin/graphics/FlutterDarwinExternalTextureMetal.h"
#include "third_party/skia/include/core/SkImage.h"
#include "third_party/skia/include/core/SkSize.h"
#include "third_party/skia/include/gpu/ganesh/GrBackendSurface.h"
#include "third_party/skia/include/gpu/ganesh/GrDirectContext.h"

namespace flutter {

static bool ValidNumTextures(int expected, int actual) {
  if (expected == actual) {
    return true;
  } else {
    FML_LOG(ERROR) << "Invalid number of textures, expected: " << expected << ", got: " << actual;
    return false;
  }
}

EmbedderExternalTextureMetal::EmbedderExternalTextureMetal(int64_t texture_identifier,
                                                           const ExternalTextureCallback& callback)
    : Texture(texture_identifier), external_texture_callback_(callback) {
  FML_DCHECK(external_texture_callback_);
}

EmbedderExternalTextureMetal::~EmbedderExternalTextureMetal() = default;

// |flutter::Texture|
void EmbedderExternalTextureMetal::Paint(PaintContext& context,
                                         const SkRect& bounds,
                                         bool freeze,
                                         const DlImageSampling sampling) {
  if (last_image_ == nullptr) {
    last_image_ = ResolveTexture(Id(), context.gr_context, context.aiks_context,
                                 SkISize::Make(bounds.width(), bounds.height()));
  }

  DlCanvas* canvas = context.canvas;
  const DlPaint* paint = context.paint;

  if (last_image_) {
    SkRect image_bounds = SkRect::Make(last_image_->bounds());
    if (bounds != image_bounds) {
      canvas->DrawImageRect(last_image_, image_bounds, bounds, sampling, paint);
    } else {
      canvas->DrawImage(last_image_, SkPoint{bounds.x(), bounds.y()}, sampling, paint);
    }
  }
}

sk_sp<DlImage> EmbedderExternalTextureMetal::ResolveTexture(int64_t texture_id,
                                                            GrDirectContext* context,
                                                            impeller::AiksContext* aiks_context,
                                                            const SkISize& size) {
  std::unique_ptr<FlutterMetalExternalTexture> texture =
      external_texture_callback_(texture_id, size.width(), size.height());

  if (!texture) {
    return nullptr;
  }
  if (aiks_context) {
    switch (texture->pixel_format) {
      case FlutterMetalExternalTexturePixelFormat::kRGBA: {
        if (ValidNumTextures(1, texture->num_textures)) {
          id<MTLTexture> rgbaTex = (__bridge id<MTLTexture>)texture->textures[0];
          return [FlutterDarwinExternalTextureImpellerImageWrapper wrapRGBATexture:rgbaTex
                                                                       aiksContext:aiks_context];
        }
        break;
      }
      case FlutterMetalExternalTexturePixelFormat::kYUVA: {
        if (ValidNumTextures(2, texture->num_textures)) {
          id<MTLTexture> yTex = (__bridge id<MTLTexture>)texture->textures[0];
          id<MTLTexture> uvTex = (__bridge id<MTLTexture>)texture->textures[1];
          impeller::YUVColorSpace colorSpace =
              texture->yuv_color_space ==
                      FlutterMetalExternalTextureYUVColorSpace::kBT601LimitedRange
                  ? impeller::YUVColorSpace::kBT601LimitedRange
                  : impeller::YUVColorSpace::kBT601FullRange;
          return [FlutterDarwinExternalTextureImpellerImageWrapper wrapYUVATexture:yTex
                                                                             UVTex:uvTex
                                                                     YUVColorSpace:colorSpace
                                                                       aiksContext:aiks_context];
        }
        break;
      }
    }
    return nullptr;
  }

  sk_sp<SkImage> image;

  switch (texture->pixel_format) {
    case FlutterMetalExternalTexturePixelFormat::kRGBA: {
      if (ValidNumTextures(1, texture->num_textures)) {
        id<MTLTexture> rgbaTex = (__bridge id<MTLTexture>)texture->textures[0];
        image = [FlutterDarwinExternalTextureSkImageWrapper wrapRGBATexture:rgbaTex
                                                                  grContext:context
                                                                      width:rgbaTex.width
                                                                     height:rgbaTex.height];
      }
      break;
    }
    case FlutterMetalExternalTexturePixelFormat::kYUVA: {
      if (ValidNumTextures(2, texture->num_textures)) {
        id<MTLTexture> yTex = (__bridge id<MTLTexture>)texture->textures[0];
        id<MTLTexture> uvTex = (__bridge id<MTLTexture>)texture->textures[1];
        SkYUVColorSpace colorSpace =
            texture->yuv_color_space == FlutterMetalExternalTextureYUVColorSpace::kBT601LimitedRange
                ? kRec601_Limited_SkYUVColorSpace
                : kJPEG_Full_SkYUVColorSpace;
        image = [FlutterDarwinExternalTextureSkImageWrapper wrapYUVATexture:yTex
                                                                      UVTex:uvTex
                                                              YUVColorSpace:colorSpace
                                                                  grContext:context
                                                                      width:size.width()
                                                                     height:size.height()];
      }
      break;
    }
  }

  if (!image) {
    FML_LOG(ERROR) << "Could not create external texture: " << texture_id;
  }

  // This image should not escape local use by EmbedderExternalTextureMetal
  return DlImage::Make(std::move(image));
}

// |flutter::Texture|
void EmbedderExternalTextureMetal::OnGrContextCreated() {}

// |flutter::Texture|
void EmbedderExternalTextureMetal::OnGrContextDestroyed() {}

// |flutter::Texture|
void EmbedderExternalTextureMetal::MarkNewFrameAvailable() {
  last_image_ = nullptr;
}

// |flutter::Texture|
void EmbedderExternalTextureMetal::OnTextureUnregistered() {}

}  // namespace flutter
