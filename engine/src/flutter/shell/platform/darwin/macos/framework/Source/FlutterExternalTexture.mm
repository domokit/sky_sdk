// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterExternalTexture.h"

#include "flutter/fml/platform/darwin/cf_utils.h"

@implementation FlutterExternalTexture {
  FlutterDarwinContextMetalSkia* _darwinMetalContext;

  int64_t _textureID;

  id<FlutterTexture> _texture;

  std::vector<FlutterMetalTextureHandle> _textures;
}

- (instancetype)initWithFlutterTexture:(id<FlutterTexture>)texture
                    darwinMetalContext:(FlutterDarwinContextMetalSkia*)context {
  self = [super init];
  if (self) {
    _texture = texture;
    _textureID = reinterpret_cast<int64_t>(_texture);
    _darwinMetalContext = context;
  }
  return self;
}

- (int64_t)textureID {
  return _textureID;
}

- (BOOL)populateTexture:(FlutterMetalExternalTexture*)textureOut {
  // Copy the pixel buffer from the FlutterTexture instance implemented on the user side.
  fml::CFRef<CVPixelBufferRef> pixelBuffer([_texture copyPixelBuffer]);

  if (!pixelBuffer) {
    return NO;
  }

  OSType pixel_format = CVPixelBufferGetPixelFormatType(pixelBuffer);
  if (pixel_format == kCVPixelFormatType_420YpCbCr8BiPlanarVideoRange ||
      pixel_format == kCVPixelFormatType_420YpCbCr8BiPlanarFullRange) {
    return [self populateTextureFromYUVAPixelBuffer:pixelBuffer textureOut:textureOut];
  } else if (pixel_format == kCVPixelFormatType_32BGRA) {
    return [self populateTextureFromRGBAPixelBuffer:pixelBuffer textureOut:textureOut];
  } else {
    NSLog(@"Unsupported pixel format: %d", pixel_format);
    return NO;
  }
}

- (BOOL)populateTextureFromYUVAPixelBuffer:(nonnull CVPixelBufferRef)pixelBuffer
                                textureOut:(nonnull FlutterMetalExternalTexture*)textureOut {
  CVMetalTextureRef yCVMetalTexture = nullptr;
  CVMetalTextureRef uvCVMetalTextureRef = nullptr;
  SkISize textureSize =
      SkISize::Make(CVPixelBufferGetWidth(pixelBuffer), CVPixelBufferGetHeight(pixelBuffer));

  CVReturn yCVReturn = CVMetalTextureCacheCreateTextureFromImage(
      /*allocator=*/kCFAllocatorDefault,
      /*textureCache=*/_darwinMetalContext.textureCache,
      /*sourceImage=*/pixelBuffer,
      /*textureAttributes=*/nullptr,
      /*pixelFormat=*/MTLPixelFormatR8Unorm,
      /*width=*/CVPixelBufferGetWidthOfPlane(pixelBuffer, 0u),
      /*height=*/CVPixelBufferGetHeightOfPlane(pixelBuffer, 0u),
      /*planeIndex=*/0u,
      /*texture=*/&yCVMetalTexture);

  if (yCVReturn != kCVReturnSuccess) {
    NSLog(@"Could not create Metal texture from pixel buffer: CVReturn %d", yCVReturn);
    return NO;
  }

  CVReturn uvCVReturn = CVMetalTextureCacheCreateTextureFromImage(
      /*allocator=*/kCFAllocatorDefault,
      /*textureCache=*/_darwinMetalContext.textureCache,
      /*sourceImage=*/pixelBuffer,
      /*textureAttributes=*/nullptr,
      /*pixelFormat=*/MTLPixelFormatRG8Unorm,
      /*width=*/CVPixelBufferGetWidthOfPlane(pixelBuffer, 1u),
      /*height=*/CVPixelBufferGetHeightOfPlane(pixelBuffer, 1u),
      /*planeIndex=*/1u,
      /*texture=*/&uvCVMetalTextureRef);

  if (uvCVReturn != kCVReturnSuccess) {
    CVBufferRelease(yCVMetalTexture);
    NSLog(@"Could not create Metal texture from pixel buffer: CVReturn %d", uvCVReturn);
    return NO;
  }

  _textures = {(__bridge FlutterMetalTextureHandle)CVMetalTextureGetTexture(yCVMetalTexture),
               (__bridge FlutterMetalTextureHandle)CVMetalTextureGetTexture(uvCVMetalTextureRef)};
  CVBufferRelease(yCVMetalTexture);
  CVBufferRelease(uvCVMetalTextureRef);

  textureOut->num_textures = 2;
  textureOut->height = textureSize.height();
  textureOut->width = textureSize.width();
  textureOut->pixel_format = FlutterMetalExternalTexturePixelFormat::kYUVA;
  textureOut->textures = _textures.data();
  OSType pixel_format = CVPixelBufferGetPixelFormatType(pixelBuffer);
  textureOut->yuv_color_space = pixel_format == kCVPixelFormatType_420YpCbCr8BiPlanarVideoRange
                                    ? FlutterMetalExternalTextureYUVColorSpace::kBT601LimitedRange
                                    : FlutterMetalExternalTextureYUVColorSpace::kBT601FullRange;

  return YES;
}

- (BOOL)populateTextureFromRGBAPixelBuffer:(nonnull CVPixelBufferRef)pixelBuffer
                                textureOut:(nonnull FlutterMetalExternalTexture*)textureOut {
  SkISize textureSize =
      SkISize::Make(CVPixelBufferGetWidth(pixelBuffer), CVPixelBufferGetHeight(pixelBuffer));

  CVMetalTextureRef cvMetalTexture = nullptr;
  CVReturn cvReturn =
      CVMetalTextureCacheCreateTextureFromImage(/*allocator=*/kCFAllocatorDefault,
                                                /*textureCache=*/_darwinMetalContext.textureCache,
                                                /*sourceImage=*/pixelBuffer,
                                                /*textureAttributes=*/nullptr,
                                                /*pixelFormat=*/MTLPixelFormatBGRA8Unorm,
                                                /*width=*/textureSize.width(),
                                                /*height=*/textureSize.height(),
                                                /*planeIndex=*/0u,
                                                /*texture=*/&cvMetalTexture);

  if (cvReturn != kCVReturnSuccess) {
    NSLog(@"Could not create Metal texture from pixel buffer: CVReturn %d", cvReturn);
    return NO;
  }

  _textures = {(__bridge FlutterMetalTextureHandle)CVMetalTextureGetTexture(cvMetalTexture)};
  CVBufferRelease(cvMetalTexture);

  textureOut->num_textures = 1;
  textureOut->height = textureSize.height();
  textureOut->width = textureSize.width();
  textureOut->pixel_format = FlutterMetalExternalTexturePixelFormat::kRGBA;
  textureOut->textures = _textures.data();

  return YES;
}

@end
