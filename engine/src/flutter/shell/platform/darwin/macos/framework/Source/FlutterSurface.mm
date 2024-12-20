// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterSurface.h"

#import <CoreMedia/CoreMedia.h>
#import <Metal/Metal.h>

#import "flutter/fml/platform/darwin/cf_utils.h"

@interface FlutterSurface () {
  CGSize _size;
  fml::CFRef<IOSurfaceRef> _ioSurface;
  id<MTLTexture> _texture;
  // Used for testing.
  BOOL _isInUseOverride;
}
@end

@implementation FlutterSurface

- (IOSurfaceRef)ioSurface {
  return _ioSurface;
}

- (CGSize)size {
  return _size;
}

- (int64_t)textureId {
  return reinterpret_cast<int64_t>(_texture);
}

- (BOOL)isInUse {
  return _isInUseOverride || IOSurfaceIsInUse(_ioSurface);
}

- (BOOL)isInUseOverride {
  return _isInUseOverride;
}

- (void)setIsInUseOverride:(BOOL)isInUseOverride {
  _isInUseOverride = isInUseOverride;
}

- (instancetype)initWithSize:(CGSize)size device:(id<MTLDevice>)device {
  if (self = [super init]) {
    self->_size = size;
    self->_ioSurface.Reset([FlutterSurface createIOSurfaceWithSize:size]);
    self->_texture = [FlutterSurface createTextureForIOSurface:_ioSurface size:size device:device];
  }
  return self;
}

- (FlutterMetalTexture)asFlutterMetalTexture {
  return FlutterMetalTexture{
      .struct_size = sizeof(FlutterMetalTexture),
      .texture_id = self.textureId,
      .texture = (__bridge void*)_texture,
      // Retain for use in [FlutterSurface fromFlutterMetalTexture]. Released in
      // destruction_callback.
      .user_data = (__bridge_retained void*)self,
      .destruction_callback =
          [](void* user_data) {
            // Balancing release for the retain when setting user_data above.
            FlutterSurface* surface = (__bridge_transfer FlutterSurface*)user_data;
            surface = nil;
          },
  };
}

+ (FlutterSurface*)fromFlutterMetalTexture:(const FlutterMetalTexture*)texture {
  return (__bridge FlutterSurface*)texture->user_data;
}

+ (IOSurfaceRef)createIOSurfaceWithSize:(CGSize)size {
  unsigned pixelFormat = kCVPixelFormatType_32BGRA;
  unsigned bytesPerElement = 4;

  size_t bytesPerRow = IOSurfaceAlignProperty(kIOSurfaceBytesPerRow, size.width * bytesPerElement);
  size_t totalBytes = IOSurfaceAlignProperty(kIOSurfaceAllocSize, size.height * bytesPerRow);
  NSDictionary* options = @{
    (id)kIOSurfaceWidth : @(size.width),
    (id)kIOSurfaceHeight : @(size.height),
    (id)kIOSurfacePixelFormat : @(pixelFormat),
    (id)kIOSurfaceBytesPerElement : @(bytesPerElement),
    (id)kIOSurfaceBytesPerRow : @(bytesPerRow),
    (id)kIOSurfaceAllocSize : @(totalBytes),
  };

  IOSurfaceRef res = IOSurfaceCreate((CFDictionaryRef)options);
  IOSurfaceSetValue(res, kIOSurfaceColorSpace, kCGColorSpaceSRGB);
  return res;
}

+ (id<MTLTexture>)createTextureForIOSurface:(IOSurfaceRef)surface
                                       size:(CGSize)size
                                     device:(id<MTLDevice>)device {
  MTLTextureDescriptor* textureDescriptor =
      [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:MTLPixelFormatBGRA8Unorm
                                                         width:size.width
                                                        height:size.height
                                                     mipmapped:NO];
  textureDescriptor.usage =
      MTLTextureUsageShaderRead | MTLTextureUsageRenderTarget | MTLTextureUsageShaderWrite;
  // plane = 0 for BGRA.
  return [device newTextureWithDescriptor:textureDescriptor iosurface:surface plane:0];
}

@end
