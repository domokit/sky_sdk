// Copyright 2019 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import "ContinuousTexture.h"

@implementation ContinuousTexture

+ (void)registerWithRegistrar:(nonnull NSObject<FlutterPluginRegistrar>*)registrar {
  NSObject<FlutterTextureRegistry>* textureRegistry = [registrar textures];
  FlutterScenarioTestTexture* texture = [[FlutterScenarioTestTexture alloc] init];
  int64_t textureId = [textureRegistry registerTexture:texture];
  [NSTimer scheduledTimerWithTimeInterval:0.05
                                  repeats:YES
                                    block:^(NSTimer* _Nonnull timer) {
                                      [textureRegistry textureFrameAvailable:textureId];
                                    }];
}

@end

@implementation FlutterScenarioTestTexture

- (CVPixelBufferRef _Nullable)copyPixelBuffer {
  return [self pixelBuffer];
}

- (CVPixelBufferRef)pixelBuffer {
  NSDictionary* options = @{
    // This key is required to generate SKPicture with CVPixelBufferRef in metal.
    (NSString*)kCVPixelBufferMetalCompatibilityKey : @YES
  };
  CVPixelBufferRef pxbuffer = NULL;
  CVReturn status = CVPixelBufferCreate(kCFAllocatorDefault, 200, 200, kCVPixelFormatType_32BGRA,
                                        (__bridge CFDictionaryRef)options, &pxbuffer);

  NSAssert(status == kCVReturnSuccess && pxbuffer != NULL, @"Failed to create pixel buffer.");
  return pxbuffer;
}

@end
