// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterRenderer.h"

#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterEngine_Internal.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterExternalTexture.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterViewController_Internal.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterViewEngineProvider.h"
#include "flutter/shell/platform/embedder/embedder.h"

#pragma mark - Static callbacks that require the engine.

static FlutterMetalTexture OnGetNextDrawable(void* user_data, const FlutterFrameInfo* frameInfo) {
  NSCAssert(NO, @"The renderer config should not be used to get the next drawable.");
  return FlutterMetalTexture{};
}

static bool OnPresentDrawable(void* user_data, const FlutterMetalTexture* texture) {
  NSCAssert(NO, @"The renderer config should not be used to present drawable.");
  return false;
}

static bool OnAcquireExternalTexture(void* user_data,
                                     int64_t textureIdentifier,
                                     size_t width,
                                     size_t height,
                                     FlutterMetalExternalTexture* metalTexture) {
  FlutterEngine* engine = (__bridge FlutterEngine*)user_data;
  return [engine.renderer populateTextureWithIdentifier:textureIdentifier
                                           metalTexture:metalTexture];
}

#pragma mark - FlutterRenderer implementation

@implementation FlutterRenderer {
  FlutterDarwinContextMetalSkia* _darwinMetalContext;
}

- (instancetype)initWithFlutterEngine:(nonnull FlutterEngine*)flutterEngine {
  self = [super initWithDelegate:self engine:flutterEngine];
  if (self) {
    _device = MTLCreateSystemDefaultDevice();
    if (!_device) {
      NSLog(@"Could not acquire Metal device.");
      return nil;
    }

    _commandQueue = [_device newCommandQueue];
    if (!_commandQueue) {
      NSLog(@"Could not create Metal command queue.");
      return nil;
    }

    _darwinMetalContext = [[FlutterDarwinContextMetalSkia alloc] initWithMTLDevice:_device
                                                                      commandQueue:_commandQueue];
  }
  return self;
}

- (FlutterRendererConfig)createRendererConfig {
  FlutterRendererConfig config = {
      .type = FlutterRendererType::kMetal,
      .metal = {
          .struct_size = sizeof(FlutterMetalRendererConfig),
          .device = (__bridge FlutterMetalDeviceHandle)_device,
          .present_command_queue = (__bridge FlutterMetalCommandQueueHandle)_commandQueue,
          .get_next_drawable_callback =
              reinterpret_cast<FlutterMetalTextureCallback>(OnGetNextDrawable),
          .present_drawable_callback =
              reinterpret_cast<FlutterMetalPresentCallback>(OnPresentDrawable),
          .external_texture_frame_callback =
              reinterpret_cast<FlutterMetalTextureFrameCallback>(OnAcquireExternalTexture),
      }};
  return config;
}

#pragma mark - Embedder callback implementations.

- (BOOL)populateTextureWithIdentifier:(int64_t)textureID
                         metalTexture:(FlutterMetalExternalTexture*)textureOut {
  FlutterExternalTexture* texture = [self getTextureWithID:textureID];
  return [texture populateTexture:textureOut];
}

#pragma mark - FlutterTextureRegistrar methods.

- (FlutterExternalTexture*)onRegisterTexture:(id<FlutterTexture>)texture {
  return [[FlutterExternalTexture alloc] initWithFlutterTexture:texture
                                             darwinMetalContext:_darwinMetalContext];
}

@end
