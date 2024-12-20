// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#if !SLIMPELLER

#import "flutter/shell/platform/darwin/graphics/FlutterDarwinContextMetalSkia.h"

#include "flutter/common/graphics/persistent_cache.h"
#include "flutter/fml/logging.h"
#include "flutter/fml/platform/darwin/cf_utils.h"
#include "flutter/shell/common/context_options.h"
#import "flutter/shell/platform/darwin/common/framework/Headers/FlutterMacros.h"
#include "third_party/skia/include/gpu/ganesh/GrDirectContext.h"
#include "third_party/skia/include/gpu/ganesh/mtl/GrMtlBackendContext.h"
#include "third_party/skia/include/gpu/ganesh/mtl/GrMtlDirectContext.h"

FLUTTER_ASSERT_ARC

@implementation FlutterDarwinContextMetalSkia {
  fml::CFRef<CVMetalTextureCacheRef> _textureCache;
}

- (instancetype)initWithDefaultMTLDevice {
  id<MTLDevice> device = MTLCreateSystemDefaultDevice();
  return [self initWithMTLDevice:device commandQueue:[device newCommandQueue]];
}

- (instancetype)initWithMTLDevice:(id<MTLDevice>)device
                     commandQueue:(id<MTLCommandQueue>)commandQueue {
  self = [super init];
  if (self != nil) {
    _device = device;

    if (!_device) {
      FML_DLOG(ERROR) << "Could not acquire Metal device.";
      return nil;
    }

    _commandQueue = commandQueue;

    if (!_commandQueue) {
      FML_DLOG(ERROR) << "Could not create Metal command queue.";
      return nil;
    }

    [_commandQueue setLabel:@"Flutter Main Queue"];

    {
      CVMetalTextureCacheRef cache = nullptr;
      CVReturn cvReturn = CVMetalTextureCacheCreate(kCFAllocatorDefault,  // allocator
                                                    nil,      // cache attributes (nil default)
                                                    _device,  // metal device
                                                    nil,      // texture attributes (nil default)
                                                    &cache    // [out] cache
      );
      _textureCache.Reset(cache);
      if (cvReturn != kCVReturnSuccess) {
        FML_DLOG(ERROR) << "Could not create Metal texture cache.";
        return nil;
      }
    }

    // The devices are in the same "sharegroup" because they share the same device and command
    // queues for now. When the resource context gets its own transfer queue, this will have to be
    // refactored.
    _mainContext = [self createGrContext];
    _resourceContext = [self createGrContext];

    if (!_mainContext || !_resourceContext) {
      FML_DLOG(ERROR) << "Could not create Skia Metal contexts.";
      return nil;
    }

    // Only log this message on iOS where the default is Impeller. On macOS
    // desktop, Skia is still the default and this log is unecessary.
#if defined(FML_OS_IOS) || defined(FML_OS_IOS_SIM)
    FML_LOG(IMPORTANT) << "Using the Skia rendering backend (Metal).";
#endif  // defined(FML_OS_IOS) || defined(FML_OS_IOS_SIM)

    _resourceContext->setResourceCacheLimit(0u);
  }
  return self;
}

- (sk_sp<GrDirectContext>)createGrContext {
  id<MTLDevice> device = _device;
  id<MTLCommandQueue> commandQueue = _commandQueue;
  return [FlutterDarwinContextMetalSkia createGrContext:device commandQueue:commandQueue];
}

+ (sk_sp<GrDirectContext>)createGrContext:(id<MTLDevice>)device
                             commandQueue:(id<MTLCommandQueue>)commandQueue {
  const auto contextOptions =
      flutter::MakeDefaultContextOptions(flutter::ContextType::kRender, GrBackendApi::kMetal);
  GrMtlBackendContext backendContext = {};
  // Skia expect arguments to `MakeMetal` transfer ownership of the reference in for release later
  // when the GrDirectContext is collected.
  backendContext.fDevice.retain((__bridge void*)device);
  backendContext.fQueue.retain((__bridge void*)commandQueue);
  return GrDirectContexts::MakeMetal(backendContext, contextOptions);
}

- (FlutterDarwinExternalTextureMetal*)
    createExternalTextureWithIdentifier:(int64_t)textureID
                                texture:(NSObject<FlutterTexture>*)texture {
  return [[FlutterDarwinExternalTextureMetal alloc] initWithTextureCache:_textureCache
                                                               textureID:textureID
                                                                 texture:texture
                                                          enableImpeller:NO];
}

- (CVMetalTextureCacheRef)textureCache {
  return _textureCache;
}

@end

#endif  //  !SLIMPELLER
