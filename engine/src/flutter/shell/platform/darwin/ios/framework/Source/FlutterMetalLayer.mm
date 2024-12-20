// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterMetalLayer.h"

#include <CoreMedia/CoreMedia.h>
#include <IOSurface/IOSurfaceObjC.h>
#include <Metal/Metal.h>
#include <UIKit/UIKit.h>

#include "flutter/fml/logging.h"
#import "flutter/shell/platform/darwin/common/framework/Headers/FlutterMacros.h"

FLUTTER_ASSERT_ARC

@interface DisplayLinkManager : NSObject
@property(class, nonatomic, readonly) BOOL maxRefreshRateEnabledOnIPhone;
+ (double)displayRefreshRate;
@end

@class FlutterTexture;
@class FlutterDrawable;

extern CFTimeInterval display_link_target;

@interface FlutterMetalLayer () {
  id<MTLDevice> _preferredDevice;
  CGSize _drawableSize;

  NSUInteger _nextDrawableId;

  NSMutableSet<FlutterTexture*>* _availableTextures;
  NSUInteger _totalTextures;

  FlutterTexture* _front;

  // There must be a CADisplayLink scheduled *on main thread* otherwise
  // core animation only updates layers 60 times a second.
  CADisplayLink* _displayLink;
  NSUInteger _displayLinkPauseCountdown;

  // Used to track whether the content was set during this display link.
  // When unlocking phone the layer (main thread) display link and raster thread
  // display link get out of sync for several seconds. Even worse, layer display
  // link does not seem to reflect actual vsync. Forcing the layer link
  // to max rate (instead range) temporarily seems to fix the issue.
  BOOL _didSetContentsDuringThisDisplayLinkPeriod;

  // Whether layer displayLink is forced to max rate.
  BOOL _displayLinkForcedMaxRate;
}

- (void)onDisplayLink:(CADisplayLink*)link;
- (void)presentTexture:(FlutterTexture*)texture;
- (void)returnTexture:(FlutterTexture*)texture;

@end

@interface FlutterTexture : NSObject

@property(readonly, nonatomic) id<MTLTexture> texture;
@property(readonly, nonatomic) IOSurface* surface;
@property(readwrite, nonatomic) CFTimeInterval presentedTime;
@property(readwrite, atomic) BOOL waitingForCompletion;

@end

@implementation FlutterTexture

- (instancetype)initWithTexture:(id<MTLTexture>)texture surface:(IOSurface*)surface {
  if (self = [super init]) {
    _texture = texture;
    _surface = surface;
  }
  return self;
}

@end

@interface FlutterDrawable : NSObject <FlutterMetalDrawable> {
  FlutterTexture* _texture;
  __weak FlutterMetalLayer* _layer;
  NSUInteger _drawableId;
  BOOL _presented;
}

- (instancetype)initWithTexture:(FlutterTexture*)texture
                          layer:(FlutterMetalLayer*)layer
                     drawableId:(NSUInteger)drawableId;

@end

@implementation FlutterDrawable

- (instancetype)initWithTexture:(FlutterTexture*)texture
                          layer:(FlutterMetalLayer*)layer
                     drawableId:(NSUInteger)drawableId {
  if (self = [super init]) {
    _texture = texture;
    _layer = layer;
    _drawableId = drawableId;
  }
  return self;
}

- (id<MTLTexture>)texture {
  return self->_texture.texture;
}

#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunguarded-availability-new"
- (CAMetalLayer*)layer {
  return (id)self->_layer;
}
#pragma clang diagnostic pop

- (NSUInteger)drawableID {
  return self->_drawableId;
}

- (CFTimeInterval)presentedTime {
  return 0;
}

- (void)present {
  [_layer presentTexture:self->_texture];
  self->_presented = YES;
}

- (void)dealloc {
  if (!_presented) {
    [_layer returnTexture:self->_texture];
  }
}

- (void)addPresentedHandler:(nonnull MTLDrawablePresentedHandler)block {
  FML_LOG(WARNING) << "FlutterMetalLayer drawable does not implement addPresentedHandler:";
}

- (void)presentAtTime:(CFTimeInterval)presentationTime {
  FML_LOG(WARNING) << "FlutterMetalLayer drawable does not implement presentAtTime:";
}

- (void)presentAfterMinimumDuration:(CFTimeInterval)duration {
  FML_LOG(WARNING) << "FlutterMetalLayer drawable does not implement presentAfterMinimumDuration:";
}

- (void)flutterPrepareForPresent:(nonnull id<MTLCommandBuffer>)commandBuffer {
  FlutterTexture* texture = _texture;
  texture.waitingForCompletion = YES;
  [commandBuffer addCompletedHandler:^(id<MTLCommandBuffer> buffer) {
    texture.waitingForCompletion = NO;
  }];
}

@end

@interface FlutterMetalLayerDisplayLinkProxy : NSObject {
  __weak FlutterMetalLayer* _layer;
}

@end

@implementation FlutterMetalLayerDisplayLinkProxy
- (instancetype)initWithLayer:(FlutterMetalLayer*)layer {
  if (self = [super init]) {
    _layer = layer;
  }
  return self;
}

- (void)onDisplayLink:(CADisplayLink*)link {
  [_layer onDisplayLink:link];
}

@end

@implementation FlutterMetalLayer

- (instancetype)init {
  if (self = [super init]) {
    _preferredDevice = MTLCreateSystemDefaultDevice();
    self.device = self.preferredDevice;
    self.pixelFormat = MTLPixelFormatBGRA8Unorm;
    _availableTextures = [[NSMutableSet alloc] init];

    FlutterMetalLayerDisplayLinkProxy* proxy =
        [[FlutterMetalLayerDisplayLinkProxy alloc] initWithLayer:self];
    _displayLink = [CADisplayLink displayLinkWithTarget:proxy selector:@selector(onDisplayLink:)];
    [self setMaxRefreshRate:DisplayLinkManager.displayRefreshRate forceMax:NO];
    [_displayLink addToRunLoop:[NSRunLoop currentRunLoop] forMode:NSRunLoopCommonModes];
    [[NSNotificationCenter defaultCenter] addObserver:self
                                             selector:@selector(didEnterBackground:)
                                                 name:UIApplicationDidEnterBackgroundNotification
                                               object:nil];
  }
  return self;
}

- (void)dealloc {
  [_displayLink invalidate];
  [[NSNotificationCenter defaultCenter] removeObserver:self];
}

- (void)setMaxRefreshRate:(double)refreshRate forceMax:(BOOL)forceMax {
  // This is copied from vsync_waiter_ios.mm. The vsync waiter has display link scheduled on UI
  // thread which does not trigger actual core animation frame. As a workaround FlutterMetalLayer
  // has it's own displaylink scheduled on main thread, which is used to trigger core animation
  // frame allowing for 120hz updates.
  if (!DisplayLinkManager.maxRefreshRateEnabledOnIPhone) {
    return;
  }
  double maxFrameRate = fmax(refreshRate, 60);
  double minFrameRate = fmax(maxFrameRate / 2, 60);
  if (@available(iOS 15.0, *)) {
    _displayLink.preferredFrameRateRange =
        CAFrameRateRangeMake(forceMax ? maxFrameRate : minFrameRate, maxFrameRate, maxFrameRate);
  } else {
    _displayLink.preferredFramesPerSecond = maxFrameRate;
  }
}

- (void)onDisplayLink:(CADisplayLink*)link {
  _didSetContentsDuringThisDisplayLinkPeriod = NO;
  // Do not pause immediately, this seems to prevent 120hz while touching.
  if (_displayLinkPauseCountdown == 3) {
    _displayLink.paused = YES;
    if (_displayLinkForcedMaxRate) {
      [self setMaxRefreshRate:DisplayLinkManager.displayRefreshRate forceMax:NO];
      _displayLinkForcedMaxRate = NO;
    }
  } else {
    ++_displayLinkPauseCountdown;
  }
}

- (BOOL)isKindOfClass:(Class)aClass {
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunguarded-availability-new"
  // Pretend that we're a CAMetalLayer so that the rest of Flutter plays along
  if ([aClass isEqual:[CAMetalLayer class]]) {
    return YES;
  }
#pragma clang diagnostic pop
  return [super isKindOfClass:aClass];
}

- (void)setDrawableSize:(CGSize)drawableSize {
  [_availableTextures removeAllObjects];
  _front = nil;
  _totalTextures = 0;
  _drawableSize = drawableSize;
}

- (void)didEnterBackground:(id)notification {
  [_availableTextures removeAllObjects];
  _totalTextures = _front != nil ? 1 : 0;
  _displayLink.paused = YES;
}

- (CGSize)drawableSize {
  return _drawableSize;
}

- (IOSurface*)createIOSurface {
  unsigned pixelFormat;
  unsigned bytesPerElement;
  if (self.pixelFormat == MTLPixelFormatRGBA16Float) {
    pixelFormat = kCVPixelFormatType_64RGBAHalf;
    bytesPerElement = 8;
  } else if (self.pixelFormat == MTLPixelFormatBGRA8Unorm) {
    pixelFormat = kCVPixelFormatType_32BGRA;
    bytesPerElement = 4;
  } else if (self.pixelFormat == MTLPixelFormatBGRA10_XR) {
    pixelFormat = kCVPixelFormatType_40ARGBLEWideGamut;
    bytesPerElement = 8;
  } else {
    FML_LOG(ERROR) << "Unsupported pixel format: " << self.pixelFormat;
    return nil;
  }
  size_t bytesPerRow =
      IOSurfaceAlignProperty(kIOSurfaceBytesPerRow, _drawableSize.width * bytesPerElement);
  size_t totalBytes =
      IOSurfaceAlignProperty(kIOSurfaceAllocSize, _drawableSize.height * bytesPerRow);
  NSDictionary* options = @{
    (id)kIOSurfaceWidth : @(_drawableSize.width),
    (id)kIOSurfaceHeight : @(_drawableSize.height),
    (id)kIOSurfacePixelFormat : @(pixelFormat),
    (id)kIOSurfaceBytesPerElement : @(bytesPerElement),
    (id)kIOSurfaceBytesPerRow : @(bytesPerRow),
    (id)kIOSurfaceAllocSize : @(totalBytes),
  };

  IOSurfaceRef res = IOSurfaceCreate((CFDictionaryRef)options);
  if (res == nil) {
    FML_LOG(ERROR) << "Failed to create IOSurface with options "
                   << options.debugDescription.UTF8String;
    return nil;
  }

  if (self.colorspace != nil) {
    CFStringRef name = CGColorSpaceGetName(self.colorspace);
    IOSurfaceSetValue(res, kIOSurfaceColorSpace, name);
  } else {
    IOSurfaceSetValue(res, kIOSurfaceColorSpace, kCGColorSpaceSRGB);
  }
  return (__bridge_transfer IOSurface*)res;
}

- (FlutterTexture*)nextTexture {
  CFTimeInterval start = CACurrentMediaTime();
  while (true) {
    FlutterTexture* texture = [self tryNextTexture];
    if (texture != nil) {
      return texture;
    }
    CFTimeInterval elapsed = CACurrentMediaTime() - start;
    if (elapsed > 1.0) {
      NSLog(@"Waited %f seconds for a drawable, giving up.", elapsed);
      return nil;
    }
  }
}

- (FlutterTexture*)tryNextTexture {
  @synchronized(self) {
    if (_front != nil && _front.waitingForCompletion) {
      return nil;
    }
    if (_totalTextures < 3) {
      ++_totalTextures;
      IOSurface* surface = [self createIOSurface];
      if (surface == nil) {
        return nil;
      }
      MTLTextureDescriptor* textureDescriptor =
          [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:_pixelFormat
                                                             width:_drawableSize.width
                                                            height:_drawableSize.height
                                                         mipmapped:NO];

      if (_framebufferOnly) {
        textureDescriptor.usage = MTLTextureUsageRenderTarget;
      } else {
        textureDescriptor.usage =
            MTLTextureUsageRenderTarget | MTLTextureUsageShaderRead | MTLTextureUsageShaderWrite;
      }
      id<MTLTexture> texture = [self.device newTextureWithDescriptor:textureDescriptor
                                                           iosurface:(__bridge IOSurfaceRef)surface
                                                               plane:0];
      FlutterTexture* flutterTexture = [[FlutterTexture alloc] initWithTexture:texture
                                                                       surface:surface];
      return flutterTexture;
    } else {
      // Prefer surface that is not in use and has been presented the longest
      // time ago.
      // When isInUse is false, the surface is definitely not used by the compositor.
      // When isInUse is true, the surface may be used by the compositor.
      // When both surfaces are in use, the one presented earlier will be returned.
      // The assumption here is that the compositor is already aware of the
      // newer texture and is unlikely to read from the older one, even though it
      // has not decreased the use count yet (there seems to be certain latency).
      FlutterTexture* res = nil;
      for (FlutterTexture* texture in _availableTextures) {
        if (res == nil) {
          res = texture;
        } else if (res.surface.isInUse && !texture.surface.isInUse) {
          // prefer texture that is not in use.
          res = texture;
        } else if (res.surface.isInUse == texture.surface.isInUse &&
                   texture.presentedTime < res.presentedTime) {
          // prefer texture with older presented time.
          res = texture;
        }
      }
      if (res != nil) {
        [_availableTextures removeObject:res];
      }
      return res;
    }
  }
}

- (id<CAMetalDrawable>)nextDrawable {
  FlutterTexture* texture = [self nextTexture];
  if (texture == nil) {
    return nil;
  }
  FlutterDrawable* drawable = [[FlutterDrawable alloc] initWithTexture:texture
                                                                 layer:self
                                                            drawableId:_nextDrawableId++];
  return drawable;
}

- (void)presentOnMainThread:(FlutterTexture*)texture {
  // This is needed otherwise frame gets skipped on touch begin / end. Go figure.
  // Might also be placebo
  [self setNeedsDisplay];

  [CATransaction begin];
  [CATransaction setDisableActions:YES];
  self.contents = texture.surface;
  [CATransaction commit];
  _displayLink.paused = NO;
  _displayLinkPauseCountdown = 0;
  if (!_didSetContentsDuringThisDisplayLinkPeriod) {
    _didSetContentsDuringThisDisplayLinkPeriod = YES;
  } else if (!_displayLinkForcedMaxRate) {
    _displayLinkForcedMaxRate = YES;
    [self setMaxRefreshRate:DisplayLinkManager.displayRefreshRate forceMax:YES];
  }
}

- (void)presentTexture:(FlutterTexture*)texture {
  @synchronized(self) {
    if (_front != nil) {
      [_availableTextures addObject:_front];
    }
    _front = texture;
    texture.presentedTime = CACurrentMediaTime();
    if ([NSThread isMainThread]) {
      [self presentOnMainThread:texture];
    } else {
      // Core animation layers can only be updated on main thread.
      dispatch_async(dispatch_get_main_queue(), ^{
        [self presentOnMainThread:texture];
      });
    }
  }
}

- (void)returnTexture:(FlutterTexture*)texture {
  @synchronized(self) {
    [_availableTextures addObject:texture];
  }
}

+ (BOOL)enabled {
  static BOOL enabled = YES;
  static BOOL didCheckInfoPlist = NO;
  if (!didCheckInfoPlist) {
    didCheckInfoPlist = YES;
    NSNumber* use_flutter_metal_layer =
        [[NSBundle mainBundle] objectForInfoDictionaryKey:@"FLTUseFlutterMetalLayer"];
    if (use_flutter_metal_layer != nil && ![use_flutter_metal_layer boolValue]) {
      enabled = NO;
    }
  }
  return enabled;
}

@end
