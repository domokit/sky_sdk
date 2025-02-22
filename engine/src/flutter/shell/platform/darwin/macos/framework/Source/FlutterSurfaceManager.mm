// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterSurfaceManager.h"

#import <Metal/Metal.h>

#include <algorithm>

#include "flutter/fml/logging.h"
#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterSurface.h"

@implementation FlutterSurfacePresentInfo
@end

@interface FlutterSurfaceManager () {
  id<MTLDevice> _device;
  id<MTLCommandQueue> _commandQueue;
  CALayer* _containingLayer;
  __weak id<FlutterSurfaceManagerDelegate> _delegate;

  // Available (cached) back buffer surfaces. These will be cleared during
  // present and replaced by current frong surfaces.
  FlutterBackBufferCache* _backBufferCache;

  // Surfaces currently used to back visible layers.
  NSMutableArray<FlutterSurface*>* _frontSurfaces;

  // Currently visible layers.
  NSMutableArray<CALayer*>* _layers;

  // Whether to highlight borders of overlay surfaces. Determined by
  // FLTEnableSurfaceDebugInfo value in main bundle Info.plist.
  NSNumber* _enableSurfaceDebugInfo;
  CATextLayer* _infoLayer;

  CFTimeInterval _lastPresentationTime;
}

/**
 * Updates underlying CALayers with the contents of the surfaces to present.
 */
- (void)commit:(NSArray<FlutterSurfacePresentInfo*>*)surfaces;

@end

static NSColor* GetBorderColorForLayer(int layer) {
  NSArray* colors = @[
    [NSColor yellowColor],
    [NSColor cyanColor],
    [NSColor magentaColor],
    [NSColor greenColor],
    [NSColor purpleColor],
    [NSColor orangeColor],
    [NSColor blueColor],
  ];
  return colors[layer % colors.count];
}

/// Creates sublayers for given layer, each one displaying a portion of the
/// of the surface determined by a rectangle in the provided paint region.
static void UpdateContentSubLayers(CALayer* layer,
                                   IOSurfaceRef surface,
                                   CGFloat scale,
                                   CGSize surfaceSize,
                                   NSColor* borderColor,
                                   const std::vector<FlutterRect>& paintRegion) {
  // Adjust sublayer count to paintRegion count.
  while (layer.sublayers.count > paintRegion.size()) {
    [layer.sublayers.lastObject removeFromSuperlayer];
  }

  while (layer.sublayers.count < paintRegion.size()) {
    CALayer* newLayer = [CALayer layer];
    [layer addSublayer:newLayer];
  }

  for (size_t i = 0; i < paintRegion.size(); i++) {
    CALayer* subLayer = [layer.sublayers objectAtIndex:i];
    const auto& rect = paintRegion[i];
    subLayer.frame = CGRectMake(rect.left / scale, rect.top / scale,
                                (rect.right - rect.left) / scale, (rect.bottom - rect.top) / scale);

    double width = surfaceSize.width;
    double height = surfaceSize.height;

    subLayer.contentsRect =
        CGRectMake(rect.left / width, rect.top / height, (rect.right - rect.left) / width,
                   (rect.bottom - rect.top) / height);

    if (borderColor != nil) {
      // Visualize sublayer
      subLayer.borderColor = borderColor.CGColor;
      subLayer.borderWidth = 1.0;
    }

    subLayer.contents = (__bridge id)surface;
  }
}

@implementation FlutterSurfaceManager

- (instancetype)initWithDevice:(id<MTLDevice>)device
                  commandQueue:(id<MTLCommandQueue>)commandQueue
                         layer:(CALayer*)containingLayer
                      delegate:(__weak id<FlutterSurfaceManagerDelegate>)delegate {
  if (self = [super init]) {
    _device = device;
    _commandQueue = commandQueue;
    _containingLayer = containingLayer;
    _delegate = delegate;

    _backBufferCache = [[FlutterBackBufferCache alloc] init];
    _frontSurfaces = [NSMutableArray array];
    _layers = [NSMutableArray array];
  }
  return self;
}

- (FlutterBackBufferCache*)backBufferCache {
  return _backBufferCache;
}

- (NSArray*)frontSurfaces {
  return _frontSurfaces;
}

- (NSArray*)layers {
  return _layers;
}

- (FlutterSurface*)surfaceForSize:(CGSize)size {
  FlutterSurface* surface = [_backBufferCache removeSurfaceForSize:size];
  if (surface == nil) {
    surface = [[FlutterSurface alloc] initWithSize:size device:_device];
  }
  return surface;
}

- (BOOL)enableSurfaceDebugInfo {
  if (_enableSurfaceDebugInfo == nil) {
    _enableSurfaceDebugInfo =
        [[NSBundle mainBundle] objectForInfoDictionaryKey:@"FLTEnableSurfaceDebugInfo"];
    if (_enableSurfaceDebugInfo == nil) {
      _enableSurfaceDebugInfo = @NO;
    }
  }
  return [_enableSurfaceDebugInfo boolValue];
}

- (void)commit:(NSArray<FlutterSurfacePresentInfo*>*)surfaces {
  FML_DCHECK([NSThread isMainThread]);

  // Release all unused back buffer surfaces and replace them with front surfaces.
  [_backBufferCache returnSurfaces:_frontSurfaces];

  // Front surfaces will be replaced by currently presented surfaces.
  [_frontSurfaces removeAllObjects];
  for (FlutterSurfacePresentInfo* info in surfaces) {
    [_frontSurfaces addObject:info.surface];
  }

  // Add or remove layers to match the count of surfaces to present.
  while (_layers.count > _frontSurfaces.count) {
    [_layers.lastObject removeFromSuperlayer];
    [_layers removeLastObject];
  }
  while (_layers.count < _frontSurfaces.count) {
    CALayer* layer = [CALayer layer];
    [_containingLayer addSublayer:layer];
    [_layers addObject:layer];
  }

  bool enableSurfaceDebugInfo = self.enableSurfaceDebugInfo;

  // Update contents of surfaces.
  for (size_t i = 0; i < surfaces.count; ++i) {
    FlutterSurfacePresentInfo* info = surfaces[i];
    CALayer* layer = _layers[i];
    CGFloat scale = _containingLayer.contentsScale;
    if (i == 0) {
      layer.frame = CGRectMake(info.offset.x / scale, info.offset.y / scale,
                               info.surface.size.width / scale, info.surface.size.height / scale);
      layer.contents = (__bridge id)info.surface.ioSurface;
    } else {
      layer.frame = CGRectZero;
      NSColor* borderColor = enableSurfaceDebugInfo ? GetBorderColorForLayer(i - 1) : nil;
      UpdateContentSubLayers(layer, info.surface.ioSurface, scale, info.surface.size, borderColor,
                             info.paintRegion);
    }
    layer.zPosition = info.zIndex;
  }

  if (enableSurfaceDebugInfo) {
    if (_infoLayer == nil) {
      _infoLayer = [[CATextLayer alloc] init];
      [_containingLayer addSublayer:_infoLayer];
      _infoLayer.fontSize = 15;
      _infoLayer.foregroundColor = [NSColor yellowColor].CGColor;
      _infoLayer.frame = CGRectMake(15, 15, 300, 100);
      _infoLayer.contentsScale = _containingLayer.contentsScale;
      _infoLayer.zPosition = 100000;
    }
    _infoLayer.string = [NSString stringWithFormat:@"Surface count: %li", _layers.count];
  }
}

static CGSize GetRequiredFrameSize(NSArray<FlutterSurfacePresentInfo*>* surfaces) {
  CGSize size = CGSizeZero;
  for (FlutterSurfacePresentInfo* info in surfaces) {
    size = CGSizeMake(std::max(size.width, info.offset.x + info.surface.size.width),
                      std::max(size.height, info.offset.y + info.surface.size.height));
  }
  return size;
}

- (void)presentSurfaces:(NSArray<FlutterSurfacePresentInfo*>*)surfaces
                 atTime:(CFTimeInterval)presentationTime
                 notify:(dispatch_block_t)notify {
  id<MTLCommandBuffer> commandBuffer = [_commandQueue commandBuffer];
  [commandBuffer commit];
  [commandBuffer waitUntilScheduled];

  CGSize size = GetRequiredFrameSize(surfaces);

  CFTimeInterval delay = 0;

  if (presentationTime > 0) {
    // Enforce frame pacing. It seems that the target timestamp of CVDisplayLink does not
    // exactly correspond to core animation deadline. Especially with 120hz, setting the frame
    // contents too close after previous target timestamp will result in uneven frame pacing.
    // Empirically setting the content in the second half of frame interval seems to work
    // well for both 60hz and 120hz.
    //
    // This schedules a timer on current (raster) thread runloop. Raster thread at
    // this point should be idle (the next frame vsync has not been signalled yet).
    //
    // Alternative could be simply blocking the raster thread, but that would show
    // as a average_frame_rasterizer_time_millis regresson.
    CFTimeInterval minPresentationTime = (presentationTime + _lastPresentationTime) / 2.0;
    CFTimeInterval now = CACurrentMediaTime();
    delay = std::max(minPresentationTime - now, 0.0);
  }
  [_delegate onPresent:size
             withBlock:^{
               _lastPresentationTime = presentationTime;
               [CATransaction begin];
               [CATransaction setDisableActions:YES];
               [self commit:surfaces];
               if (notify != nil) {
                 notify();
               }
               [CATransaction commit];
             }
                 delay:delay];
}

@end

// Cached back buffers will be released after kIdleDelay if there is no activity.
static const double kIdleDelay = 1.0;
// Once surfaces reach kEvictionAge, they will be evicted from the cache.
// The age of 30 has been chosen to reduce potential surface allocation churn.
// For unused surface 30 frames means only half a second at 60fps, and there is
// idle timeout of 1 second where all surfaces are evicted.
static const int kSurfaceEvictionAge = 30;

@interface FlutterBackBufferCache () {
  NSMutableArray<FlutterSurface*>* _surfaces;
  NSMapTable<FlutterSurface*, NSNumber*>* _surfaceAge;
}

@end

@implementation FlutterBackBufferCache

- (instancetype)init {
  if (self = [super init]) {
    self->_surfaces = [[NSMutableArray alloc] init];
    self->_surfaceAge = [NSMapTable weakToStrongObjectsMapTable];
  }
  return self;
}

- (int)ageForSurface:(FlutterSurface*)surface {
  NSNumber* age = [_surfaceAge objectForKey:surface];
  return age != nil ? age.intValue : 0;
}

- (void)setAge:(int)age forSurface:(FlutterSurface*)surface {
  [_surfaceAge setObject:@(age) forKey:surface];
}

- (nullable FlutterSurface*)removeSurfaceForSize:(CGSize)size {
  @synchronized(self) {
    // Purge all cached surfaces if the size has changed.
    if (_surfaces.firstObject != nil && !CGSizeEqualToSize(_surfaces.firstObject.size, size)) {
      [_surfaces removeAllObjects];
    }

    FlutterSurface* res;

    // Returns youngest surface that is not in use. Returning youngest surface ensures
    // that the cache doesn't keep more surfaces than it needs to, as the unused surfaces
    // kept in cache will have their age kept increasing until purged (inside [returnSurfaces:]).
    for (FlutterSurface* surface in _surfaces) {
      if (!surface.isInUse &&
          (res == nil || [self ageForSurface:res] > [self ageForSurface:surface])) {
        res = surface;
      }
    }
    if (res != nil) {
      [_surfaces removeObject:res];
    }
    return res;
  }
}

- (void)returnSurfaces:(nonnull NSArray<FlutterSurface*>*)returnedSurfaces {
  @synchronized(self) {
    for (FlutterSurface* surface in returnedSurfaces) {
      [self setAge:0 forSurface:surface];
    }
    for (FlutterSurface* surface in _surfaces) {
      [self setAge:[self ageForSurface:surface] + 1 forSurface:surface];
    }

    [_surfaces addObjectsFromArray:returnedSurfaces];

    // Purge all surface with age = kSurfaceEvictionAge. Reaching this age can mean two things:
    // - Surface is still in use and we can't return it. This can happen in some edge
    //   cases where the compositor holds on to the surface for much longer than expected.
    // - Surface is not in use but it hasn't been requested from the cache for a while.
    //   This means there are too many surfaces in the cache.
    [_surfaces filterUsingPredicate:[NSPredicate predicateWithBlock:^BOOL(FlutterSurface* surface,
                                                                          NSDictionary* bindings) {
                 return [self ageForSurface:surface] < kSurfaceEvictionAge;
               }]];
  }

  // performSelector:withObject:afterDelay needs to be performed on RunLoop thread
  [self performSelectorOnMainThread:@selector(reschedule) withObject:nil waitUntilDone:NO];
}

- (NSUInteger)count {
  @synchronized(self) {
    return _surfaces.count;
  }
}

- (void)onIdle {
  @synchronized(self) {
    [_surfaces removeAllObjects];
  }
}

- (void)reschedule {
  [NSObject cancelPreviousPerformRequestsWithTarget:self selector:@selector(onIdle) object:nil];
  [self performSelector:@selector(onIdle) withObject:nil afterDelay:kIdleDelay];
}

- (void)dealloc {
  [NSObject cancelPreviousPerformRequestsWithTarget:self selector:@selector(onIdle) object:nil];
}

@end
