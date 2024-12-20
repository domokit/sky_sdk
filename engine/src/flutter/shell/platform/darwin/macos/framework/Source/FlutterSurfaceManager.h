// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTERSURFACEMANAGER_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTERSURFACEMANAGER_H_

#import <Cocoa/Cocoa.h>
#import <QuartzCore/QuartzCore.h>

#include <vector>

#import "flutter/shell/platform/darwin/macos/framework/Source/FlutterSurface.h"

/**
 * Surface with additional properties needed for presenting.
 */
@interface FlutterSurfacePresentInfo : NSObject

@property(readwrite, strong, nonatomic, nonnull) FlutterSurface* surface;
@property(readwrite, nonatomic) CGPoint offset;
@property(readwrite, nonatomic) size_t zIndex;
@property(readwrite, nonatomic) std::vector<FlutterRect> paintRegion;

@end

@protocol FlutterSurfaceManagerDelegate <NSObject>

/*
 * Schedules the block on the platform thread and blocks until the block is executed.
 * Provided `frameSize` is used to unblock the platform thread if it waits for
 * a certain frame size during resizing.
 */
- (void)onPresent:(CGSize)frameSize withBlock:(nonnull dispatch_block_t)block;

@end

/**
 * FlutterSurfaceManager is responsible for providing and presenting Core Animation render
 * surfaces and managing sublayers.
 *
 * Owned by `FlutterView`.
 */
@interface FlutterSurfaceManager : NSObject

/**
 * Initializes and returns a surface manager that renders to a child layer (referred to as the
 * content layer) of the containing layer.
 */
- (nullable instancetype)initWithDevice:(nonnull id<MTLDevice>)device
                           commandQueue:(nonnull id<MTLCommandQueue>)commandQueue
                                  layer:(nonnull CALayer*)containingLayer
                               delegate:(nonnull id<FlutterSurfaceManagerDelegate>)delegate;

/**
 * Returns a back buffer surface of the given size to which Flutter can render content.
 * A cached surface will be returned if available; otherwise a new one will be created.
 *
 * Must be called on raster thread.
 */
- (nonnull FlutterSurface*)surfaceForSize:(CGSize)size;

/**
 * Sets the provided surfaces as contents of FlutterView. Will create, update and
 * remove sublayers as needed.
 *
 * Must be called on raster thread. This will schedule a commit on the platform thread and block the
 * raster thread until the commit is done. The `notify` block will be invoked on the platform thread
 * and can be used to perform additional work, such as mutating platform views. It is guaranteed be
 * called in the same CATransaction.
 */
- (void)presentSurfaces:(nonnull NSArray<FlutterSurfacePresentInfo*>*)surfaces
                 atTime:(CFTimeInterval)presentationTime
                 notify:(nullable dispatch_block_t)notify;

@end

/**
 * Cache of back buffers to prevent unnecessary IOSurface allocations.
 */
@interface FlutterBackBufferCache : NSObject

/**
 * Removes surface with given size from cache (if available) and returns it.
 */
- (nullable FlutterSurface*)removeSurfaceForSize:(CGSize)size;

/**
 * Removes all cached surfaces replacing them with new ones.
 */
- (void)returnSurfaces:(nonnull NSArray<FlutterSurface*>*)surfaces;

/**
 * Returns number of surfaces currently in cache. Used for tests.
 */
- (NSUInteger)count;

@end

/**
 * Interface to internal properties used for testing.
 */
@interface FlutterSurfaceManager (Private)

@property(readonly, nonatomic, nonnull) FlutterBackBufferCache* backBufferCache;
@property(readonly, nonatomic, nonnull) NSArray<FlutterSurface*>* frontSurfaces;
@property(readonly, nonatomic, nonnull) NSArray<CALayer*>* layers;

@end

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_MACOS_FRAMEWORK_SOURCE_FLUTTERSURFACEMANAGER_H_
