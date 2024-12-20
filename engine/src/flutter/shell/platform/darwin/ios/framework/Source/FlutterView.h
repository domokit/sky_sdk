// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_FLUTTERVIEW_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_FLUTTERVIEW_H_

#import <Metal/Metal.h>
#import <UIKit/UIKit.h>

#include "flutter/shell/common/rasterizer.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterPlatformViews_Internal.h"

@protocol FlutterViewEngineDelegate <NSObject>

@property(nonatomic, readonly) BOOL isUsingImpeller;
@property(nonatomic, readonly) FlutterPlatformViewsController* platformViewsController;

- (flutter::Rasterizer::Screenshot)takeScreenshot:(flutter::Rasterizer::ScreenshotType)type
                                  asBase64Encoded:(BOOL)base64Encode;

/**
 * A callback that is called when iOS queries accessibility information of the Flutter view.
 *
 * This is useful to predict the current iOS accessibility status. For example, there is
 * no API to listen whether voice control is turned on or off. The Flutter engine uses
 * this callback to enable semantics in order to catch the case that voice control is
 * on.
 */
- (void)flutterViewAccessibilityDidCall;
@end

@interface FlutterView : UIView

- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
- (instancetype)initWithFrame:(CGRect)frame NS_UNAVAILABLE;
- (instancetype)initWithCoder:(NSCoder*)aDecoder NS_UNAVAILABLE;

- (instancetype)initWithDelegate:(id<FlutterViewEngineDelegate>)delegate
                          opaque:(BOOL)opaque
                 enableWideGamut:(BOOL)isWideGamutEnabled NS_DESIGNATED_INITIALIZER;

- (UIScreen*)screen;
- (MTLPixelFormat)pixelFormat;

// Set by FlutterEngine or FlutterViewController to override software rendering.
@property(class, nonatomic) BOOL forceSoftwareRendering;
@end

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_FLUTTERVIEW_H_
