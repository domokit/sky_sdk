// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterView.h"

#include "flutter/fml/platform/darwin/cf_utils.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/SemanticsObject.h"

FLUTTER_ASSERT_ARC

@interface FlutterView ()
@property(nonatomic, weak) id<FlutterViewEngineDelegate> delegate;
@end

@implementation FlutterView {
  BOOL _isWideGamutEnabled;
}

- (instancetype)init {
  NSAssert(NO, @"FlutterView must initWithDelegate");
  return nil;
}

- (instancetype)initWithFrame:(CGRect)frame {
  NSAssert(NO, @"FlutterView must initWithDelegate");
  return nil;
}

- (instancetype)initWithCoder:(NSCoder*)aDecoder {
  NSAssert(NO, @"FlutterView must initWithDelegate");
  return nil;
}

- (UIScreen*)screen {
  if (@available(iOS 13.0, *)) {
    return self.window.windowScene.screen;
  }
  return UIScreen.mainScreen;
}

- (MTLPixelFormat)pixelFormat {
  if ([self.layer isKindOfClass:[CAMetalLayer class]]) {
// It is a known Apple bug that CAMetalLayer incorrectly reports its supported
// SDKs. It is, in fact, available since iOS 8.
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunguarded-availability-new"
    CAMetalLayer* layer = (CAMetalLayer*)self.layer;
    return layer.pixelFormat;
  }
  return MTLPixelFormatBGRA8Unorm;
}
- (BOOL)isWideGamutSupported {
  if (!self.delegate.isUsingImpeller) {
    return NO;
  }

  FML_DCHECK(self.screen);

  // This predicates the decision on the capabilities of the iOS device's
  // display.  This means external displays will not support wide gamut if the
  // device's display doesn't support it.  It practice that should be never.
  return self.screen.traitCollection.displayGamut != UIDisplayGamutSRGB;
}

- (instancetype)initWithDelegate:(id<FlutterViewEngineDelegate>)delegate
                          opaque:(BOOL)opaque
                 enableWideGamut:(BOOL)isWideGamutEnabled {
  if (delegate == nil) {
    NSLog(@"FlutterView delegate was nil.");
    return nil;
  }

  self = [super initWithFrame:CGRectNull];

  if (self) {
    _delegate = delegate;
    _isWideGamutEnabled = isWideGamutEnabled;
    self.layer.opaque = opaque;
  }

  return self;
}

static void PrintWideGamutWarningOnce() {
  static BOOL did_print = NO;
  if (did_print) {
    return;
  }
  FML_DLOG(WARNING) << "Rendering wide gamut colors is turned on but isn't "
                       "supported, downgrading the color gamut to sRGB.";
  did_print = YES;
}

- (void)layoutSubviews {
  if ([self.layer isKindOfClass:[CAMetalLayer class]]) {
// It is a known Apple bug that CAMetalLayer incorrectly reports its supported
// SDKs. It is, in fact, available since iOS 8.
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunguarded-availability-new"
    CAMetalLayer* layer = (CAMetalLayer*)self.layer;
#pragma clang diagnostic pop
    CGFloat screenScale = self.screen.scale;
    layer.allowsGroupOpacity = YES;
    layer.contentsScale = screenScale;
    layer.rasterizationScale = screenScale;
    layer.framebufferOnly = flutter::Settings::kSurfaceDataAccessible ? NO : YES;
    BOOL isWideGamutSupported = self.isWideGamutSupported;
    if (_isWideGamutEnabled && isWideGamutSupported) {
      fml::CFRef<CGColorSpaceRef> srgb(CGColorSpaceCreateWithName(kCGColorSpaceExtendedSRGB));
      layer.colorspace = srgb;
      layer.pixelFormat = MTLPixelFormatBGRA10_XR;
    } else if (_isWideGamutEnabled && !isWideGamutSupported) {
      PrintWideGamutWarningOnce();
    }
  }

  [super layoutSubviews];
}

static BOOL _forceSoftwareRendering;

+ (BOOL)forceSoftwareRendering {
  return _forceSoftwareRendering;
}

+ (void)setForceSoftwareRendering:(BOOL)forceSoftwareRendering {
  _forceSoftwareRendering = forceSoftwareRendering;
}

+ (Class)layerClass {
  return flutter::GetCoreAnimationLayerClassForRenderingAPI(
      flutter::GetRenderingAPIForProcess(FlutterView.forceSoftwareRendering));
}

- (void)drawLayer:(CALayer*)layer inContext:(CGContextRef)context {
  TRACE_EVENT0("flutter", "SnapshotFlutterView");

  if (layer != self.layer || context == nullptr) {
    return;
  }

  auto screenshot = [_delegate takeScreenshot:flutter::Rasterizer::ScreenshotType::UncompressedImage
                              asBase64Encoded:NO];

  if (!screenshot.data || screenshot.data->isEmpty() || screenshot.frame_size.isEmpty()) {
    return;
  }

  NSData* data = [NSData dataWithBytes:const_cast<void*>(screenshot.data->data())
                                length:screenshot.data->size()];

  fml::CFRef<CGDataProviderRef> image_data_provider(
      CGDataProviderCreateWithCFData(reinterpret_cast<CFDataRef>(data)));

  fml::CFRef<CGColorSpaceRef> colorspace(CGColorSpaceCreateDeviceRGB());

  // Defaults for RGBA8888.
  size_t bits_per_component = 8u;
  size_t bits_per_pixel = 32u;
  size_t bytes_per_row_multiplier = 4u;
  CGBitmapInfo bitmap_info =
      static_cast<CGBitmapInfo>(static_cast<uint32_t>(kCGImageAlphaPremultipliedLast) |
                                static_cast<uint32_t>(kCGBitmapByteOrder32Big));

  switch (screenshot.pixel_format) {
    case flutter::Rasterizer::ScreenshotFormat::kUnknown:
    case flutter::Rasterizer::ScreenshotFormat::kR8G8B8A8UNormInt:
      // Assume unknown is Skia and is RGBA8888. Keep defaults.
      break;
    case flutter::Rasterizer::ScreenshotFormat::kB8G8R8A8UNormInt:
      // Treat this as little endian with the alpha first so that it's read backwards.
      bitmap_info =
          static_cast<CGBitmapInfo>(static_cast<uint32_t>(kCGImageAlphaPremultipliedFirst) |
                                    static_cast<uint32_t>(kCGBitmapByteOrder32Little));
      break;
    case flutter::Rasterizer::ScreenshotFormat::kR16G16B16A16Float:
      bits_per_component = 16u;
      bits_per_pixel = 64u;
      bytes_per_row_multiplier = 8u;
      bitmap_info =
          static_cast<CGBitmapInfo>(static_cast<uint32_t>(kCGImageAlphaPremultipliedLast) |
                                    static_cast<uint32_t>(kCGBitmapFloatComponents) |
                                    static_cast<uint32_t>(kCGBitmapByteOrder16Little));
      break;
  }

  fml::CFRef<CGImageRef> image(CGImageCreate(
      screenshot.frame_size.width(),                             // size_t width
      screenshot.frame_size.height(),                            // size_t height
      bits_per_component,                                        // size_t bitsPerComponent
      bits_per_pixel,                                            // size_t bitsPerPixel,
      bytes_per_row_multiplier * screenshot.frame_size.width(),  // size_t bytesPerRow
      colorspace,                                                // CGColorSpaceRef space
      bitmap_info,                                               // CGBitmapInfo bitmapInfo
      image_data_provider,                                       // CGDataProviderRef provider
      nullptr,                                                   // const CGFloat* decode
      false,                                                     // bool shouldInterpolate
      kCGRenderingIntentDefault                                  // CGColorRenderingIntent intent
      ));

  const CGRect frame_rect =
      CGRectMake(0.0, 0.0, screenshot.frame_size.width(), screenshot.frame_size.height());
  CGContextSaveGState(context);
  // If the CGContext is not a bitmap based context, this returns zero.
  CGFloat height = CGBitmapContextGetHeight(context);
  if (height == 0) {
    height = CGFloat(screenshot.frame_size.height());
  }
  CGContextTranslateCTM(context, 0.0, height);
  CGContextScaleCTM(context, 1.0, -1.0);
  CGContextDrawImage(context, frame_rect, image);
  CGContextRestoreGState(context);
}

- (BOOL)isAccessibilityElement {
  // iOS does not provide an API to query whether the voice control
  // is turned on or off. It is likely at least one of the assitive
  // technologies is turned on if this method is called. If we do
  // not catch it in notification center, we will catch it here.
  //
  // TODO(chunhtai): Remove this workaround once iOS provides an
  // API to query whether voice control is enabled.
  // https://github.com/flutter/flutter/issues/76808.
  [self.delegate flutterViewAccessibilityDidCall];
  return NO;
}

// Enables keyboard-based navigation when the user turns on
// full keyboard access (FKA), using existing accessibility information.
//
// iOS does not provide any API for monitoring or querying whether FKA is on,
// but it does call isAccessibilityElement if FKA is on,
// so the isAccessibilityElement implementation above will be called
// when the view appears and the accessibility information will most likely
// be available by the time the user starts to interact with the app using FKA.
//
// See SemanticsObject+UIFocusSystem.mm for more details.
- (NSArray<id<UIFocusItem>>*)focusItemsInRect:(CGRect)rect {
  NSObject* rootAccessibilityElement =
      [self.accessibilityElements count] > 0 ? self.accessibilityElements[0] : nil;
  return [rootAccessibilityElement isKindOfClass:[SemanticsObjectContainer class]]
             ? @[ [rootAccessibilityElement accessibilityElementAtIndex:0] ]
             : nil;
}

- (NSArray<id<UIFocusEnvironment>>*)preferredFocusEnvironments {
  // Occasionally we add subviews to FlutterView (text fields for example).
  // These views shouldn't be directly visible to the iOS focus engine, instead
  // the focus engine should only interact with the designated focus items
  // (SemanticsObjects).
  return nil;
}

@end
