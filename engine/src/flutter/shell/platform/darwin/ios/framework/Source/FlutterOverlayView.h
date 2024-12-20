// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_FLUTTEROVERLAYVIEW_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_FLUTTEROVERLAYVIEW_H_

#include <Metal/Metal.h>
#include <UIKit/UIKit.h>

/// UIViews that are used by |FlutterPlatformViews| to present Flutter
/// rendering on top of system compositor rendering (ex. a web view).
///
/// When there is a view composited by the system compositor within a Flutter
/// view hierarchy, instead of rendering into a single render target, Flutter
/// renders into multiple render targets (depending on the number of
/// interleaving levels between Flutter & non-Flutter contents). While the
/// FlutterView contains the backing store for the root render target, the
/// FlutterOverlay view contains the backing stores for the rest. The overlay
/// views also handle touch propagation and the like for touches that occurs
/// either on overlays or otherwise may be intercepted by the platform views.
@interface FlutterOverlayView : UIView
- (instancetype)initWithFrame:(CGRect)frame NS_UNAVAILABLE;
- (instancetype)initWithCoder:(NSCoder*)aDecoder NS_UNAVAILABLE;

- (instancetype)init NS_DESIGNATED_INITIALIZER;
- (instancetype)initWithContentsScale:(CGFloat)contentsScale
                          pixelFormat:(MTLPixelFormat)pixelFormat;

@end

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_FLUTTEROVERLAYVIEW_H_
