// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_HEADERS_FLUTTERPLATFORMVIEWS_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_HEADERS_FLUTTERPLATFORMVIEWS_H_

#import <UIKit/UIKit.h>

#import "FlutterCodecs.h"
#import "FlutterMacros.h"

NS_ASSUME_NONNULL_BEGIN

/**
 * Wraps a `UIView` for embedding in the Flutter hierarchy
 */
@protocol FlutterPlatformView <NSObject>
/**
 * Returns a reference to the `UIView` that is wrapped by this `FlutterPlatformView`.
 */
- (UIView*)view;
@end

FLUTTER_DARWIN_EXPORT
@protocol FlutterPlatformViewFactory <NSObject>
/**
 * Create a `FlutterPlatformView`.
 *
 * Implemented by iOS code that expose a `UIView` for embedding in a Flutter app.
 *
 * The implementation of this method should create a new `UIView` and return it.
 *
 * @param frame The rectangle for the newly created `UIView` measured in points.
 * @param viewId A unique identifier for this `UIView`.
 * @param args Parameters for creating the `UIView` sent from the Dart side of the Flutter app.
 *   If `createArgsCodec` is not implemented, or if no creation arguments were sent from the Dart
 *   code, this will be null. Otherwise this will be the value sent from the Dart code as decoded by
 *   `createArgsCodec`.
 */
- (NSObject<FlutterPlatformView>*)createWithFrame:(CGRect)frame
                                   viewIdentifier:(int64_t)viewId
                                        arguments:(id _Nullable)args;

/**
 * Returns the `FlutterMessageCodec` for decoding the args parameter of `createWithFrame`.
 *
 * Only needs to be implemented if `createWithFrame` needs an arguments parameter.
 */
@optional
- (NSObject<FlutterMessageCodec>*)createArgsCodec;
@end

NS_ASSUME_NONNULL_END

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_HEADERS_FLUTTERPLATFORMVIEWS_H_
