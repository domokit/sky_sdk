// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_FLUTTERINDIRECTSCRIBBLEDELEGATE_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_FLUTTERINDIRECTSCRIBBLEDELEGATE_H_

#import <Foundation/Foundation.h>

NS_ASSUME_NONNULL_BEGIN
@class FlutterTextInputPlugin;

@protocol FlutterIndirectScribbleDelegate <NSObject>
- (void)flutterTextInputPlugin:(FlutterTextInputPlugin*)textInputPlugin
                  focusElement:(UIScribbleElementIdentifier)elementIdentifier
                       atPoint:(CGPoint)referencePoint
                        result:(FlutterResult)callback;
- (void)flutterTextInputPlugin:(FlutterTextInputPlugin*)textInputPlugin
         requestElementsInRect:(CGRect)rect
                        result:(FlutterResult)callback;
@end
NS_ASSUME_NONNULL_END

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_FLUTTERINDIRECTSCRIBBLEDELEGATE_H_
