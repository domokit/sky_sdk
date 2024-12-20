// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_FLUTTERUNDOMANAGERPLUGIN_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_FLUTTERUNDOMANAGERPLUGIN_H_

#import <UIKit/UIKit.h>

#import "flutter/shell/platform/darwin/common/framework/Headers/FlutterChannels.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterUndoManagerDelegate.h"

@interface FlutterUndoManagerPlugin : NSObject

- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;

- (instancetype)initWithDelegate:(id<FlutterUndoManagerDelegate>)undoManagerDelegate
    NS_DESIGNATED_INITIALIZER;

- (void)handleMethodCall:(FlutterMethodCall*)call result:(FlutterResult)result;

@end

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_FLUTTERUNDOMANAGERPLUGIN_H_
