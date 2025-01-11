// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_FLUTTERPLATFORMPLUGIN_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_FLUTTERPLATFORMPLUGIN_H_

#import "flutter/shell/platform/darwin/common/framework/Headers/FlutterChannels.h"
#import "flutter/shell/platform/darwin/ios/framework/Headers/FlutterEngine.h"

@interface FlutterPlatformPlugin : NSObject
- (instancetype)init NS_UNAVAILABLE;
+ (instancetype)new NS_UNAVAILABLE;
- (instancetype)initWithEngine:(FlutterEngine*)engine NS_DESIGNATED_INITIALIZER;
- (void)handleMethodCall:(FlutterMethodCall*)call result:(FlutterResult)result;

- (void)showShareViewController:(NSString*)content;
- (void)searchWeb:(NSString*)searchTerm;
- (void)showLookUpViewController:(NSString*)term;
@end

namespace flutter {

extern const char* const kOrientationUpdateNotificationName;
extern const char* const kOrientationUpdateNotificationKey;
extern const char* const kOverlayStyleUpdateNotificationName;
extern const char* const kOverlayStyleUpdateNotificationKey;

}  // namespace flutter

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_FLUTTERPLATFORMPLUGIN_H_
