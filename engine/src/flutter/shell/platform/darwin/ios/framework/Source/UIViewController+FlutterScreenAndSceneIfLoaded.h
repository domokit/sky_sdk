// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_UIVIEWCONTROLLER_FLUTTERSCREENANDSCENEIFLOADED_H_
#define FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_UIVIEWCONTROLLER_FLUTTERSCREENANDSCENEIFLOADED_H_

#import <UIKit/UIKit.h>

@interface UIViewController (FlutterScreenAndSceneIfLoaded)

/// Returns a UIWindowScene if the UIViewController's view is loaded, and nil otherwise.
- (UIWindowScene*)flutterWindowSceneIfViewLoaded API_AVAILABLE(ios(13.0));

/// Before iOS 13, returns the main screen; After iOS 13, returns the screen the UIViewController is
/// attached to if its view is loaded, and nil otherwise.
- (UIScreen*)flutterScreenIfViewLoaded;

@end

#endif  // FLUTTER_SHELL_PLATFORM_DARWIN_IOS_FRAMEWORK_SOURCE_UIVIEWCONTROLLER_FLUTTERSCREENANDSCENEIFLOADED_H_
