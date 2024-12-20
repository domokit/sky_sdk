// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_TESTING_SCENARIO_APP_IOS_FLUTTERAPPEXTENSIONTESTHOST_FLUTTERAPPEXTENSIONTESTHOST_SCENEDELEGATE_H_
#define FLUTTER_TESTING_SCENARIO_APP_IOS_FLUTTERAPPEXTENSIONTESTHOST_FLUTTERAPPEXTENSIONTESTHOST_SCENEDELEGATE_H_

#import <UIKit/UIKit.h>

@interface SceneDelegate : UIResponder <UIWindowSceneDelegate>

@property(strong, nonatomic) UIWindow* window;

@end

#endif  // FLUTTER_TESTING_SCENARIO_APP_IOS_FLUTTERAPPEXTENSIONTESTHOST_FLUTTERAPPEXTENSIONTESTHOST_SCENEDELEGATE_H_
