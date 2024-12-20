// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import <XCTest/XCTest.h>

#import "flutter/shell/platform/darwin/common/framework/Headers/FlutterMacros.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/UIViewController+FlutterScreenAndSceneIfLoaded.h"

FLUTTER_ASSERT_ARC

@interface UIViewController_FlutterViewAndSceneIfLoadedTest : XCTestCase
@end

@implementation UIViewController_FlutterViewAndSceneIfLoadedTest

- (void)testWindowSceneIfViewLoadedReturnsWindowSceneIfViewLoaded {
  if (@available(iOS 13.0, *)) {
    UIViewController* viewController = [[UIViewController alloc] initWithNibName:nil bundle:nil];

    NSSet<UIScene*>* scenes = UIApplication.sharedApplication.connectedScenes;
    XCTAssertEqual(scenes.count, 1UL, @"There must only be 1 scene for test");
    UIScene* scene = scenes.anyObject;
    XCTAssert([scene isKindOfClass:[UIWindowScene class]], @"Must be a window scene for test");
    UIWindowScene* windowScene = (UIWindowScene*)scene;
    XCTAssert(windowScene.windows.count > 0, @"There must be at least 1 window for test");
    UIWindow* window = windowScene.windows[0];
    [window addSubview:viewController.view];

    [viewController loadView];
    XCTAssertEqual(viewController.flutterWindowSceneIfViewLoaded, windowScene,
                   @"Must return the correct window scene when view loaded");
  }
}

- (void)testWindowSceneIfViewLoadedReturnsNilIfViewNotLoaded {
  if (@available(iOS 13.0, *)) {
    UIViewController* viewController = [[UIViewController alloc] initWithNibName:nil bundle:nil];
    XCTAssertNil(viewController.flutterWindowSceneIfViewLoaded,
                 @"Must return nil window scene when view not loaded");
  }
}

- (void)testScreenIfViewLoadedReturnsMainScreenBeforeIOS13 {
  if (@available(iOS 13.0, *)) {
    return;
  }

  UIViewController* viewController = [[UIViewController alloc] initWithNibName:nil bundle:nil];
  XCTAssertEqual(viewController.flutterScreenIfViewLoaded, UIScreen.mainScreen,
                 @"Must return UIScreen.mainScreen before iOS 13");
}

- (void)testScreenIfViewLoadedReturnsScreenIfViewLoadedAfterIOS13 {
  if (@available(iOS 13.0, *)) {
    UIViewController* viewController = [[UIViewController alloc] initWithNibName:nil bundle:nil];

    NSSet<UIScene*>* scenes = UIApplication.sharedApplication.connectedScenes;
    XCTAssertEqual(scenes.count, 1UL, @"There must only be 1 scene for test");
    UIScene* scene = scenes.anyObject;
    XCTAssert([scene isKindOfClass:[UIWindowScene class]], @"Must be a window scene for test");
    UIWindowScene* windowScene = (UIWindowScene*)scene;
    XCTAssert(windowScene.windows.count > 0, @"There must be at least 1 window for test");
    UIWindow* window = windowScene.windows[0];
    [window addSubview:viewController.view];

    [viewController loadView];
    XCTAssertEqual(viewController.flutterScreenIfViewLoaded, windowScene.screen,
                   @"Must return the correct screen when view loaded");
  }
}

- (void)testScreenIfViewLoadedReturnsNilIfViewNotLoadedAfterIOS13 {
  if (@available(iOS 13.0, *)) {
    UIViewController* viewController = [[UIViewController alloc] initWithNibName:nil bundle:nil];
    XCTAssertNil(viewController.flutterScreenIfViewLoaded,
                 @"Must return nil screen when view not loaded");
  }
}

@end
