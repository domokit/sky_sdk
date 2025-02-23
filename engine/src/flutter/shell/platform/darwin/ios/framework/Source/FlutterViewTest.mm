// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import <XCTest/XCTest.h>

#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterView.h"

FLUTTER_ASSERT_ARC

@interface FakeDelegate : NSObject <FlutterViewEngineDelegate>
@property(nonatomic) BOOL callbackCalled;
@end

@implementation FakeDelegate

@synthesize platformViewsController = _platformViewsController;

- (instancetype)init {
  _callbackCalled = NO;
  return self;
}

- (flutter::Rasterizer::Screenshot)takeScreenshot:(flutter::Rasterizer::ScreenshotType)type
                                  asBase64Encoded:(BOOL)base64Encode {
  return {};
}

- (void)flutterViewAccessibilityDidCall {
  _callbackCalled = YES;
}

@end

@interface FlutterViewTest : XCTestCase
@end

@implementation FlutterViewTest

- (void)testFlutterViewEnableSemanticsWhenIsAccessibilityElementIsCalled {
  FakeDelegate* delegate = [[FakeDelegate alloc] init];
  FlutterView* view = [[FlutterView alloc] initWithDelegate:delegate opaque:NO enableWideGamut:NO];
  delegate.callbackCalled = NO;
  XCTAssertFalse(view.isAccessibilityElement);
  XCTAssertTrue(delegate.callbackCalled);
}

- (void)testFlutterViewBackgroundColorIsNil {
  FakeDelegate* delegate = [[FakeDelegate alloc] init];
  FlutterView* view = [[FlutterView alloc] initWithDelegate:delegate opaque:NO enableWideGamut:NO];
  XCTAssertNil(view.backgroundColor);
}

- (void)testLayerScalesMatchScreenAfterLayoutSubviews {
  FakeDelegate* delegate = [[FakeDelegate alloc] init];
  FlutterView* view = [[FlutterView alloc] initWithDelegate:delegate opaque:NO enableWideGamut:NO];
  view.layer.contentsScale = CGFloat(-99.0);
  view.layer.rasterizationScale = CGFloat(-99.0);
  UIScreen* screen = [view screen];
  XCTAssertNotEqual(view.layer.contentsScale, screen.scale);
  XCTAssertNotEqual(view.layer.rasterizationScale, screen.scale);
  [view layoutSubviews];
  XCTAssertEqual(view.layer.contentsScale, screen.scale);
  XCTAssertEqual(view.layer.rasterizationScale, screen.scale);
}

@end
