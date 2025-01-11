// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import "GoldenPlatformViewTests.h"

static const NSInteger kSecondsToWaitForPlatformView = 30;

@interface PlatformViewUITests : GoldenPlatformViewTests

@end

@implementation PlatformViewUITests

- (instancetype)initWithInvocation:(NSInvocation*)invocation {
  GoldenTestManager* manager = [[GoldenTestManager alloc] initWithLaunchArg:@"--platform-view"];
  return [super initWithManager:manager invocation:invocation];
}

- (void)testPlatformView {
  [self checkPlatformViewGolden];
}

@end

@interface NonFullScreenFlutterViewPlatformViewUITests : GoldenPlatformViewTests

@end

@implementation NonFullScreenFlutterViewPlatformViewUITests

- (instancetype)initWithInvocation:(NSInvocation*)invocation {
  GoldenTestManager* manager =
      [[GoldenTestManager alloc] initWithLaunchArg:@"--non-full-screen-flutter-view-platform-view"];
  return [super initWithManager:manager invocation:invocation];
}

- (void)testPlatformView {
  [self checkPlatformViewGolden];
}

@end

@interface MultiplePlatformViewsTest : GoldenPlatformViewTests

@end

@implementation MultiplePlatformViewsTest

- (instancetype)initWithInvocation:(NSInvocation*)invocation {
  GoldenTestManager* manager =
      [[GoldenTestManager alloc] initWithLaunchArg:@"--platform-view-multiple"];
  return [super initWithManager:manager invocation:invocation];
}

- (void)testPlatformView {
  [self checkPlatformViewGolden];
}

@end

@interface MultiplePlatformViewsBackgroundForegroundTest : GoldenPlatformViewTests

@end

@implementation MultiplePlatformViewsBackgroundForegroundTest

- (instancetype)initWithInvocation:(NSInvocation*)invocation {
  GoldenTestManager* manager = [[GoldenTestManager alloc]
      initWithLaunchArg:@"--platform-view-multiple-background-foreground"];
  return [super initWithManager:manager invocation:invocation];
}

- (void)testPlatformView {
  [[XCUIDevice sharedDevice] pressButton:XCUIDeviceButtonHome];
  [self.application activate];
  [self checkPlatformViewGolden];
}

@end

// Clip Rect Tests
@interface PlatformViewMutationClipRectTests : GoldenPlatformViewTests

@end

@implementation PlatformViewMutationClipRectTests

- (instancetype)initWithInvocation:(NSInvocation*)invocation {
  GoldenTestManager* manager =
      [[GoldenTestManager alloc] initWithLaunchArg:@"--platform-view-cliprect"];
  return [super initWithManager:manager invocation:invocation];
}

- (void)testPlatformView {
  [self checkPlatformViewGolden];
}

@end

// Clip Rect Tests
@interface PlatformViewMutationClipRectWithMultiupleClipsTests : GoldenPlatformViewTests

@end

@implementation PlatformViewMutationClipRectWithMultiupleClipsTests

- (instancetype)initWithInvocation:(NSInvocation*)invocation {
  GoldenTestManager* manager =
      [[GoldenTestManager alloc] initWithLaunchArg:@"--platform-view-cliprect-multiple-clips"];
  return [super initWithManager:manager invocation:invocation];
}

- (void)testPlatformView {
  [self checkPlatformViewGolden];
}

@end

@interface PlatformViewMutationClipRectAfterMovedTests : GoldenPlatformViewTests

@end

@implementation PlatformViewMutationClipRectAfterMovedTests

- (instancetype)initWithInvocation:(NSInvocation*)invocation {
  GoldenTestManager* manager =
      [[GoldenTestManager alloc] initWithLaunchArg:@"--platform-view-cliprect-after-moved"];
  return [super initWithManager:manager invocation:invocation];
}

- (void)testPlatformView {
  // This test needs to wait for several frames for the PlatformView to settle to
  // the correct position. The PlatformView accessiblity is set to platform_view[10000] when it is
  // ready.
  XCUIElement* element = self.application.otherElements[@"platform_view[10000]"];
  BOOL exists = [element waitForExistenceWithTimeout:kSecondsToWaitForPlatformView];
  if (!exists) {
    XCTFail(@"It took longer than %@ second to find the platform view."
            @"There might be issues with the platform view's construction,"
            @"or with how the scenario is built.",
            @(kSecondsToWaitForPlatformView));
  }

  [self checkPlatformViewGolden];
}

@end

@interface PlatformViewMutationClipRectAfterMovedMultipleClipsTests : GoldenPlatformViewTests

@end

@implementation PlatformViewMutationClipRectAfterMovedMultipleClipsTests

- (instancetype)initWithInvocation:(NSInvocation*)invocation {
  GoldenTestManager* manager = [[GoldenTestManager alloc]
      initWithLaunchArg:@"--platform-view-cliprect-after-moved-multiple-clips"];
  return [super initWithManager:manager invocation:invocation];
}

- (void)testPlatformView {
  // This test needs to wait for several frames for the PlatformView to settle to
  // the correct position. The PlatformView accessiblity is set to platform_view[10000] when it is
  // ready.
  XCUIElement* element = self.application.otherElements[@"platform_view[10000]"];
  BOOL exists = [element waitForExistenceWithTimeout:kSecondsToWaitForPlatformView];
  if (!exists) {
    XCTFail(@"It took longer than %@ second to find the platform view."
            @"There might be issues with the platform view's construction,"
            @"or with how the scenario is built.",
            @(kSecondsToWaitForPlatformView));
  }

  [self checkPlatformViewGolden];
}

@end

@interface PlatformViewMutationClipRRectTests : GoldenPlatformViewTests

@end

@implementation PlatformViewMutationClipRRectTests

- (instancetype)initWithInvocation:(NSInvocation*)invocation {
  GoldenTestManager* manager =
      [[GoldenTestManager alloc] initWithLaunchArg:@"--platform-view-cliprrect"];
  return [super initWithManager:manager invocation:invocation];
}

- (void)testPlatformView {
  [self checkPlatformViewGolden];
}

@end

@interface PlatformViewMutationClipRRectMultipleClipsTests : GoldenPlatformViewTests

@end

@implementation PlatformViewMutationClipRRectMultipleClipsTests

- (instancetype)initWithInvocation:(NSInvocation*)invocation {
  GoldenTestManager* manager =
      [[GoldenTestManager alloc] initWithLaunchArg:@"--platform-view-cliprrect-multiple-clips"];
  return [super initWithManager:manager invocation:invocation];
}

- (void)testPlatformView {
  [self checkPlatformViewGolden];
}

@end

@interface PlatformViewMutationLargeClipRRectTests : GoldenPlatformViewTests

@end

@implementation PlatformViewMutationLargeClipRRectTests

- (instancetype)initWithInvocation:(NSInvocation*)invocation {
  GoldenTestManager* manager =
      [[GoldenTestManager alloc] initWithLaunchArg:@"--platform-view-large-cliprrect"];
  return [super initWithManager:manager invocation:invocation];
}

- (void)testPlatformView {
  [self checkPlatformViewGolden];
}

@end

@interface PlatformViewMutationLargeClipRRectMultipleClipsTests : GoldenPlatformViewTests

@end

@implementation PlatformViewMutationLargeClipRRectMultipleClipsTests

- (instancetype)initWithInvocation:(NSInvocation*)invocation {
  GoldenTestManager* manager = [[GoldenTestManager alloc]
      initWithLaunchArg:@"--platform-view-large-cliprrect-multiple-clips"];
  return [super initWithManager:manager invocation:invocation];
}

- (void)testPlatformView {
  [self checkPlatformViewGolden];
}

@end

@interface PlatformViewMutationClipPathTests : GoldenPlatformViewTests

@end

@implementation PlatformViewMutationClipPathTests

- (instancetype)initWithInvocation:(NSInvocation*)invocation {
  GoldenTestManager* manager =
      [[GoldenTestManager alloc] initWithLaunchArg:@"--platform-view-clippath"];
  return [super initWithManager:manager invocation:invocation];
}

- (void)testPlatformView {
  [self checkPlatformViewGolden];
}

@end

@interface PlatformViewMutationClipPathMultipleClipsTests : GoldenPlatformViewTests

@end

@implementation PlatformViewMutationClipPathMultipleClipsTests

- (instancetype)initWithInvocation:(NSInvocation*)invocation {
  GoldenTestManager* manager =
      [[GoldenTestManager alloc] initWithLaunchArg:@"--platform-view-clippath-multiple-clips"];
  return [super initWithManager:manager invocation:invocation];
}

- (void)testPlatformView {
  [self checkPlatformViewGolden];
}

@end

@interface PlatformViewMutationClipRectWithTransformTests : GoldenPlatformViewTests

@end

@implementation PlatformViewMutationClipRectWithTransformTests

- (instancetype)initWithInvocation:(NSInvocation*)invocation {
  GoldenTestManager* manager =
      [[GoldenTestManager alloc] initWithLaunchArg:@"--platform-view-cliprect-with-transform"];
  return [super initWithManager:manager invocation:invocation];
}

- (void)testPlatformView {
  [self checkPlatformViewGolden];
}

@end

@interface PlatformViewMutationClipRectWithTransformMultipleClipsTests : GoldenPlatformViewTests

@end

@implementation PlatformViewMutationClipRectWithTransformMultipleClipsTests

- (instancetype)initWithInvocation:(NSInvocation*)invocation {
  GoldenTestManager* manager = [[GoldenTestManager alloc]
      initWithLaunchArg:@"--platform-view-cliprect-with-transform-multiple-clips"];
  return [super initWithManager:manager invocation:invocation];
}

- (void)testPlatformView {
  [self checkPlatformViewGolden];
}

@end

@interface PlatformViewMutationClipRRectWithTransformTests : GoldenPlatformViewTests

@end

@implementation PlatformViewMutationClipRRectWithTransformTests

- (instancetype)initWithInvocation:(NSInvocation*)invocation {
  GoldenTestManager* manager =
      [[GoldenTestManager alloc] initWithLaunchArg:@"--platform-view-cliprrect-with-transform"];
  return [super initWithManager:manager invocation:invocation];
}

- (void)testPlatformView {
  [self checkPlatformViewGolden];
}

@end

@interface PlatformViewMutationClipRRectWithTransformMultipleClipsTests : GoldenPlatformViewTests

@end

@implementation PlatformViewMutationClipRRectWithTransformMultipleClipsTests

- (instancetype)initWithInvocation:(NSInvocation*)invocation {
  GoldenTestManager* manager = [[GoldenTestManager alloc]
      initWithLaunchArg:@"--platform-view-cliprrect-with-transform-multiple-clips"];
  return [super initWithManager:manager invocation:invocation];
}

- (void)testPlatformView {
  [self checkPlatformViewGolden];
}

@end

@interface PlatformViewMutationLargeClipRRectWithTransformTests : GoldenPlatformViewTests

@end

@implementation PlatformViewMutationLargeClipRRectWithTransformTests

- (instancetype)initWithInvocation:(NSInvocation*)invocation {
  GoldenTestManager* manager = [[GoldenTestManager alloc]
      initWithLaunchArg:@"--platform-view-large-cliprrect-with-transform"];
  return [super initWithManager:manager invocation:invocation];
}

- (void)testPlatformView {
  [self checkPlatformViewGolden];
}

@end

@interface PlatformViewMutationLargeClipRRectWithTransformMultipleClipsTests
    : GoldenPlatformViewTests

@end

@implementation PlatformViewMutationLargeClipRRectWithTransformMultipleClipsTests

- (instancetype)initWithInvocation:(NSInvocation*)invocation {
  GoldenTestManager* manager = [[GoldenTestManager alloc]
      initWithLaunchArg:@"--platform-view-large-cliprrect-with-transform-multiple-clips"];
  return [super initWithManager:manager invocation:invocation];
}

- (void)testPlatformView {
  [self checkPlatformViewGolden];
}

@end

@interface PlatformViewMutationClipPathWithTransformTests : GoldenPlatformViewTests

@end

@implementation PlatformViewMutationClipPathWithTransformTests

- (instancetype)initWithInvocation:(NSInvocation*)invocation {
  GoldenTestManager* manager =
      [[GoldenTestManager alloc] initWithLaunchArg:@"--platform-view-clippath-with-transform"];
  return [super initWithManager:manager invocation:invocation];
}

- (void)testPlatformView {
  [self checkPlatformViewGolden];
}

@end

@interface PlatformViewMutationClipPathWithTransformMultipleClipsTests : GoldenPlatformViewTests

@end

@implementation PlatformViewMutationClipPathWithTransformMultipleClipsTests

- (instancetype)initWithInvocation:(NSInvocation*)invocation {
  GoldenTestManager* manager = [[GoldenTestManager alloc]
      initWithLaunchArg:@"--platform-view-clippath-with-transform-multiple-clips"];
  return [super initWithManager:manager invocation:invocation];
}

- (void)testPlatformView {
  [self checkPlatformViewGolden];
}

@end

@interface TwoPlatformViewClipRectTests : GoldenPlatformViewTests

@end

@implementation TwoPlatformViewClipRectTests

- (instancetype)initWithInvocation:(NSInvocation*)invocation {
  GoldenTestManager* manager =
      [[GoldenTestManager alloc] initWithLaunchArg:@"--two-platform-view-clip-rect"];
  return [super initWithManager:manager invocation:invocation];
}

- (void)testPlatformView {
  [self checkPlatformViewGolden];
}

@end

@interface TwoPlatformViewClipRectMultipleClipsTests : GoldenPlatformViewTests

@end

@implementation TwoPlatformViewClipRectMultipleClipsTests

- (instancetype)initWithInvocation:(NSInvocation*)invocation {
  GoldenTestManager* manager =
      [[GoldenTestManager alloc] initWithLaunchArg:@"--two-platform-view-clip-rect-multiple-clips"];
  return [super initWithManager:manager invocation:invocation];
}

- (void)testPlatformView {
  [self checkPlatformViewGolden];
}

@end

@interface TwoPlatformViewClipRRectTests : GoldenPlatformViewTests

@end

@implementation TwoPlatformViewClipRRectTests

- (instancetype)initWithInvocation:(NSInvocation*)invocation {
  GoldenTestManager* manager =
      [[GoldenTestManager alloc] initWithLaunchArg:@"--two-platform-view-clip-rrect"];
  return [super initWithManager:manager invocation:invocation];
}

- (void)testPlatformView {
  [self checkPlatformViewGolden];
}

@end

@interface TwoPlatformViewClipRRectMultipleClipsTests : GoldenPlatformViewTests

@end

@implementation TwoPlatformViewClipRRectMultipleClipsTests

- (instancetype)initWithInvocation:(NSInvocation*)invocation {
  GoldenTestManager* manager = [[GoldenTestManager alloc]
      initWithLaunchArg:@"--two-platform-view-clip-rrect-multiple-clips"];
  return [super initWithManager:manager invocation:invocation];
}

- (void)testPlatformView {
  [self checkPlatformViewGolden];
}

@end

@interface TwoPlatformViewClipPathTests : GoldenPlatformViewTests

@end

@implementation TwoPlatformViewClipPathTests

- (instancetype)initWithInvocation:(NSInvocation*)invocation {
  GoldenTestManager* manager =
      [[GoldenTestManager alloc] initWithLaunchArg:@"--two-platform-view-clip-path"];
  return [super initWithManager:manager invocation:invocation];
}

- (void)testPlatformView {
  [self checkPlatformViewGolden];
}

@end

@interface TwoPlatformViewClipPathMultipleClipsTests : GoldenPlatformViewTests

@end

@implementation TwoPlatformViewClipPathMultipleClipsTests

- (instancetype)initWithInvocation:(NSInvocation*)invocation {
  GoldenTestManager* manager =
      [[GoldenTestManager alloc] initWithLaunchArg:@"--two-platform-view-clip-path-multiple-clips"];
  return [super initWithManager:manager invocation:invocation];
}

- (void)testPlatformView {
  [self checkPlatformViewGolden];
}

@end

@interface PlatformViewMutationTransformTests : GoldenPlatformViewTests

@end

@implementation PlatformViewMutationTransformTests

- (instancetype)initWithInvocation:(NSInvocation*)invocation {
  GoldenTestManager* manager =
      [[GoldenTestManager alloc] initWithLaunchArg:@"--platform-view-transform"];
  return [super initWithManager:manager invocation:invocation];
}

- (void)testPlatformView {
  [self checkPlatformViewGolden];
}

@end

@interface PlatformViewMutationOpacityTests : GoldenPlatformViewTests

@end

@implementation PlatformViewMutationOpacityTests

- (instancetype)initWithInvocation:(NSInvocation*)invocation {
  GoldenTestManager* manager =
      [[GoldenTestManager alloc] initWithLaunchArg:@"--platform-view-opacity"];
  return [super initWithManager:manager invocation:invocation];
}

- (void)testPlatformView {
  [self checkPlatformViewGolden];
}

@end

@interface PlatformViewWithOtherBackdropFilterTests : GoldenPlatformViewTests

@end

@implementation PlatformViewWithOtherBackdropFilterTests

- (instancetype)initWithInvocation:(NSInvocation*)invocation {
  GoldenTestManager* manager =
      [[GoldenTestManager alloc] initWithLaunchArg:@"--platform-view-with-other-backdrop-filter"];
  return [super initWithManager:manager invocation:invocation];
}

- (void)testPlatformView {
  [self checkPlatformViewGolden];
}

@end

@interface TwoPlatformViewsWithOtherBackDropFilterTests : GoldenPlatformViewTests

@end

@implementation TwoPlatformViewsWithOtherBackDropFilterTests

- (instancetype)initWithInvocation:(NSInvocation*)invocation {
  GoldenTestManager* manager = [[GoldenTestManager alloc]
      initWithLaunchArg:@"--two-platform-views-with-other-backdrop-filter"];
  return [super initWithManager:manager invocation:invocation];
}

- (void)testPlatformView {
  // (TODO)cyanglaz: remove the threshold adjustment after all the ci migrates to macOS13.
  // https://github.com/flutter/flutter/issues/133207
  if ([NSProcessInfo processInfo].operatingSystemVersion.majorVersion >= 13) {
    self.rmseThreadhold = 0.7;
  }
  [self checkPlatformViewGolden];
}

@end

@interface PlatformViewWithNegativeOtherBackDropFilterTests : GoldenPlatformViewTests

@end

@implementation PlatformViewWithNegativeOtherBackDropFilterTests

- (instancetype)initWithInvocation:(NSInvocation*)invocation {
  GoldenTestManager* manager = [[GoldenTestManager alloc]
      initWithLaunchArg:@"--platform-view-with-negative-backdrop-filter"];
  return [super initWithManager:manager invocation:invocation];
}

- (void)testPlatformView {
  [self checkPlatformViewGolden];
}

@end

@interface PlatformViewRotation : GoldenPlatformViewTests
@end

@implementation PlatformViewRotation
- (instancetype)initWithInvocation:(NSInvocation*)invocation {
  GoldenTestManager* manager =
      [[GoldenTestManager alloc] initWithLaunchArg:@"--platform-view-rotate"];
  return [super initWithManager:manager invocation:invocation];
}

- (void)tearDown {
  XCUIDevice.sharedDevice.orientation = UIDeviceOrientationPortrait;
  [super tearDown];
}

- (void)testPlatformView {
  XCUIDevice.sharedDevice.orientation = UIDeviceOrientationLandscapeLeft;
  [self checkPlatformViewGolden];
}

@end

@interface PlatformViewWithContinuousTexture : XCTestCase

@end

@implementation PlatformViewWithContinuousTexture

- (void)setUp {
  self.continueAfterFailure = NO;
}

- (void)testPlatformViewWithContinuousTexture {
  XCUIApplication* app = [[XCUIApplication alloc] init];
  app.launchArguments =
      @[ @"--platform-view-with-continuous-texture", @"--with-continuous-texture" ];
  [app launch];

  XCUIElement* platformView = app.textViews.firstMatch;
  BOOL exists = [platformView waitForExistenceWithTimeout:kSecondsToWaitForPlatformView];
  if (!exists) {
    XCTFail(@"It took longer than %@ second to find the platform view."
            @"There might be issues with the platform view's construction,"
            @"or with how the scenario is built.",
            @(kSecondsToWaitForPlatformView));
  }

  XCTAssertNotNil(platformView);
}

@end

@interface PlatformViewScrollingUnderWidget : XCTestCase

@end

@implementation PlatformViewScrollingUnderWidget

- (void)setUp {
  [super setUp];
  self.continueAfterFailure = NO;
}

- (void)testPlatformViewScrollingUnderWidget {
  XCUIApplication* app = [[XCUIApplication alloc] init];
  app.launchArguments =
      @[ @"--platform-view-scrolling-under-widget", @"--with-continuous-texture" ];
  [app launch];

  XCUIElement* platformView = app.textViews.firstMatch;
  BOOL exists = [platformView waitForExistenceWithTimeout:kSecondsToWaitForPlatformView];
  if (!exists) {
    XCTFail(@"It took longer than %@ second to find the platform view."
            @"There might be issues with the platform view's construction,"
            @"or with how the scenario is built.",
            @(kSecondsToWaitForPlatformView));
  }

  // Wait and let the scenario app scroll a bit.
  XCTWaiterResult waitResult = [XCTWaiter
      waitForExpectations:@[ [[XCTestExpectation alloc] initWithDescription:@"Wait for 5 seconds"] ]
                  timeout:5];
  // If the waiter is not interrupted, we know the app is in a valid state after timeout, thus the
  // test passes.
  XCTAssert(waitResult != XCTWaiterResultInterrupted);
}

@end

@interface PlatformViewWithClipsScrolling : XCTestCase

@end

@implementation PlatformViewWithClipsScrolling

- (void)setUp {
  [super setUp];
  self.continueAfterFailure = NO;
}

- (void)testPlatformViewsWithClipsScrolling {
  XCUIApplication* app = [[XCUIApplication alloc] init];
  app.launchArguments =
      @[ @"--platform-views-with-clips-scrolling", @"platform_views_with_clips_scrolling" ];
  [app launch];

  XCUIElement* platformView = app.textViews.firstMatch;
  BOOL exists = [platformView waitForExistenceWithTimeout:kSecondsToWaitForPlatformView];
  if (!exists) {
    XCTFail(@"It took longer than %@ second to find the platform view."
            @"There might be issues with the platform view's construction,"
            @"or with how the scenario is built.",
            @(kSecondsToWaitForPlatformView));
  }

  // Wait and let the scenario app scroll a bit.
  XCTWaiterResult waitResult = [XCTWaiter
      waitForExpectations:@[ [[XCTestExpectation alloc] initWithDescription:@"Wait for 5 seconds"] ]
                  timeout:5];
  // If the waiter is not interrupted, we know the app is in a valid state after timeout, thus the
  // test passes.
  XCTAssert(waitResult != XCTWaiterResultInterrupted);
}

@end

@interface PlatformViewWithClipsScrollingMultipleClips : XCTestCase

@end

@implementation PlatformViewWithClipsScrollingMultipleClips

- (void)setUp {
  [super setUp];
  self.continueAfterFailure = NO;
}

- (void)testPlatformViewsWithClipsScrolling {
  XCUIApplication* app = [[XCUIApplication alloc] init];
  app.launchArguments = @[
    @"--platform-views-with-clips-scrolling", @"platform_views_with_clips_scrolling-multiple-clips"
  ];
  [app launch];

  XCUIElement* platformView = app.textViews.firstMatch;
  BOOL exists = [platformView waitForExistenceWithTimeout:kSecondsToWaitForPlatformView];
  if (!exists) {
    XCTFail(@"It took longer than %@ second to find the platform view."
            @"There might be issues with the platform view's construction,"
            @"or with how the scenario is built.",
            @(kSecondsToWaitForPlatformView));
  }

  // Wait and let the scenario app scroll a bit.
  XCTWaiterResult waitResult = [XCTWaiter
      waitForExpectations:@[ [[XCTestExpectation alloc] initWithDescription:@"Wait for 5 seconds"] ]
                  timeout:5];
  // If the waiter is not interrupted, we know the app is in a valid state after timeout, thus the
  // test passes.
  XCTAssert(waitResult != XCTWaiterResultInterrupted);
}

@end
