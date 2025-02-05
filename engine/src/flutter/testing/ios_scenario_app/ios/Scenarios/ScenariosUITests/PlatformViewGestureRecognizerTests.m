// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import <XCTest/XCTest.h>

static const NSInteger kSecondsToWaitForPlatformView = 30;

@interface PlatformViewGestureRecognizerTests : XCTestCase

@end

@implementation PlatformViewGestureRecognizerTests

- (void)setUp {
  self.continueAfterFailure = NO;
}

- (void)testRejectPolicyUtilTouchesEnded {
  XCUIApplication* app = [[XCUIApplication alloc] init];
  app.launchArguments = @[ @"--gesture-reject-after-touches-ended" ];
  [app launch];

  NSPredicate* predicateToFindPlatformView =
      [NSPredicate predicateWithBlock:^BOOL(id _Nullable evaluatedObject,
                                            NSDictionary<NSString*, id>* _Nullable bindings) {
        XCUIElement* element = evaluatedObject;
        return [element.identifier hasPrefix:@"platform_view"];
      }];
  XCUIElement* textView =
      [app.otherElements elementMatchingPredicate:predicateToFindPlatformView].textViews.firstMatch;
  if (![textView waitForExistenceWithTimeout:kSecondsToWaitForPlatformView]) {
    NSLog(@"%@", app.debugDescription);
    XCTFail(@"Failed due to not able to find any textView with %@ seconds",
            @(kSecondsToWaitForPlatformView));
  }

  XCTAssertNotNil(textView);
  XCTAssertEqualObjects(textView.label, @"");

  NSPredicate* predicate =
      [NSPredicate predicateWithFormat:@"label == %@", @"-gestureTouchesBegan-gestureTouchesEnded"];
  XCTNSPredicateExpectation* exception =
      [[XCTNSPredicateExpectation alloc] initWithPredicate:predicate object:textView];

  [textView tap];
  [self waitForExpectations:@[ exception ] timeout:kSecondsToWaitForPlatformView];
  XCTAssertEqualObjects(textView.label, @"-gestureTouchesBegan-gestureTouchesEnded");
}

- (void)testRejectPolicyEager {
  XCUIApplication* app = [[XCUIApplication alloc] init];
  app.launchArguments = @[ @"--gesture-reject-eager" ];
  [app launch];

  NSPredicate* predicateToFindPlatformView =
      [NSPredicate predicateWithBlock:^BOOL(id _Nullable evaluatedObject,
                                            NSDictionary<NSString*, id>* _Nullable bindings) {
        XCUIElement* element = evaluatedObject;
        return [element.identifier hasPrefix:@"platform_view"];
      }];
  XCUIElement* textView =
      [app.otherElements elementMatchingPredicate:predicateToFindPlatformView].textViews.firstMatch;
  if (![textView waitForExistenceWithTimeout:kSecondsToWaitForPlatformView]) {
    NSLog(@"%@", app.debugDescription);
    XCTFail(@"Failed due to not able to find any textView with %@ seconds",
            @(kSecondsToWaitForPlatformView));
  }

  XCTAssertNotNil(textView);
  XCTAssertEqualObjects(textView.label, @"");

  NSPredicate* predicate =
      [NSPredicate predicateWithBlock:^BOOL(id _Nullable evaluatedObject,
                                            NSDictionary<NSString*, id>* _Nullable bindings) {
        XCUIElement* view = (XCUIElement*)evaluatedObject;
        return [view.label containsString:@"-gestureTouchesBegan"];
      }];
  XCTNSPredicateExpectation* exception =
      [[XCTNSPredicateExpectation alloc] initWithPredicate:predicate object:textView];

  [textView tap];
  [self waitForExpectations:@[ exception ] timeout:kSecondsToWaitForPlatformView];
  XCTAssertTrue([textView.label containsString:@"-gestureTouchesBegan"]);
}

- (void)testAccept {
  XCUIApplication* app = [[XCUIApplication alloc] init];
  app.launchArguments = @[ @"--gesture-accept" ];
  [app launch];

  NSPredicate* predicateToFindPlatformView =
      [NSPredicate predicateWithBlock:^BOOL(id _Nullable evaluatedObject,
                                            NSDictionary<NSString*, id>* _Nullable bindings) {
        XCUIElement* element = evaluatedObject;
        return [element.identifier hasPrefix:@"platform_view"];
      }];
  XCUIElement* textView =
      [app.otherElements elementMatchingPredicate:predicateToFindPlatformView].textViews.firstMatch;
  if (![textView waitForExistenceWithTimeout:kSecondsToWaitForPlatformView]) {
    NSLog(@"%@", app.debugDescription);
    XCTFail(@"Failed due to not able to find any textView with %@ seconds",
            @(kSecondsToWaitForPlatformView));
  }

  XCTAssertNotNil(textView);
  XCTAssertEqualObjects(textView.label, @"");

  NSPredicate* predicate = [NSPredicate
      predicateWithFormat:@"label == %@",
                          @"-gestureTouchesBegan-gestureTouchesEnded-platformViewTapped"];
  XCTNSPredicateExpectation* exception =
      [[XCTNSPredicateExpectation alloc] initWithPredicate:predicate object:textView];

  [textView tap];

  [self waitForExpectations:@[ exception ] timeout:kSecondsToWaitForPlatformView];
  XCTAssertEqualObjects(textView.label,
                        @"-gestureTouchesBegan-gestureTouchesEnded-platformViewTapped");
}

- (void)testGestureWithMaskViewBlockingPlatformView {
  XCUIApplication* app = [[XCUIApplication alloc] init];
  app.launchArguments = @[ @"--gesture-accept", @"--maskview-blocking" ];
  [app launch];

  NSPredicate* predicateToFindPlatformView =
      [NSPredicate predicateWithBlock:^BOOL(id _Nullable evaluatedObject,
                                            NSDictionary<NSString*, id>* _Nullable bindings) {
        XCUIElement* element = evaluatedObject;
        return [element.identifier hasPrefix:@"platform_view"];
      }];
  XCUIElement* textView =
      [app.otherElements elementMatchingPredicate:predicateToFindPlatformView].textViews.firstMatch;
  if (![textView waitForExistenceWithTimeout:kSecondsToWaitForPlatformView]) {
    NSLog(@"%@", app.debugDescription);
    XCTFail(@"Failed due to not able to find any platformView with %@ seconds",
            @(kSecondsToWaitForPlatformView));
  }

  XCTAssertNotNil(textView);
  XCTAssertEqualObjects(textView.label, @"");

  NSPredicate* predicate = [NSPredicate
      predicateWithFormat:@"label == %@",
                          @"-gestureTouchesBegan-gestureTouchesEnded-platformViewTapped"];
  XCTNSPredicateExpectation* exception =
      [[XCTNSPredicateExpectation alloc] initWithPredicate:predicate object:textView];

  XCUICoordinate* coordinate =
      [self getNormalizedCoordinate:app
                              point:CGVectorMake(textView.frame.origin.x + 10,
                                                 textView.frame.origin.y + 10)];
  [coordinate tap];

  [self waitForExpectations:@[ exception ] timeout:kSecondsToWaitForPlatformView];
  XCTAssertEqualObjects(textView.label,
                        @"-gestureTouchesBegan-gestureTouchesEnded-platformViewTapped");
}

- (XCUICoordinate*)getNormalizedCoordinate:(XCUIApplication*)app point:(CGVector)vector {
  XCUICoordinate* appZero = [app coordinateWithNormalizedOffset:CGVectorMake(0, 0)];
  XCUICoordinate* coordinate = [appZero coordinateWithOffset:vector];
  return coordinate;
}

- (void)testGestureWithOverlappingPlatformViews {
  XCUIApplication* app = [[XCUIApplication alloc] init];
  app.launchArguments = @[ @"--gesture-accept-with-overlapping-platform-views" ];
  [app launch];

  XCUIElement* foreground = app.otherElements[@"platform_view[0]"];
  XCTAssertEqual(foreground.frame.origin.x, 50);
  XCTAssertEqual(foreground.frame.origin.y, 50);
  XCTAssertEqual(foreground.frame.size.width, 50);
  XCTAssertEqual(foreground.frame.size.height, 50);
  XCTAssertTrue([foreground waitForExistenceWithTimeout:kSecondsToWaitForPlatformView]);

  XCUIElement* background = app.otherElements[@"platform_view[1]"];
  XCTAssertEqual(background.frame.origin.x, 0);
  XCTAssertEqual(background.frame.origin.y, 0);
  XCTAssertEqual(background.frame.size.width, 150);
  XCTAssertEqual(background.frame.size.height, 150);
  XCTAssertTrue([background waitForExistenceWithTimeout:kSecondsToWaitForPlatformView]);

  XCUIElement* textView = foreground.textViews.firstMatch;
  XCTAssertTrue(textView.exists);

  XCTAssertTrue(foreground.isHittable);
  [foreground tap];

  XCTAssertEqualObjects(textView.label,
                        @"-gestureTouchesBegan-gestureTouchesEnded-platformViewTapped");
}
@end
