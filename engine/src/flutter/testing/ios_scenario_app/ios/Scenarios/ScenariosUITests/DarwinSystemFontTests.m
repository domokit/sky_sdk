// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import <Flutter/Flutter.h>
#import <XCTest/XCTest.h>
#import "GoldenTestManager.h"

@interface DarwinSystemFontTests : XCTestCase

@end

@implementation DarwinSystemFontTests

- (void)testFontRendering {
  self.continueAfterFailure = NO;

  XCUIApplication* application = [[XCUIApplication alloc] init];
  application.launchArguments = @[ @"--darwin-system-font" ];
  [application launch];

  XCUIElement* addTextField = application.textFields[@"ready"];
  XCTAssertTrue([addTextField waitForExistenceWithTimeout:30]);

  GoldenTestManager* manager =
      [[GoldenTestManager alloc] initWithLaunchArg:@"--darwin-system-font"];
  [manager checkGoldenForTest:self rmesThreshold:kDefaultRmseThreshold];
}

@end
