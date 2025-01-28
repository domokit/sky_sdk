// Copyright 2020 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import "GoldenPlatformViewTests.h"

@interface SpawnEngineTest : XCTestCase
@end

@implementation SpawnEngineTest

- (void)testSpawnEngineWorks {
  self.continueAfterFailure = NO;

  XCUIApplication* application = [[XCUIApplication alloc] init];
  application.launchArguments = @[ @"--spawn-engine-works" ];
  [application launch];

  XCUIElement* addTextField = application.textFields[@"ready"];
  XCTAssertTrue([addTextField waitForExistenceWithTimeout:30]);

  GoldenTestManager* manager =
      [[GoldenTestManager alloc] initWithLaunchArg:@"--spawn-engine-works"];
  [manager checkGoldenForTest:self rmesThreshold:kDefaultRmseThreshold];
}

@end
