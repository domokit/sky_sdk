// Copyright 2020 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import "StatusBarTest.h"

@implementation StatusBarTest

- (void)setUp {
  [super setUp];
  self.continueAfterFailure = NO;

  self.application = [[XCUIApplication alloc] init];
  self.application.launchArguments = @[ @"--tap-status-bar" ];
  [self.application launch];
}

- (void)testTapStatusBar {
  if (@available(iOS 13, *)) {
    XCUIApplication* systemApp =
        [[XCUIApplication alloc] initWithBundleIdentifier:@"com.apple.springboard"];
    XCUIElement* statusBar = [systemApp.statusBars firstMatch];
    if (statusBar.isHittable) {
      [statusBar tap];
    } else {
      XCUICoordinate* coordinates = [statusBar coordinateWithNormalizedOffset:CGVectorMake(0, 0)];
      [coordinates tap];
    }
  } else {
    [[self.application.statusBars firstMatch] tap];
  }

  XCUIElement* addTextField =
      self.application
          .textFields[@"0,PointerChange.add,device=0,buttons=0,signalKind=PointerSignalKind.none"];
  BOOL exists = [addTextField waitForExistenceWithTimeout:1];
  XCTAssertTrue(exists, @"");
  XCUIElement* downTextField =
      self.application
          .textFields[@"1,PointerChange.down,device=0,buttons=0,signalKind=PointerSignalKind.none"];
  exists = [downTextField waitForExistenceWithTimeout:1];
  XCTAssertTrue(exists, @"");
  XCUIElement* upTextField =
      self.application
          .textFields[@"2,PointerChange.up,device=0,buttons=0,signalKind=PointerSignalKind.none"];
  exists = [upTextField waitForExistenceWithTimeout:1];
  XCTAssertTrue(exists, @"");
}

@end
