// Copyright 2020 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import <XCTest/XCTest.h>
#import "GoldenTestManager.h"

@interface AppExtensionTests : XCTestCase
@property(nonatomic, strong) XCUIApplication* hostApplication;
@end

@implementation AppExtensionTests

- (void)setUp {
  [super setUp];
  self.continueAfterFailure = NO;
  self.hostApplication =
      [[XCUIApplication alloc] initWithBundleIdentifier:@"dev.flutter.FlutterAppExtensionTestHost"];
}

- (void)testAppExtensionLaunching {
  // Launch the Scenarios app first to ensure it's installed then close it.
  XCUIApplication* app = [[XCUIApplication alloc] init];
  [app launch];
  [app terminate];

  [self.hostApplication launch];
  XCUIElement* button = self.hostApplication.buttons[@"Open Share"];
  if (![button waitForExistenceWithTimeout:10]) {
    NSLog(@"%@", self.hostApplication.debugDescription);
    XCTFail(@"Failed due to not able to find any button with %@ seconds", @(10));
  }
  [button tap];
  BOOL launchedExtensionInFlutter = NO;

  // Custom share extension button (like the one in this test) does not have a
  // unique identity on older versions of iOS. They are all identified as
  // `XCElementSnapshotPrivilegedValuePlaceholder`. On iOS 17, they are
  // identified by name. Loop through all the buttons labeled
  // `XCElementSnapshotPrivilegedValuePlaceholder` or `Scenarios` to find the
  // Flutter one.
  NSPredicate* cellPredicate = [NSPredicate
      predicateWithFormat:
          @"label == 'XCElementSnapshotPrivilegedValuePlaceholder' OR label = 'Scenarios'"];

  // Wait for the first cell matching the cellPredicate on the share sheet to appear.
  XCUIElement* firstCell =
      [self.hostApplication.collectionViews.cells matchingPredicate:cellPredicate].firstMatch;
  if (![firstCell waitForExistenceWithTimeout:10]) {
    NSLog(@"%@", self.hostApplication.debugDescription);
    XCTFail(@"Failed due to not able to find Scenarios cell within %@ seconds", @(10));
  }

  NSArray<XCUIElement*>* shareSheetCells =
      [self.hostApplication.collectionViews.cells matchingPredicate:cellPredicate]
          .allElementsBoundByIndex;
  for (XCUIElement* shareSheetCell in shareSheetCells) {
    [shareSheetCell tap];

    XCUIElement* flutterView = self.hostApplication.otherElements[@"flutter_view"];
    if ([flutterView waitForExistenceWithTimeout:10]) {
      launchedExtensionInFlutter = YES;
      break;
    }

    // All the built-in share extensions have a Cancel button.
    // Tap the Cancel button to close the built-in extension.
    XCUIElement* cancel = self.hostApplication.buttons[@"Cancel"];
    if ([cancel waitForExistenceWithTimeout:10]) {
      [cancel tap];
    }
  }
  // App extension successfully launched flutter view.
  XCTAssertTrue(launchedExtensionInFlutter);
}

@end
