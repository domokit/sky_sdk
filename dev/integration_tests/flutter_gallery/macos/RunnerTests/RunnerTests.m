// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import <FlutterMacOS/FlutterMacOS.h>
#import <XCTest/XCTest.h>

@interface RunnerTests : XCTestCase
@end

@implementation RunnerTests

- (void)testMenu {
  NSMenu *applicationMenu = ((FlutterAppDelegate *)NSApplication.sharedApplication.delegate).applicationMenu;
  XCTAssertEqual(applicationMenu.numberOfItems, 11);
  XCTAssertEqualObjects([applicationMenu itemAtIndex:0].title, @"About Flutter Gallery");

  NSMenu *mainMenu = NSApplication.sharedApplication.mainMenu;
  XCTAssertEqual([mainMenu indexOfItemWithSubmenu:applicationMenu], 0);

  // The number of submenu items changes depending on what the OS decides to inject.
  // Just check there's at least one per menu item.
  XCTAssertGreaterThanOrEqual([mainMenu itemWithTitle:@"File"].submenu.numberOfItems, 1);
  XCTAssertGreaterThanOrEqual([mainMenu itemWithTitle:@"Edit"].submenu.numberOfItems, 1);
  XCTAssertGreaterThanOrEqual([mainMenu itemWithTitle:@"View"].submenu.numberOfItems, 1);
  XCTAssertGreaterThanOrEqual([mainMenu itemWithTitle:@"Window"].submenu.numberOfItems, 1);

  NSMenu *helpMenu = NSApplication.sharedApplication.helpMenu;
  XCTAssertNotNil(helpMenu);
  // Only the help menu search text box.
  XCTAssertEqual(helpMenu.numberOfItems, 0);
}

@end
