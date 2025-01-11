// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import <XCTest/XCTest.h>

static const CGFloat kCompareAccuracy = 0.001;

@interface UnobstructedPlatformViewTests : XCTestCase

@end

@implementation UnobstructedPlatformViewTests

- (void)setUp {
  self.continueAfterFailure = NO;
}

// A is the layer, which z index is higher than the platform view.
// +--------+
// | PV     |  +---+
// +--------+  | A |
//             +---+
- (void)testNoOverlay {
  XCUIApplication* app = [[XCUIApplication alloc] init];
  app.launchArguments = @[ @"--platform-view-no-overlay-intersection" ];
  [app launch];

  XCUIElement* platform_view = app.otherElements[@"platform_view[0]"];
  XCTAssertTrue([platform_view waitForExistenceWithTimeout:1.0]);
  XCTAssertEqual(platform_view.frame.origin.x, 0);
  XCTAssertEqual(platform_view.frame.origin.y, 0);
  XCTAssertEqual(platform_view.frame.size.width, 250);
  XCTAssertEqual(platform_view.frame.size.height, 250);

  XCUIElement* overlay = app.otherElements[@"platform_view[0].overlay[0]"];
  XCTAssertFalse(overlay.exists);
}

// A is the layer above the platform view.
// +-----------------+
// | PV        +---+ |
// |           | A | |
// |           +---+ |
// +-----------------+
- (void)testOneOverlay {
  XCUIApplication* app = [[XCUIApplication alloc] init];
  app.launchArguments = @[ @"--platform-view" ];
  [app launch];

  XCUIElement* platform_view = app.otherElements[@"platform_view[0]"];
  XCTAssertTrue([platform_view waitForExistenceWithTimeout:1.0]);
  XCTAssertEqual(platform_view.frame.origin.x, 0);
  XCTAssertEqual(platform_view.frame.origin.y, 0);
  XCTAssertEqual(platform_view.frame.size.width, 250);
  XCTAssertEqual(platform_view.frame.size.height, 250);

  XCUIElement* overlay = app.otherElements[@"platform_view[0].overlay[0]"];
  XCTAssertTrue(overlay.exists);
  XCTAssertEqual(overlay.frame.origin.x, 150);
  XCTAssertEqual(overlay.frame.origin.y, 150);
  XCTAssertEqual(overlay.frame.size.width, 50);
  XCTAssertEqual(overlay.frame.size.height, 50);

  XCUIElement* overlayView = app.otherElements[@"platform_view[0].overlay_view[0]"];
  XCTAssertTrue(overlayView.exists);
  // Overlay should always be the same frame as the app.
  XCTAssertEqualWithAccuracy(overlayView.frame.origin.x, app.frame.origin.x, kCompareAccuracy);
  XCTAssertEqualWithAccuracy(overlayView.frame.origin.y, app.frame.origin.x, kCompareAccuracy);
  XCTAssertEqualWithAccuracy(overlayView.frame.size.width, app.frame.size.width, kCompareAccuracy);
  XCTAssertEqualWithAccuracy(overlayView.frame.size.height, app.frame.size.height,
                             kCompareAccuracy);
}

// A is the layer above the platform view.
// +-----------------+
// | PV        +---+ |
// +-----------| A |-+
//             +---+
- (void)testOneOverlayPartialIntersection {
  XCUIApplication* app = [[XCUIApplication alloc] init];
  app.launchArguments = @[ @"--platform-view-partial-intersection" ];
  [app launch];

  XCUIElement* platform_view = app.otherElements[@"platform_view[0]"];
  XCTAssertTrue([platform_view waitForExistenceWithTimeout:1.0]);
  XCTAssertEqual(platform_view.frame.origin.x, 0);
  XCTAssertEqual(platform_view.frame.origin.y, 0);
  XCTAssertEqual(platform_view.frame.size.width, 250);
  XCTAssertEqual(platform_view.frame.size.height, 250);

  XCUIElement* overlay = app.otherElements[@"platform_view[0].overlay[0]"];
  XCTAssertTrue(overlay.exists);
  XCTAssertEqual(overlay.frame.origin.x, 200);
  XCTAssertEqual(overlay.frame.origin.y, 245);
  XCTAssertEqual(overlay.frame.size.width, 50);
  // Half the height of the overlay.
  XCTAssertEqual(overlay.frame.size.height, 5);

  XCUIElement* overlayView = app.otherElements[@"platform_view[0].overlay_view[0]"];
  XCTAssertTrue(overlayView.exists);
  // Overlay should always be the same frame as the app.
  XCTAssertEqualWithAccuracy(overlayView.frame.origin.x, app.frame.origin.x, kCompareAccuracy);
  XCTAssertEqualWithAccuracy(overlayView.frame.origin.y, app.frame.origin.x, kCompareAccuracy);
  XCTAssertEqualWithAccuracy(overlayView.frame.size.width, app.frame.size.width, kCompareAccuracy);
  XCTAssertEqualWithAccuracy(overlayView.frame.size.height, app.frame.size.height,
                             kCompareAccuracy);
}

// A and B are the layers above the platform view.
// +--------------------+
// | PV  +------------+ |
// |      | B +-----+ | |
// |      +---|  A  |-+ |
// +----------|     |---+
//            +-----+
- (void)testTwoIntersectingOverlays {
  XCUIApplication* app = [[XCUIApplication alloc] init];
  app.launchArguments = @[ @"--platform-view-two-intersecting-overlays" ];
  [app launch];

  XCUIElement* platform_view = app.otherElements[@"platform_view[0]"];
  XCTAssertTrue([platform_view waitForExistenceWithTimeout:1.0]);
  XCTAssertEqual(platform_view.frame.origin.x, 0);
  XCTAssertEqual(platform_view.frame.origin.y, 0);
  XCTAssertEqual(platform_view.frame.size.width, 250);
  XCTAssertEqual(platform_view.frame.size.height, 250);

  XCUIElement* overlay = app.otherElements[@"platform_view[0].overlay[0]"];
  XCTAssertTrue(overlay.exists);
  XCTAssertEqual(overlay.frame.origin.x, 150);
  XCTAssertEqual(overlay.frame.origin.y, 150);
  XCTAssertEqual(overlay.frame.size.width, 75);
  XCTAssertEqual(overlay.frame.size.height, 75);

  XCTAssertFalse(app.otherElements[@"platform_view[0].overlay[1]"].exists);
}

// A, B, and C are the layers above the platform view.
// +-------------------------+
// | PV        +-----------+ |
// | +---+     | B +-----+ | |
// | | C |     +---|  A  |-+ |
// | +---+         +-----+   |
// +-------------------------+
- (void)testOneOverlayAndTwoIntersectingOverlays {
  XCUIApplication* app = [[XCUIApplication alloc] init];
  app.launchArguments = @[ @"--platform-view-one-overlay-two-intersecting-overlays" ];
  [app launch];

  XCUIElement* platform_view = app.otherElements[@"platform_view[0]"];
  XCTAssertTrue([platform_view waitForExistenceWithTimeout:1.0]);
  XCTAssertEqual(platform_view.frame.origin.x, 0);
  XCTAssertEqual(platform_view.frame.origin.y, 0);
  XCTAssertEqual(platform_view.frame.size.width, 250);
  XCTAssertEqual(platform_view.frame.size.height, 250);

  XCUIElement* overlay1 = app.otherElements[@"platform_view[0].overlay[0]"];
  XCTAssertTrue(overlay1.exists);
  XCTAssertEqual(overlay1.frame.origin.x, 75);
  XCTAssertEqual(overlay1.frame.origin.y, 150);
  XCTAssertEqual(overlay1.frame.size.width, 150);
  XCTAssertEqual(overlay1.frame.size.height, 100);

  // There are three non overlapping rects above platform view, which
  // FlutterPlatformViewsController merges into one.
  XCTAssertFalse(app.otherElements[@"platform_view[0].overlay[1]"].exists);

  XCUIElement* overlayView0 = app.otherElements[@"platform_view[0].overlay_view[0]"];
  XCTAssertTrue(overlayView0.exists);
  // Overlay should always be the same frame as the app.
  XCTAssertEqualWithAccuracy(overlayView0.frame.origin.x, app.frame.origin.x, kCompareAccuracy);
  XCTAssertEqualWithAccuracy(overlayView0.frame.origin.y, app.frame.origin.x, kCompareAccuracy);
  XCTAssertEqualWithAccuracy(overlayView0.frame.size.width, app.frame.size.width, kCompareAccuracy);
  XCTAssertEqualWithAccuracy(overlayView0.frame.size.height, app.frame.size.height,
                             kCompareAccuracy);
}

// A is the layer, which z index is higher than the platform view.
// +--------+
// | PV     |  +---+
// +--------+  | A |
// +--------+  +---+
// | PV     |
// +--------+
- (void)testMultiplePlatformViewsWithoutOverlays {
  XCUIApplication* app = [[XCUIApplication alloc] init];
  app.launchArguments = @[ @"--platform-view-multiple-without-overlays" ];
  [app launch];

  XCUIElement* platform_view1 = app.otherElements[@"platform_view[0]"];
  XCTAssertTrue([platform_view1 waitForExistenceWithTimeout:1.0]);
  XCTAssertEqual(platform_view1.frame.origin.x, 0);
  XCTAssertEqual(platform_view1.frame.origin.y, 300);
  XCTAssertEqual(platform_view1.frame.size.width, 250);
  XCTAssertEqual(platform_view1.frame.size.height, 250);

  XCUIElement* platform_view2 = app.otherElements[@"platform_view[1]"];
  XCTAssertTrue(platform_view2.exists);
  XCTAssertEqual(platform_view2.frame.origin.x, 0);
  XCTAssertEqual(platform_view2.frame.origin.y, 0);
  XCTAssertEqual(platform_view2.frame.size.width, 250);
  XCTAssertEqual(platform_view2.frame.size.height, 250);

  XCTAssertFalse(app.otherElements[@"platform_view[0].overlay[0]"].exists);
  XCTAssertFalse(app.otherElements[@"platform_view[1].overlay[0]"].exists);
  XCTAssertFalse(app.otherElements[@"platform_view[0].overlay_view[0]"].exists);
  XCTAssertFalse(app.otherElements[@"platform_view[1].overlay_view[0]"].exists);
}

// A is the layer above both platform view.
// +------------+
// | PV  +----+ |
// +-----| A  |-+
// +-----|    |-+
// | PV  +----+ |
// +------------+
- (void)testMultiplePlatformViewsWithOverlays {
  XCUIApplication* app = [[XCUIApplication alloc] init];
  app.launchArguments = @[ @"--platform-view-multiple-background-foreground" ];
  [app launch];

  XCUIElement* platform_view1 = app.otherElements[@"platform_view[0]"];
  XCTAssertTrue([platform_view1 waitForExistenceWithTimeout:1.0]);
  XCTAssertEqual(platform_view1.frame.origin.x, 25);
  XCTAssertEqual(platform_view1.frame.origin.y, 300);
  XCTAssertEqual(platform_view1.frame.size.width, 250);
  XCTAssertEqual(platform_view1.frame.size.height, 250);

  XCUIElement* platform_view2 = app.otherElements[@"platform_view[1]"];
  XCTAssertTrue(platform_view2.exists);
  XCTAssertEqual(platform_view2.frame.origin.x, 25);
  XCTAssertEqual(platform_view2.frame.origin.y, 0);
  XCTAssertEqual(platform_view2.frame.size.width, 250);
  XCTAssertEqual(platform_view2.frame.size.height, 250);

  XCUIElement* overlay1 = app.otherElements[@"platform_view[1].overlay[0]"];
  XCTAssertTrue(overlay1.exists);
  XCTAssertEqual(overlay1.frame.origin.x, 25);
  XCTAssertEqual(overlay1.frame.origin.y, 0);
  XCTAssertEqual(overlay1.frame.size.width, 225);
  XCTAssertEqual(overlay1.frame.size.height, 500);

  XCUIElement* overlayView0 = app.otherElements[@"platform_view[1].overlay_view[0]"];
  XCTAssertTrue(overlayView0.exists);
  // Overlay should always be the same frame as the app.
  XCTAssertEqualWithAccuracy(overlayView0.frame.origin.x, app.frame.origin.x, kCompareAccuracy);
  XCTAssertEqualWithAccuracy(overlayView0.frame.origin.y, app.frame.origin.x, kCompareAccuracy);
  XCTAssertEqualWithAccuracy(overlayView0.frame.size.width, app.frame.size.width, kCompareAccuracy);
  XCTAssertEqualWithAccuracy(overlayView0.frame.size.height, app.frame.size.height,
                             kCompareAccuracy);
}

// More then two overlays are merged into a single layer.
// +---------------------+
// | +---+  +---+  +---+ |
// | | A |  | B |  | C | |
// | +---+  +---+  +---+ |
// | +-------+           |
// +-|   D   |-----------+
//   +-------+
- (void)testPlatformViewsMaxOverlays {
  XCUIApplication* app = [[XCUIApplication alloc] init];
  app.launchArguments = @[ @"--platform-view-max-overlays" ];
  [app launch];

  XCUIElement* platform_view = app.otherElements[@"platform_view[0]"];
  XCTAssertTrue([platform_view waitForExistenceWithTimeout:1.0]);
  XCTAssertEqual(platform_view.frame.origin.x, 0);
  XCTAssertEqual(platform_view.frame.origin.y, 0);
  XCTAssertEqual(platform_view.frame.size.width, 250);
  XCTAssertEqual(platform_view.frame.size.height, 250);

  XCUIElement* overlay = app.otherElements[@"platform_view[0].overlay[0]"];
  XCTAssertTrue(overlay.exists);
  XCTAssertFalse(app.otherElements[@"platform_view[0].overlay[1]"].exists);
  XCTAssertTrue(CGRectContainsRect(platform_view.frame, overlay.frame));

  XCUIElement* overlayView0 = app.otherElements[@"platform_view[0].overlay_view[0]"];
  XCTAssertTrue(overlayView0.exists);
  // Overlay should always be the same frame as the app.
  XCTAssertEqualWithAccuracy(overlayView0.frame.origin.x, app.frame.origin.x, kCompareAccuracy);
  XCTAssertEqualWithAccuracy(overlayView0.frame.origin.y, app.frame.origin.x, kCompareAccuracy);
  XCTAssertEqualWithAccuracy(overlayView0.frame.size.width, app.frame.size.width, kCompareAccuracy);
  XCTAssertEqualWithAccuracy(overlayView0.frame.size.height, app.frame.size.height,
                             kCompareAccuracy);

  XCUIElement* overlayView1 = app.otherElements[@"platform_view[0].overlay_view[1]"];
  XCTAssertFalse(overlayView1.exists);
}

// Platform view surrounded by adjacent layers on each side should not create any overlays.
//      +----+
//      | B  |
//  +---+----+---+
//  | A | PV | C |
//  +---+----+---+
//      | D  |
//      +----+
- (void)testPlatformViewsWithAdjacentSurroundingLayersAndFractionalCoordinate {
  XCUIApplication* app = [[XCUIApplication alloc] init];
  app.launchArguments = @[ @"--platform-view-surrounding-layers-fractional-coordinate" ];
  [app launch];

  XCUIElement* platform_view = app.otherElements[@"platform_view[0]"];
  XCTAssertTrue([platform_view waitForExistenceWithTimeout:1.0]);

  CGFloat scale = [UIScreen mainScreen].scale;
  XCTAssertEqual(platform_view.frame.origin.x * scale, 100.5);
  XCTAssertEqual(platform_view.frame.origin.y * scale, 100.5);
  XCTAssertEqual(platform_view.frame.size.width * scale, 100);
  XCTAssertEqual(platform_view.frame.size.height * scale, 100);

  XCUIElement* overlay = app.otherElements[@"platform_view[0].overlay[0]"];
  XCTAssertFalse(overlay.exists);
}

// Platform view partially intersect with a layer in fractional coordinate.
// +-------+
// |       |
// | PV +--+--+
// |    |     |
// +----+  A  |
//      |     |
//      +-----+
- (void)testPlatformViewsWithPartialIntersectionAndFractionalCoordinate {
  XCUIApplication* app = [[XCUIApplication alloc] init];
  app.launchArguments = @[ @"--platform-view-partial-intersection-fractional-coordinate" ];
  [app launch];

  XCUIElement* platform_view = app.otherElements[@"platform_view[0]"];
  XCTAssertTrue([platform_view waitForExistenceWithTimeout:1.0]);

  CGFloat scale = [UIScreen mainScreen].scale;
  XCTAssertEqual(platform_view.frame.origin.x * scale, 0.5);
  XCTAssertEqual(platform_view.frame.origin.y * scale, 0.5);
  XCTAssertEqual(platform_view.frame.size.width * scale, 100);
  XCTAssertEqual(platform_view.frame.size.height * scale, 100);

  XCUIElement* overlay = app.otherElements[@"platform_view[0].overlay[0]"];
  XCTAssert(overlay.exists);

  // We want to make sure the overlay covers the edge (which is at 100.5).
  XCTAssertEqual(CGRectGetMaxX(overlay.frame) * scale, 101);
  XCTAssertEqual(CGRectGetMaxY(overlay.frame) * scale, 101);
}
@end
