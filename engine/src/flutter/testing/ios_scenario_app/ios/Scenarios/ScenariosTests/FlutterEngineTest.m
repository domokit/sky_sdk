// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import <Flutter/Flutter.h>
#import <XCTest/XCTest.h>

#import "AppDelegate.h"

@interface FlutterEngineTest : XCTestCase
@end

@implementation FlutterEngineTest

extern NSNotificationName const FlutterViewControllerWillDealloc;

- (void)testIsolateId {
  FlutterEngine* engine = [[FlutterEngine alloc] initWithName:@"test" project:nil];
  XCTAssertNil(engine.isolateId);
  [self keyValueObservingExpectationForObject:engine keyPath:@"isolateId" handler:nil];

  XCTAssertTrue([engine runWithEntrypoint:nil]);

  [self waitForExpectationsWithTimeout:30.0 handler:nil];

  XCTAssertNotNil(engine.isolateId);
  XCTAssertTrue([engine.isolateId hasPrefix:@"isolates/"]);

  [engine destroyContext];

  XCTAssertNil(engine.isolateId);
}

- (void)testChannelSetup {
  FlutterEngine* engine = [[FlutterEngine alloc] initWithName:@"test" project:nil];
  XCTAssertNil(engine.navigationChannel);
  XCTAssertNil(engine.platformChannel);
  XCTAssertNil(engine.lifecycleChannel);

  XCTAssertTrue([engine run]);

  XCTAssertNotNil(engine.navigationChannel);
  XCTAssertNotNil(engine.platformChannel);
  XCTAssertNotNil(engine.lifecycleChannel);

  [engine destroyContext];

  XCTAssertNil(engine.navigationChannel);
  XCTAssertNil(engine.platformChannel);
  XCTAssertNil(engine.lifecycleChannel);
}

// https://github.com/flutter/flutter/issues/123776
- (void)testReleaseViewControllerAfterDestroyContextInHeadlessMode {
  FlutterEngine* engine = [[FlutterEngine alloc] initWithName:@"test"
                                                      project:nil
                                       allowHeadlessExecution:YES];
  XCTAssertNil(engine.navigationChannel);
  XCTAssertNil(engine.platformChannel);
  XCTAssertNil(engine.lifecycleChannel);

  XCTAssertTrue([engine run]);

  XCTAssertNotNil(engine.navigationChannel);
  XCTAssertNotNil(engine.platformChannel);
  XCTAssertNotNil(engine.lifecycleChannel);
  XCTestExpectation* expectation =
      [[XCTestExpectation alloc] initWithDescription:@"notification called"];
  @autoreleasepool {
    FlutterViewController* viewController = [[FlutterViewController alloc] initWithEngine:engine
                                                                                  nibName:nil
                                                                                   bundle:nil];
    [engine setViewController:viewController];
    [engine destroyContext];
    [[NSNotificationCenter defaultCenter] addObserverForName:FlutterViewControllerWillDealloc
                                                      object:nil
                                                       queue:[NSOperationQueue mainQueue]
                                                  usingBlock:^(NSNotification* _Nonnull note) {
                                                    [expectation fulfill];
                                                  }];
    viewController = nil;
  }
  [self waitForExpectations:@[ expectation ] timeout:30.0];
  XCTAssertNil(engine.navigationChannel);
  XCTAssertNil(engine.platformChannel);
  XCTAssertNil(engine.lifecycleChannel);
}

@end
