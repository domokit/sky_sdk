// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import <OCMock/OCMock.h>
#import <XCTest/XCTest.h>

#import "flutter/shell/platform/darwin/common/framework/Headers/FlutterMacros.h"
#import "flutter/shell/platform/darwin/ios/framework/Headers/FlutterPluginAppLifeCycleDelegate.h"

FLUTTER_ASSERT_ARC

@interface FakePlugin : NSObject <FlutterApplicationLifeCycleDelegate>
@end

@implementation FakePlugin
@end

@interface FlutterPluginAppLifeCycleDelegateTest : XCTestCase
@end

@implementation FlutterPluginAppLifeCycleDelegateTest

- (void)testCreate {
  FlutterPluginAppLifeCycleDelegate* delegate = [[FlutterPluginAppLifeCycleDelegate alloc] init];
  XCTAssertNotNil(delegate);
}

#if not APPLICATION_EXTENSION_API_ONLY
- (void)testDidEnterBackground {
  XCTNSNotificationExpectation* expectation = [[XCTNSNotificationExpectation alloc]
      initWithName:UIApplicationDidEnterBackgroundNotification];
  FlutterPluginAppLifeCycleDelegate* delegate = [[FlutterPluginAppLifeCycleDelegate alloc] init];
  id plugin = OCMProtocolMock(@protocol(FlutterPlugin));
  [delegate addDelegate:plugin];
  [[NSNotificationCenter defaultCenter]
      postNotificationName:UIApplicationDidEnterBackgroundNotification
                    object:nil];

  [self waitForExpectations:@[ expectation ] timeout:5.0];
  OCMVerify([plugin applicationDidEnterBackground:[UIApplication sharedApplication]]);
}

- (void)testWillEnterForeground {
  XCTNSNotificationExpectation* expectation = [[XCTNSNotificationExpectation alloc]
      initWithName:UIApplicationWillEnterForegroundNotification];

  FlutterPluginAppLifeCycleDelegate* delegate = [[FlutterPluginAppLifeCycleDelegate alloc] init];
  id plugin = OCMProtocolMock(@protocol(FlutterPlugin));
  [delegate addDelegate:plugin];
  [[NSNotificationCenter defaultCenter]
      postNotificationName:UIApplicationWillEnterForegroundNotification
                    object:nil];
  [self waitForExpectations:@[ expectation ] timeout:5.0];
  OCMVerify([plugin applicationWillEnterForeground:[UIApplication sharedApplication]]);
}

- (void)testWillResignActive {
  XCTNSNotificationExpectation* expectation =
      [[XCTNSNotificationExpectation alloc] initWithName:UIApplicationWillResignActiveNotification];

  FlutterPluginAppLifeCycleDelegate* delegate = [[FlutterPluginAppLifeCycleDelegate alloc] init];
  id plugin = OCMProtocolMock(@protocol(FlutterPlugin));
  [delegate addDelegate:plugin];
  [[NSNotificationCenter defaultCenter]
      postNotificationName:UIApplicationWillResignActiveNotification
                    object:nil];
  [self waitForExpectations:@[ expectation ] timeout:5.0];
  OCMVerify([plugin applicationWillResignActive:[UIApplication sharedApplication]]);
}

- (void)testDidBecomeActive {
  XCTNSNotificationExpectation* expectation =
      [[XCTNSNotificationExpectation alloc] initWithName:UIApplicationDidBecomeActiveNotification];

  FlutterPluginAppLifeCycleDelegate* delegate = [[FlutterPluginAppLifeCycleDelegate alloc] init];
  id plugin = OCMProtocolMock(@protocol(FlutterPlugin));
  [delegate addDelegate:plugin];
  [[NSNotificationCenter defaultCenter]
      postNotificationName:UIApplicationDidBecomeActiveNotification
                    object:nil];
  [self waitForExpectations:@[ expectation ] timeout:5.0];
  OCMVerify([plugin applicationDidBecomeActive:[UIApplication sharedApplication]]);
}

- (void)testWillTerminate {
  XCTNSNotificationExpectation* expectation =
      [[XCTNSNotificationExpectation alloc] initWithName:UIApplicationWillTerminateNotification];

  FlutterPluginAppLifeCycleDelegate* delegate = [[FlutterPluginAppLifeCycleDelegate alloc] init];
  id plugin = OCMProtocolMock(@protocol(FlutterPlugin));
  [delegate addDelegate:plugin];
  [[NSNotificationCenter defaultCenter] postNotificationName:UIApplicationWillTerminateNotification
                                                      object:nil];
  [self waitForExpectations:@[ expectation ] timeout:5.0];
  OCMVerify([plugin applicationWillTerminate:[UIApplication sharedApplication]]);
}

- (void)testReleasesPluginOnDealloc {
  __weak id<FlutterApplicationLifeCycleDelegate> weakPlugin;
  __weak FlutterPluginAppLifeCycleDelegate* weakDelegate;
  @autoreleasepool {
    FakePlugin* fakePlugin = [[FakePlugin alloc] init];
    weakPlugin = fakePlugin;
    FlutterPluginAppLifeCycleDelegate* delegate = [[FlutterPluginAppLifeCycleDelegate alloc] init];
    [delegate addDelegate:fakePlugin];
    weakDelegate = delegate;
  }
  XCTAssertNil(weakPlugin);
  XCTAssertNil(weakDelegate);
}

#endif

@end
