// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterRestorationPlugin.h"

#import <OCMock/OCMock.h>
#import <XCTest/XCTest.h>

#import "flutter/shell/platform/darwin/common/framework/Headers/FlutterChannels.h"
#import "flutter/shell/platform/darwin/common/framework/Headers/FlutterMacros.h"
#import "flutter/shell/platform/darwin/ios/framework/Headers/FlutterViewController.h"
#import "flutter/shell/platform/darwin/ios/framework/Source/FlutterViewController_Internal.h"

FLUTTER_ASSERT_ARC

@interface FlutterRestorationPlugin ()
- (void)handleMethodCall:(FlutterMethodCall*)call result:(FlutterResult)result;
@end

@interface FlutterRestorationPluginTest : XCTestCase
@end

@implementation FlutterRestorationPluginTest {
  id restorationChannel;
}

- (void)setUp {
  [super setUp];
  restorationChannel = OCMClassMock([FlutterMethodChannel class]);
}

- (void)tearDown {
  [restorationChannel stopMocking];

  [super tearDown];
}

#pragma mark - Tests

- (void)testRestoratonViewControllerEncodeAndDecode {
  FlutterEngine* engine = [[FlutterEngine alloc] initWithName:@"test"
                                                      project:nil
                                       allowHeadlessExecution:YES
                                           restorationEnabled:YES];
  [engine run];
  FlutterViewController* flutterViewController =
      [[FlutterViewController alloc] initWithEngine:engine nibName:nil bundle:nil];
  FlutterRestorationPlugin* restorationPlugin = flutterViewController.restorationPlugin;

  NSData* data = [@"testrestortiondata" dataUsingEncoding:NSUTF8StringEncoding];
  [restorationPlugin setRestorationData:data];

  NSKeyedArchiver* archiver = [[NSKeyedArchiver alloc] initRequiringSecureCoding:YES];
  [flutterViewController encodeRestorableStateWithCoder:archiver];

  [restorationPlugin setRestorationData:nil];

  NSKeyedUnarchiver* unarchiver =
      [[NSKeyedUnarchiver alloc] initForReadingWithData:archiver.encodedData];
  [flutterViewController decodeRestorableStateWithCoder:unarchiver];

  XCTAssert([[restorationPlugin restorationData] isEqualToData:data],
            "Restoration state data must be equal");
}

- (void)testRestorationEnabledWaitsForData {
  FlutterRestorationPlugin* restorationPlugin =
      [[FlutterRestorationPlugin alloc] initWithChannel:restorationChannel restorationEnabled:YES];

  FlutterMethodCall* methodCall = [FlutterMethodCall methodCallWithMethodName:@"get" arguments:nil];
  __block id capturedResult;
  [restorationPlugin handleMethodCall:methodCall
                               result:^(id _Nullable result) {
                                 capturedResult = result;
                               }];
  XCTAssertNil(capturedResult);

  NSData* data = [@"testrestortiondata" dataUsingEncoding:NSUTF8StringEncoding];
  [restorationPlugin setRestorationData:data];
  XCTAssertEqual([capturedResult count], 2u);
  XCTAssertEqual([capturedResult objectForKey:@"enabled"], @YES);
  XCTAssertEqualObjects([[capturedResult objectForKey:@"data"] data], data);
}

- (void)testRestorationDisabledRespondsRightAway {
  FlutterRestorationPlugin* restorationPlugin =
      [[FlutterRestorationPlugin alloc] initWithChannel:restorationChannel restorationEnabled:NO];

  FlutterMethodCall* methodCall = [FlutterMethodCall methodCallWithMethodName:@"get" arguments:nil];
  __block id capturedResult;
  [restorationPlugin handleMethodCall:methodCall
                               result:^(id _Nullable result) {
                                 capturedResult = result;
                               }];
  XCTAssertEqual([capturedResult count], 1u);
  XCTAssertEqual([capturedResult objectForKey:@"enabled"], @NO);
}

- (void)testRespondsRightAwayWhenDataIsSet {
  FlutterRestorationPlugin* restorationPlugin =
      [[FlutterRestorationPlugin alloc] initWithChannel:restorationChannel restorationEnabled:YES];

  NSData* data = [@"testrestortiondata" dataUsingEncoding:NSUTF8StringEncoding];
  [restorationPlugin setRestorationData:data];

  __block id capturedResult;
  FlutterMethodCall* methodCall = [FlutterMethodCall methodCallWithMethodName:@"get" arguments:nil];
  [restorationPlugin handleMethodCall:methodCall
                               result:^(id _Nullable result) {
                                 capturedResult = result;
                               }];
  XCTAssertEqual([capturedResult count], 2u);
  XCTAssertEqual([capturedResult objectForKey:@"enabled"], @YES);
  XCTAssertEqualObjects([[capturedResult objectForKey:@"data"] data], data);
}

- (void)testRespondsWithNoDataWhenRestorationIsCompletedWithoutData {
  FlutterRestorationPlugin* restorationPlugin =
      [[FlutterRestorationPlugin alloc] initWithChannel:restorationChannel restorationEnabled:YES];

  FlutterMethodCall* methodCall = [FlutterMethodCall methodCallWithMethodName:@"get" arguments:nil];
  __block id capturedResult;
  [restorationPlugin handleMethodCall:methodCall
                               result:^(id _Nullable result) {
                                 capturedResult = result;
                               }];
  XCTAssertNil(capturedResult);

  [restorationPlugin markRestorationComplete];
  XCTAssertEqual([capturedResult count], 1u);
  XCTAssertEqual([capturedResult objectForKey:@"enabled"], @YES);
}

- (void)testRespondsRightAwayWithNoDataWhenRestorationIsCompleted {
  FlutterRestorationPlugin* restorationPlugin =
      [[FlutterRestorationPlugin alloc] initWithChannel:restorationChannel restorationEnabled:YES];

  [restorationPlugin markRestorationComplete];

  __block id capturedResult;
  FlutterMethodCall* methodCall = [FlutterMethodCall methodCallWithMethodName:@"get" arguments:nil];
  [restorationPlugin handleMethodCall:methodCall
                               result:^(id _Nullable result) {
                                 capturedResult = result;
                               }];
  XCTAssertEqual([capturedResult count], 1u);
  XCTAssertEqual([capturedResult objectForKey:@"enabled"], @YES);
}

- (void)testReturnsDataSetByFramework {
  FlutterRestorationPlugin* restorationPlugin =
      [[FlutterRestorationPlugin alloc] initWithChannel:restorationChannel restorationEnabled:YES];
  [restorationPlugin markRestorationComplete];

  NSData* data = [@"testrestortiondata" dataUsingEncoding:NSUTF8StringEncoding];
  FlutterMethodCall* methodCall = [FlutterMethodCall
      methodCallWithMethodName:@"put"
                     arguments:[FlutterStandardTypedData typedDataWithBytes:data]];
  [restorationPlugin handleMethodCall:methodCall
                               result:^(id _Nullable result) {
                                 XCTAssertNil(result);
                               }];
  XCTAssertEqualObjects([restorationPlugin restorationData], data);
}

- (void)testRespondsWithDataSetByFramework {
  FlutterRestorationPlugin* restorationPlugin =
      [[FlutterRestorationPlugin alloc] initWithChannel:restorationChannel restorationEnabled:YES];
  [restorationPlugin markRestorationComplete];

  NSData* data = [@"testrestortiondata" dataUsingEncoding:NSUTF8StringEncoding];
  FlutterMethodCall* methodCall = [FlutterMethodCall
      methodCallWithMethodName:@"put"
                     arguments:[FlutterStandardTypedData typedDataWithBytes:data]];
  [restorationPlugin handleMethodCall:methodCall
                               result:^(id _Nullable result) {
                                 XCTAssertNil(result);
                               }];
  XCTAssertEqualObjects([restorationPlugin restorationData], data);

  __block id capturedResult;
  methodCall = [FlutterMethodCall methodCallWithMethodName:@"get" arguments:nil];
  [restorationPlugin handleMethodCall:methodCall
                               result:^(id _Nullable result) {
                                 capturedResult = result;
                               }];
  XCTAssertEqual([capturedResult count], 2u);
  XCTAssertEqual([capturedResult objectForKey:@"enabled"], @YES);
  XCTAssertEqualObjects([[capturedResult objectForKey:@"data"] data], data);
}

- (void)testResetClearsData {
  FlutterRestorationPlugin* restorationPlugin =
      [[FlutterRestorationPlugin alloc] initWithChannel:restorationChannel restorationEnabled:YES];
  [restorationPlugin markRestorationComplete];

  NSData* data = [@"testrestortiondata" dataUsingEncoding:NSUTF8StringEncoding];
  FlutterMethodCall* methodCall = [FlutterMethodCall
      methodCallWithMethodName:@"put"
                     arguments:[FlutterStandardTypedData typedDataWithBytes:data]];
  [restorationPlugin handleMethodCall:methodCall
                               result:^(id _Nullable result) {
                                 XCTAssertNil(result);
                               }];
  XCTAssertEqualObjects([restorationPlugin restorationData], data);

  [restorationPlugin reset];
  XCTAssertNil([restorationPlugin restorationData]);

  __block id capturedResult;
  methodCall = [FlutterMethodCall methodCallWithMethodName:@"get" arguments:nil];
  [restorationPlugin handleMethodCall:methodCall
                               result:^(id _Nullable result) {
                                 capturedResult = result;
                               }];
  XCTAssertEqual([capturedResult count], 1u);
  XCTAssertEqual([capturedResult objectForKey:@"enabled"], @YES);
}

@end
