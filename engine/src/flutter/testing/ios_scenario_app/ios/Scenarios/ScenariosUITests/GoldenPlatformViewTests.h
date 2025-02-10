// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_TESTING_IOS_SCENARIO_APP_IOS_SCENARIOS_SCENARIOSUITESTS_GOLDENPLATFORMVIEWTESTS_H_
#define FLUTTER_TESTING_IOS_SCENARIO_APP_IOS_SCENARIOS_SCENARIOSUITESTS_GOLDENPLATFORMVIEWTESTS_H_

#import <XCTest/XCTest.h>
#import "GoldenTestManager.h"

NS_ASSUME_NONNULL_BEGIN

// The base class of all the PlatformView golden tests.
//
// A new PlatformView golden tests can subclass this and override the `-initiWithInvocation:`
// method, which then retun the `-initWithManager:invocation:`
//
// Then in any test method, call `checkPlatformViewGolden` to perform the golden test.
//
// This base class doesn't run any test case on its own.
@interface GoldenPlatformViewTests : XCTestCase

@property(nonatomic, strong) XCUIApplication* application;
@property(nonatomic, assign) double rmseThreadhold;

// Initialize with a `GoldenTestManager`.
- (instancetype)initWithManager:(GoldenTestManager*)manager invocation:(NSInvocation*)invocation;

// Take a sceenshot of the test app and check it has the same pixels with goldenImage inside the
// `GoldenTestManager`.
- (void)checkPlatformViewGolden;

@end

NS_ASSUME_NONNULL_END

#endif  // FLUTTER_TESTING_IOS_SCENARIO_APP_IOS_SCENARIOS_SCENARIOSUITESTS_GOLDENPLATFORMVIEWTESTS_H_
