// Copyright 2020 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_TESTING_IOS_SCENARIO_APP_IOS_SCENARIOS_SCENARIOSUITESTS_STATUSBARTEST_H_
#define FLUTTER_TESTING_IOS_SCENARIO_APP_IOS_SCENARIOS_SCENARIOSUITESTS_STATUSBARTEST_H_

#import <XCTest/XCTest.h>

NS_ASSUME_NONNULL_BEGIN

@interface StatusBarTest : XCTestCase
@property(nonatomic, strong) XCUIApplication* application;
@end

NS_ASSUME_NONNULL_END

#endif  // FLUTTER_TESTING_IOS_SCENARIO_APP_IOS_SCENARIOS_SCENARIOSUITESTS_STATUSBARTEST_H_
