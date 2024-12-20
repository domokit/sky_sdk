// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_TESTING_SCENARIO_APP_IOS_SCENARIOS_SCENARIOSUITESTS_GOLDENTESTMANAGER_H_
#define FLUTTER_TESTING_SCENARIO_APP_IOS_SCENARIOS_SCENARIOSUITESTS_GOLDENTESTMANAGER_H_

#import <Foundation/Foundation.h>
#import <XCTest/XCTest.h>
#import "GoldenImage.h"

NS_ASSUME_NONNULL_BEGIN

extern NSDictionary* launchArgsMap;
const extern double kDefaultRmseThreshold;

// Manages a `GoldenPlatformViewTests`.
//
// It creates the correct `identifer` based on the `launchArg`.
// It also generates the correct GoldenImage based on the `identifier`.
@interface GoldenTestManager : NSObject

@property(readonly, strong, nonatomic) GoldenImage* goldenImage;
@property(readonly, copy, nonatomic) NSString* identifier;
@property(readonly, copy, nonatomic) NSString* launchArg;

// Initilize with launchArg.
//
// Crahes if the launchArg is not mapped in `Appdelegate.launchArgsMap`.
- (instancetype)initWithLaunchArg:(NSString*)launchArg;

// Take a sceenshot of the test app and check it has the same pixels with
// goldenImage inside the `GoldenTestManager`.
- (void)checkGoldenForTest:(XCTestCase*)test rmesThreshold:(double)rmesThreshold;

@end

NS_ASSUME_NONNULL_END

#endif  // FLUTTER_TESTING_SCENARIO_APP_IOS_SCENARIOS_SCENARIOSUITESTS_GOLDENTESTMANAGER_H_
