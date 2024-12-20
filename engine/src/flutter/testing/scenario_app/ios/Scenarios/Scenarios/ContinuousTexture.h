// Copyright 2019 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_TESTING_SCENARIO_APP_IOS_SCENARIOS_SCENARIOS_CONTINUOUSTEXTURE_H_
#define FLUTTER_TESTING_SCENARIO_APP_IOS_SCENARIOS_SCENARIOS_CONTINUOUSTEXTURE_H_

#import <Flutter/Flutter.h>
#import <Foundation/Foundation.h>

NS_ASSUME_NONNULL_BEGIN

// A texture plugin that ready textures continuously.
@interface ContinuousTexture : NSObject <FlutterPlugin>

@end

// The testing texture used by |ContinuousTexture|
@interface FlutterScenarioTestTexture : NSObject <FlutterTexture>

@end

NS_ASSUME_NONNULL_END

#endif  // FLUTTER_TESTING_SCENARIO_APP_IOS_SCENARIOS_SCENARIOS_CONTINUOUSTEXTURE_H_
