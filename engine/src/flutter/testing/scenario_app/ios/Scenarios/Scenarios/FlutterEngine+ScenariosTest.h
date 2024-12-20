// Copyright 2019 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_TESTING_SCENARIO_APP_IOS_SCENARIOS_SCENARIOS_FLUTTERENGINE_SCENARIOSTEST_H_
#define FLUTTER_TESTING_SCENARIO_APP_IOS_SCENARIOS_SCENARIOS_FLUTTERENGINE_SCENARIOSTEST_H_

#import <Flutter/Flutter.h>

NS_ASSUME_NONNULL_BEGIN
@interface FlutterEngine (ScenariosTest)
- (instancetype)initWithScenario:(NSString*)scenario
                  withCompletion:(nullable void (^)(void))engineRunCompletion;
- (FlutterEngine*)spawnWithEntrypoint:(nullable NSString*)entrypoint
                           libraryURI:(nullable NSString*)libraryURI
                         initialRoute:(nullable NSString*)initialRoute
                       entrypointArgs:(nullable NSArray<NSString*>*)entrypointArgs;
@end
NS_ASSUME_NONNULL_END

#endif  // FLUTTER_TESTING_SCENARIO_APP_IOS_SCENARIOS_SCENARIOS_FLUTTERENGINE_SCENARIOSTEST_H_
