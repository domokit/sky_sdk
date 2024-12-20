// Copyright 2013 The Chromium Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_TESTING_SCENARIO_APP_IOS_SCENARIOS_SCENARIOS_TEXTPLATFORMVIEW_H_
#define FLUTTER_TESTING_SCENARIO_APP_IOS_SCENARIOS_SCENARIOS_TEXTPLATFORMVIEW_H_

#import <Flutter/Flutter.h>

NS_ASSUME_NONNULL_BEGIN

@interface TextPlatformView : NSObject <FlutterPlatformView>

- (instancetype)initWithFrame:(CGRect)frame
               viewIdentifier:(int64_t)viewId
                    arguments:(id _Nullable)args
              binaryMessenger:(NSObject<FlutterBinaryMessenger>*)messenger;

- (UIView*)view;
@end

@interface TextPlatformViewFactory : NSObject <FlutterPlatformViewFactory>
- (instancetype)initWithMessenger:(NSObject<FlutterBinaryMessenger>*)messenger;
@end

NS_ASSUME_NONNULL_END

#endif  // FLUTTER_TESTING_SCENARIO_APP_IOS_SCENARIOS_SCENARIOS_TEXTPLATFORMVIEW_H_
