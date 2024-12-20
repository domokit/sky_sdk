// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#ifndef FLUTTER_TESTING_SCENARIO_APP_IOS_SCENARIOS_SCENARIOSUITESTS_GOLDENIMAGE_H_
#define FLUTTER_TESTING_SCENARIO_APP_IOS_SCENARIOS_SCENARIOSUITESTS_GOLDENIMAGE_H_

#import <UIKit/UIKit.h>

NS_ASSUME_NONNULL_BEGIN

@interface GoldenImage : NSObject

@property(readonly, copy, nonatomic) NSString* goldenName;
@property(readonly, strong, nonatomic) UIImage* image;

// Initilize with the golden file's prefix.
//
// Create an image from a golden file named prefix+devicemodel.
- (instancetype)initWithGoldenNamePrefix:(NSString*)prefix;

// Compare this GoldenImage to `image`.
//
// Return YES if the `image` of this GoldenImage have the same pixels of provided `image`.
- (BOOL)compareGoldenToImage:(UIImage*)image rmesThreshold:(double)rmesThreshold;

@end

NS_ASSUME_NONNULL_END

#endif  // FLUTTER_TESTING_SCENARIO_APP_IOS_SCENARIOS_SCENARIOSUITESTS_GOLDENIMAGE_H_
