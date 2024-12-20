// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include <cmath>
#import "flutter/third_party/spring_animation/spring_animation.h"
#include "gtest/gtest.h"

TEST(SpringAnimationTest, CanInitializeCorrectly) {
  SpringAnimation* animation = [[SpringAnimation alloc] initWithStiffness:1000
                                                                  damping:500
                                                                     mass:3
                                                          initialVelocity:0
                                                                fromValue:0
                                                                  toValue:1000];
  ASSERT_TRUE(animation.stiffness == 1000);
  ASSERT_TRUE(animation.damping == 500);
  ASSERT_TRUE(animation.mass == 3);
  ASSERT_TRUE(animation.initialVelocity == 0);
  ASSERT_TRUE(animation.fromValue == 0);
  ASSERT_TRUE(animation.toValue == 1000);
}

TEST(SpringAnimationTest, CurveFunctionCanWorkCorrectly) {
  // Here is the keyboard curve config on iOS platform.
  SpringAnimation* animation = [[SpringAnimation alloc] initWithStiffness:1000
                                                                  damping:500
                                                                     mass:3
                                                          initialVelocity:0
                                                                fromValue:0
                                                                  toValue:1000];
  const double accuracy = 1.0;
  const double startTime = 0;
  const double endTime = 0.6;

  const double startValue = [animation curveFunction:startTime];
  ASSERT_TRUE(fabs(startValue - animation.fromValue) < accuracy);
  const double toValue = [animation curveFunction:endTime];
  ASSERT_TRUE(fabs(toValue - animation.toValue) < accuracy);
}

TEST(SpringAnimationTest, CanUpdatePositionValuesAndCalculateCorrectly) {
  SpringAnimation* animation = [[SpringAnimation alloc] initWithStiffness:1000
                                                                  damping:500
                                                                     mass:3
                                                          initialVelocity:0
                                                                fromValue:0
                                                                  toValue:1000];
  const double startTime = 0;
  const double endTime = 0.6;

  const double startValue1 = [animation curveFunction:startTime];
  const double toValue1 = [animation curveFunction:endTime];

  animation.fromValue = 10;
  animation.toValue = 800;

  ASSERT_TRUE(animation.fromValue == 10);
  ASSERT_TRUE(animation.toValue == 800);

  const double startValue2 = [animation curveFunction:startTime];
  const double toValue2 = [animation curveFunction:endTime];
  ASSERT_TRUE(startValue2 > startValue1);
  ASSERT_TRUE(toValue2 < toValue1);
}
