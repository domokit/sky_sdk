//
// Copyright (c) Meta Platforms, Inc. and affiliates.
//
// This source code is licensed under the MIT license found in the
// LICENSE file in the root directory of this source tree.
//
// @flow
// @format
//

#import "spring_animation.h"

#include <Foundation/Foundation.h>

@interface SpringAnimation ()

@property(nonatomic, assign) double zeta;
@property(nonatomic, assign) double omega0;
@property(nonatomic, assign) double omega1;
@property(nonatomic, assign) double v0;

@end

// Spring code adapted from React Native's Animation Library, see:
// https://github.com/facebook/react-native/blob/main/Libraries/Animated/animations/SpringAnimation.js
@implementation SpringAnimation
- (instancetype)initWithStiffness:(double)stiffness
                          damping:(double)damping
                             mass:(double)mass
                  initialVelocity:(double)initialVelocity
                        fromValue:(double)fromValue
                          toValue:(double)toValue {
  self = [super init];
  if (self) {
    _stiffness = stiffness;
    _damping = damping;
    _mass = mass;
    _initialVelocity = initialVelocity;
    _fromValue = fromValue;
    _toValue = toValue;

    _zeta = _damping / (2 * sqrt(_stiffness * _mass));  // Damping ratio.
    _omega0 = sqrt(_stiffness / _mass);             // Undamped angular frequency of the oscillator.
    _omega1 = _omega0 * sqrt(1.0 - _zeta * _zeta);  // Exponential decay.
    _v0 = -_initialVelocity;
  }
  return self;
}

- (double)curveFunction:(double)t {
  const double x0 = _toValue - _fromValue;

  double y;
  if (_zeta < 1) {
    // Under damped.
    const double envelope = exp(-_zeta * _omega0 * t);
    y = _toValue - envelope * (((_v0 + _zeta * _omega0 * x0) / _omega1) * sin(_omega1 * t) +
                               x0 * cos(_omega1 * t));
  } else {
    // Critically damped.
    const double envelope = exp(-_omega0 * t);
    y = _toValue - envelope * (x0 + (_v0 + _omega0 * x0) * t);
  }

  return y;
}
@end
