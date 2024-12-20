// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math';
import 'package:flutter/material.dart';
import 'package:vector_math/vector_math.dart' show Vector2;

// Provides calculations for an object moving with inertia and friction using
// the equation of motion from physics.
// https://en.wikipedia.org/wiki/Equations_of_motion#Constant_translational_acceleration_in_a_straight_line
@immutable
class InertialMotion {
  const InertialMotion(this._initialVelocity, this._initialPosition);

  static const double _kFrictionalAcceleration = 0.01; // How quickly to stop
  final Velocity _initialVelocity;
  final Offset _initialPosition;

  // The position when the motion stops.
  Offset get finalPosition {
    return _getPositionAt(Duration(milliseconds: duration.toInt()));
  }

  // The total time that the animation takes start to stop in milliseconds.
  double get duration {
    return (_initialVelocity.pixelsPerSecond.dx / 1000 / _acceleration.x).abs();
  }

  // The acceleration opposing the initial velocity in x and y components.
  Vector2 get _acceleration {
    final double velocityTotal =
        _initialVelocity.pixelsPerSecond.dx.abs() + _initialVelocity.pixelsPerSecond.dy.abs();
    final double vRatioX = _initialVelocity.pixelsPerSecond.dx / velocityTotal;
    final double vRatioY = _initialVelocity.pixelsPerSecond.dy / velocityTotal;
    return Vector2(_kFrictionalAcceleration * vRatioX, _kFrictionalAcceleration * vRatioY);
  }

  // The position at a given time.
  Offset _getPositionAt(Duration time) {
    final double xf = _getPosition(
      r0: _initialPosition.dx,
      v0: _initialVelocity.pixelsPerSecond.dx / 1000,
      t: time.inMilliseconds,
      a: _acceleration.x,
    );
    final double yf = _getPosition(
      r0: _initialPosition.dy,
      v0: _initialVelocity.pixelsPerSecond.dy / 1000,
      t: time.inMilliseconds,
      a: _acceleration.y,
    );
    return Offset(xf, yf);
  }

  // Solve the equation of motion to find the position at a given point in time
  // in one dimension.
  double _getPosition({required double r0, required double v0, required int t, required double a}) {
    // Stop movement when it would otherwise reverse direction.
    final double stopTime = (v0 / a).abs();
    if (t > stopTime) {
      t = stopTime.toInt();
    }

    return r0 + v0 * t + 0.5 * a * pow(t, 2);
  }
}
