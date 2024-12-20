// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';

/// Structure that specifies maximum allowable magnitudes for distances,
/// durations, and velocity differences to be considered equal.
class Tolerance {
  /// Creates a [Tolerance] object. By default, the distance, time, and velocity
  /// tolerances are all ±0.001; the constructor arguments override this.
  ///
  /// The arguments should all be positive values.
  const Tolerance({
    this.distance = _epsilonDefault,
    this.time = _epsilonDefault,
    this.velocity = _epsilonDefault,
  });

  static const double _epsilonDefault = 1e-3;

  /// A default tolerance of 0.001 for all three values.
  static const Tolerance defaultTolerance = Tolerance();

  /// The magnitude of the maximum distance between two points for them to be
  /// considered within tolerance.
  ///
  /// The units for the distance tolerance must be the same as the units used
  /// for the distances that are to be compared to this tolerance.
  final double distance;

  /// The magnitude of the maximum duration between two times for them to be
  /// considered within tolerance.
  ///
  /// The units for the time tolerance must be the same as the units used
  /// for the times that are to be compared to this tolerance.
  final double time;

  /// The magnitude of the maximum difference between two velocities for them to
  /// be considered within tolerance.
  ///
  /// The units for the velocity tolerance must be the same as the units used
  /// for the velocities that are to be compared to this tolerance.
  final double velocity;

  @override
  String toString() =>
      '${objectRuntimeType(this, 'Tolerance')}(distance: ±$distance, time: ±$time, velocity: ±$velocity)';
}
