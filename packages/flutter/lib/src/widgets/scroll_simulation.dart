// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'scroll_activity.dart';
library;

import 'dart:math' as math;

import 'package:flutter/foundation.dart';
import 'package:flutter/physics.dart';

/// An implementation of scroll physics that matches iOS.
///
/// See also:
///
///  * [ClampingScrollSimulation], which implements Android scroll physics.
class BouncingScrollSimulation extends Simulation {
  /// Creates a simulation group for scrolling on iOS, with the given
  /// parameters.
  ///
  /// The position and velocity arguments must use the same units as will be
  /// expected from the [x] and [dx] methods respectively (typically logical
  /// pixels and logical pixels per second respectively).
  ///
  /// The leading and trailing extents must use the unit of length, the same
  /// unit as used for the position argument and as expected from the [x]
  /// method (typically logical pixels).
  ///
  /// The units used with the provided [SpringDescription] must similarly be
  /// consistent with the other arguments. A default set of constants is used
  /// for the `spring` description if it is omitted; these defaults assume
  /// that the unit of length is the logical pixel.
  BouncingScrollSimulation({
    required double position,
    required double velocity,
    required this.leadingExtent,
    required this.trailingExtent,
    required this.spring,
    double constantDeceleration = 0,
    super.tolerance,
  }) : assert(leadingExtent <= trailingExtent) {
    if (position < leadingExtent) {
      _springSimulation = _underscrollSimulation(position, velocity);
      _springTime = double.negativeInfinity;
    } else if (position > trailingExtent) {
      _springSimulation = _overscrollSimulation(position, velocity);
      _springTime = double.negativeInfinity;
    } else {
      // Taken from UIScrollView.decelerationRate (.normal = 0.998)
      // 0.998^1000 = ~0.135
      _frictionSimulation = FrictionSimulation(
        0.135,
        position,
        velocity,
        constantDeceleration: constantDeceleration,
      );
      final double finalX = _frictionSimulation.finalX;
      if (velocity > 0.0 && finalX > trailingExtent) {
        _springTime = _frictionSimulation.timeAtX(trailingExtent);
        _springSimulation = _overscrollSimulation(
          trailingExtent,
          math.min(_frictionSimulation.dx(_springTime), maxSpringTransferVelocity),
        );
        assert(_springTime.isFinite);
      } else if (velocity < 0.0 && finalX < leadingExtent) {
        _springTime = _frictionSimulation.timeAtX(leadingExtent);
        _springSimulation = _underscrollSimulation(
          leadingExtent,
          math.min(_frictionSimulation.dx(_springTime), maxSpringTransferVelocity),
        );
        assert(_springTime.isFinite);
      } else {
        _springTime = double.infinity;
      }
    }
  }

  /// The maximum velocity that can be transferred from the inertia of a ballistic
  /// scroll into overscroll.
  static const double maxSpringTransferVelocity = 5000.0;

  /// When [x] falls below this value the simulation switches from an internal friction
  /// model to a spring model which causes [x] to "spring" back to [leadingExtent].
  final double leadingExtent;

  /// When [x] exceeds this value the simulation switches from an internal friction
  /// model to a spring model which causes [x] to "spring" back to [trailingExtent].
  final double trailingExtent;

  /// The spring used to return [x] to either [leadingExtent] or [trailingExtent].
  final SpringDescription spring;

  late FrictionSimulation _frictionSimulation;
  late Simulation _springSimulation;
  late double _springTime;
  double _timeOffset = 0.0;

  Simulation _underscrollSimulation(double x, double dx) {
    return ScrollSpringSimulation(spring, x, leadingExtent, dx);
  }

  Simulation _overscrollSimulation(double x, double dx) {
    return ScrollSpringSimulation(spring, x, trailingExtent, dx);
  }

  Simulation _simulation(double time) {
    final Simulation simulation;
    if (time > _springTime) {
      _timeOffset = _springTime.isFinite ? _springTime : 0.0;
      simulation = _springSimulation;
    } else {
      _timeOffset = 0.0;
      simulation = _frictionSimulation;
    }
    return simulation..tolerance = tolerance;
  }

  @override
  double x(double time) => _simulation(time).x(time - _timeOffset);

  @override
  double dx(double time) => _simulation(time).dx(time - _timeOffset);

  @override
  bool isDone(double time) => _simulation(time).isDone(time - _timeOffset);

  @override
  String toString() {
    return '${objectRuntimeType(this, 'BouncingScrollSimulation')}(leadingExtent: $leadingExtent, trailingExtent: $trailingExtent)';
  }
}

/// An implementation of scroll physics that aligns with Android.
///
/// For any value of [velocity], this travels the same total distance as the
/// Android scroll physics.
///
/// This scroll physics has been adjusted relative to Android's in order to make
/// it ballistic, meaning that the deceleration at any moment is a function only
/// of the current velocity [dx] and does not depend on how long ago the
/// simulation was started.  (This is required by Flutter's scrolling protocol,
/// where [ScrollActivityDelegate.goBallistic] may restart a scroll activity
/// using only its current velocity and the scroll position's own state.)
/// Compared to this scroll physics, Android's moves faster at the very
/// beginning, then slower, and it ends at the same place but a little later.
///
/// Times are measured in seconds, and positions in logical pixels.
///
/// See also:
///
///  * [BouncingScrollSimulation], which implements iOS scroll physics.
//
// This class is based on OverScroller.java from Android:
//   https://android.googlesource.com/platform/frameworks/base/+/android-13.0.0_r24/core/java/android/widget/OverScroller.java#738
// and in particular class SplineOverScroller (at the end of the file), starting
// at method "fling".  (A very similar algorithm is in Scroller.java in the same
// directory, but OverScroller is what's used by RecyclerView.)
//
// In the Android implementation, times are in milliseconds, positions are in
// physical pixels, but velocity is in physical pixels per whole second.
//
// The "See..." comments below refer to SplineOverScroller methods and values.
class ClampingScrollSimulation extends Simulation {
  /// Creates a scroll physics simulation that aligns with Android scrolling.
  ClampingScrollSimulation({
    required this.position,
    required this.velocity,
    this.friction = 0.015,
    super.tolerance,
  }) {
    _duration = _flingDuration();
    _distance = _flingDistance();
  }

  /// The position of the particle at the beginning of the simulation, in
  /// logical pixels.
  final double position;

  /// The velocity at which the particle is traveling at the beginning of the
  /// simulation, in logical pixels per second.
  final double velocity;

  /// The amount of friction the particle experiences as it travels.
  ///
  /// The more friction the particle experiences, the sooner it stops and the
  /// less far it travels.
  ///
  /// The default value causes the particle to travel the same total distance
  /// as in the Android scroll physics.
  // See mFlingFriction.
  final double friction;

  /// The total time the simulation will run, in seconds.
  late double _duration;

  /// The total, signed, distance the simulation will travel, in logical pixels.
  late double _distance;

  // See DECELERATION_RATE.
  static final double _kDecelerationRate = math.log(0.78) / math.log(0.9);

  // See INFLEXION.
  static const double _kInflexion = 0.35;

  // See mPhysicalCoeff.  This has a value of 0.84 times Earth gravity,
  // expressed in units of logical pixels per second^2.
  static const double _physicalCoeff =
      9.80665 // g, in meters per second^2
      *
      39.37 // 1 meter / 1 inch
      *
      160.0 // 1 inch / 1 logical pixel
      *
      0.84; // "look and feel tuning"

  // See getSplineFlingDuration().
  double _flingDuration() {
    // See getSplineDeceleration().  That function's value is
    // math.log(velocity.abs() / referenceVelocity).
    final double referenceVelocity = friction * _physicalCoeff / _kInflexion;

    // This is the value getSplineFlingDuration() would return, but in seconds.
    final double androidDuration =
        math.pow(velocity.abs() / referenceVelocity, 1 / (_kDecelerationRate - 1.0)) as double;

    // We finish a bit sooner than Android, in order to travel the
    // same total distance.
    return _kDecelerationRate * _kInflexion * androidDuration;
  }

  // See getSplineFlingDistance().  This returns the same value but with the
  // sign of [velocity], and in logical pixels.
  double _flingDistance() {
    final double distance = velocity * _duration / _kDecelerationRate;
    assert(() {
      // This is the more complicated calculation that getSplineFlingDistance()
      // actually performs, which boils down to the much simpler formula above.
      final double referenceVelocity = friction * _physicalCoeff / _kInflexion;
      final double logVelocity = math.log(velocity.abs() / referenceVelocity);
      final double distanceAgain =
          friction *
          _physicalCoeff *
          math.exp(logVelocity * _kDecelerationRate / (_kDecelerationRate - 1.0));
      return (distance.abs() - distanceAgain).abs() < tolerance.distance;
    }());
    return distance;
  }

  @override
  double x(double time) {
    final double t = clampDouble(time / _duration, 0.0, 1.0);
    return position + _distance * (1.0 - math.pow(1.0 - t, _kDecelerationRate));
  }

  @override
  double dx(double time) {
    final double t = clampDouble(time / _duration, 0.0, 1.0);
    return velocity * math.pow(1.0 - t, _kDecelerationRate - 1.0);
  }

  @override
  bool isDone(double time) {
    return time >= _duration;
  }
}
