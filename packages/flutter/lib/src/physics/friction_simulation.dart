// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math' as math;

import 'package:flutter/foundation.dart';

import 'simulation.dart';

export 'tolerance.dart' show Tolerance;

/// Numerically determine the input value which produces output value [target]
/// for a function [f], given its first-derivative [df].
double _newtonsMethod({
  required double initialGuess,
  required double target,
  required double Function(double) f,
  required double Function(double) df,
  required int iterations,
}) {
  double guess = initialGuess;
  for (int i = 0; i < iterations; i++) {
    guess = guess - (f(guess) - target) / df(guess);
  }
  return guess;
}

/// A simulation that applies a drag to slow a particle down.
///
/// Models a particle affected by fluid drag, e.g. air resistance.
///
/// The simulation ends when the velocity of the particle drops to zero (within
/// the current velocity [tolerance]).
class FrictionSimulation extends Simulation {
  /// Creates a [FrictionSimulation] with the given arguments, namely: the fluid
  /// drag coefficient _cₓ_, a unitless value; the initial position _x₀_, in the same
  /// length units as used for [x]; and the initial velocity _dx₀_, in the same
  /// velocity units as used for [dx].
  FrictionSimulation(
    double drag,
    double position,
    double velocity, {
    super.tolerance,
    double constantDeceleration = 0,
  }) : _drag = drag,
       _dragLog = math.log(drag),
       _x = position,
       _v = velocity,
       _constantDeceleration = constantDeceleration * velocity.sign {
    _finalTime = _newtonsMethod(
      initialGuess: 0,
      target: 0,
      f: dx,
      df: (double time) => (_v * math.pow(_drag, time) * _dragLog) - _constantDeceleration,
      iterations: 10,
    );
  }

  /// Creates a new friction simulation with its fluid drag coefficient (_cₓ_) set so
  /// as to ensure that the simulation starts and ends at the specified
  /// positions and velocities.
  ///
  /// The positions must use the same units as expected from [x], and the
  /// velocities must use the same units as expected from [dx].
  ///
  /// The sign of the start and end velocities must be the same, the magnitude
  /// of the start velocity must be greater than the magnitude of the end
  /// velocity, and the velocities must be in the direction appropriate for the
  /// particle to start from the start position and reach the end position.
  factory FrictionSimulation.through(
    double startPosition,
    double endPosition,
    double startVelocity,
    double endVelocity,
  ) {
    assert(startVelocity == 0.0 || endVelocity == 0.0 || startVelocity.sign == endVelocity.sign);
    assert(startVelocity.abs() >= endVelocity.abs());
    assert((endPosition - startPosition).sign == startVelocity.sign);
    return FrictionSimulation(
      _dragFor(startPosition, endPosition, startVelocity, endVelocity),
      startPosition,
      startVelocity,
      tolerance: Tolerance(velocity: endVelocity.abs()),
    );
  }

  final double _drag;
  final double _dragLog;
  final double _x;
  final double _v;
  final double _constantDeceleration;
  // The time at which the simulation should be stopped.
  // This is needed when constantDeceleration is not zero (on Desktop), when
  // using the pure friction simulation, acceleration naturally reduces to zero
  // and creates a stopping point.
  double _finalTime =
      double.infinity; // needs to be infinity for newtonsMethod call in constructor.

  // Return the drag value for a FrictionSimulation whose x() and dx() values pass
  // through the specified start and end position/velocity values.
  //
  // Total time to reach endVelocity is just: (log(endVelocity) / log(startVelocity)) / log(_drag)
  // or (log(v1) - log(v0)) / log(D), given v = v0 * D^t per the dx() function below.
  // Solving for D given x(time) is trickier. Algebra courtesy of Wolfram Alpha:
  // x1 = x0 + (v0 * D^((log(v1) - log(v0)) / log(D))) / log(D) - v0 / log(D), find D
  static double _dragFor(
    double startPosition,
    double endPosition,
    double startVelocity,
    double endVelocity,
  ) {
    return math.pow(math.e, (startVelocity - endVelocity) / (startPosition - endPosition))
        as double;
  }

  @override
  double x(double time) {
    if (time > _finalTime) {
      return finalX;
    }
    return _x +
        _v * math.pow(_drag, time) / _dragLog -
        _v / _dragLog -
        ((_constantDeceleration / 2) * time * time);
  }

  @override
  double dx(double time) {
    if (time > _finalTime) {
      return 0;
    }
    return _v * math.pow(_drag, time) - _constantDeceleration * time;
  }

  /// The value of [x] at `double.infinity`.
  double get finalX {
    if (_constantDeceleration == 0) {
      return _x - _v / _dragLog;
    }
    return x(_finalTime);
  }

  /// The time at which the value of `x(time)` will equal [x].
  ///
  /// Returns `double.infinity` if the simulation will never reach [x].
  double timeAtX(double x) {
    if (x == _x) {
      return 0.0;
    }
    if (_v == 0.0 || (_v > 0 ? (x < _x || x > finalX) : (x > _x || x < finalX))) {
      return double.infinity;
    }
    return _newtonsMethod(target: x, initialGuess: 0, f: this.x, df: dx, iterations: 10);
  }

  @override
  bool isDone(double time) {
    return dx(time).abs() < tolerance.velocity;
  }

  @override
  String toString() =>
      '${objectRuntimeType(this, 'FrictionSimulation')}(cₓ: ${_drag.toStringAsFixed(1)}, x₀: ${_x.toStringAsFixed(1)}, dx₀: ${_v.toStringAsFixed(1)})';
}

/// A [FrictionSimulation] that clamps the modeled particle to a specific range
/// of values.
///
/// Only the position is clamped. The velocity [dx] will continue to report
/// unbounded simulated velocities once the particle has reached the bounds.
class BoundedFrictionSimulation extends FrictionSimulation {
  /// Creates a [BoundedFrictionSimulation] with the given arguments, namely:
  /// the fluid drag coefficient _cₓ_, a unitless value; the initial position _x₀_, in the
  /// same length units as used for [x]; the initial velocity _dx₀_, in the same
  /// velocity units as used for [dx], the minimum value for the position, and
  /// the maximum value for the position. The minimum and maximum values must be
  /// in the same units as the initial position, and the initial position must
  /// be within the given range.
  BoundedFrictionSimulation(super.drag, super.position, super.velocity, this._minX, this._maxX)
    : assert(clampDouble(position, _minX, _maxX) == position);

  final double _minX;
  final double _maxX;

  @override
  double x(double time) {
    return clampDouble(super.x(time), _minX, _maxX);
  }

  @override
  bool isDone(double time) {
    return super.isDone(time) ||
        (x(time) - _minX).abs() < tolerance.distance ||
        (x(time) - _maxX).abs() < tolerance.distance;
  }

  @override
  String toString() =>
      '${objectRuntimeType(this, 'BoundedFrictionSimulation')}(cₓ: ${_drag.toStringAsFixed(1)}, x₀: ${_x.toStringAsFixed(1)}, dx₀: ${_v.toStringAsFixed(1)}, x: ${_minX.toStringAsFixed(1)}..${_maxX.toStringAsFixed(1)})';
}
