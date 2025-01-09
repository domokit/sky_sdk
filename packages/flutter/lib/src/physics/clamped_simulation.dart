// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';

import 'simulation.dart';

export 'simulation.dart' show Simulation;

/// A simulation that applies limits to another simulation.
///
/// The limits are only applied to the other simulation's outputs. For example,
/// if a maximum position was applied to a gravity simulation with the
/// particle's initial velocity being up, and the acceleration being down, and
/// the maximum position being between the initial position and the curve's
/// apogee, then the particle would return to its initial position in the same
/// amount of time as it would have if the maximum had not been applied; the
/// difference would just be that the position would be reported as pinned to
/// the maximum value for the times that it would otherwise have been reported
/// as higher.
///
/// Similarly, this means that the [x] value will change at a rate that does not
/// match the reported [dx] value while one or the other is being clamped.
///
/// The [isDone] logic is unaffected by the clamping; it reflects the logic of
/// the underlying simulation.
class ClampedSimulation extends Simulation {
  /// Creates a [ClampedSimulation] that clamps the given simulation.
  ///
  /// The named arguments specify the ranges for the clamping behavior, as
  /// applied to [x] and [dx].
  ClampedSimulation(
    this.simulation, {
    this.xMin = double.negativeInfinity,
    this.xMax = double.infinity,
    this.dxMin = double.negativeInfinity,
    this.dxMax = double.infinity,
  }) : assert(xMax >= xMin),
       assert(dxMax >= dxMin);

  /// The simulation being clamped. Calls to [x], [dx], and [isDone] are
  /// forwarded to the simulation.
  final Simulation simulation;

  /// The minimum to apply to [x].
  final double xMin;

  /// The maximum to apply to [x].
  final double xMax;

  /// The minimum to apply to [dx].
  final double dxMin;

  /// The maximum to apply to [dx].
  final double dxMax;

  @override
  double x(double time) => clampDouble(simulation.x(time), xMin, xMax);

  @override
  double dx(double time) => clampDouble(simulation.dx(time), dxMin, dxMax);

  @override
  bool isDone(double time) => simulation.isDone(time);

  @override
  String toString() =>
      '${objectRuntimeType(this, 'ClampedSimulation')}(simulation: $simulation, x: ${xMin.toStringAsFixed(1)}..${xMax.toStringAsFixed(1)}, dx: ${dxMin.toStringAsFixed(1)}..${dxMax.toStringAsFixed(1)})';
}
