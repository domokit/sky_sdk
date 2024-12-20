// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';

import 'tolerance.dart';

export 'tolerance.dart' show Tolerance;

/// The base class for all simulations.
///
/// A simulation models an object, in a one-dimensional space, on which particular
/// forces are being applied, and exposes:
///
///  * The object's position, [x]
///  * The object's velocity, [dx]
///  * Whether the simulation is "done", [isDone]
///
/// A simulation is generally "done" if the object has, to a given [tolerance],
/// come to a complete rest.
///
/// The [x], [dx], and [isDone] functions take a time argument which specifies
/// the time for which they are to be evaluated. In principle, simulations can
/// be stateless, and thus can be queried with arbitrary times. In practice,
/// however, some simulations are not, and calling any of these functions will
/// advance the simulation to the given time.
///
/// As a general rule, therefore, a simulation should only be queried using
/// times that are equal to or greater than all times previously used for that
/// simulation.
///
/// Simulations do not specify units for distance, velocity, and time. Client
/// should establish a convention and use that convention consistently with all
/// related objects.
abstract class Simulation {
  /// Initializes the [tolerance] field for subclasses.
  Simulation({this.tolerance = Tolerance.defaultTolerance});

  /// The position of the object in the simulation at the given time.
  double x(double time);

  /// The velocity of the object in the simulation at the given time.
  double dx(double time);

  /// Whether the simulation is "done" at the given time.
  bool isDone(double time);

  /// How close to the actual end of the simulation a value at a particular time
  /// must be before [isDone] considers the simulation to be "done".
  ///
  /// A simulation with an asymptotic curve would never technically be "done",
  /// but once the difference from the value at a particular time and the
  /// asymptote itself could not be seen, it would be pointless to continue. The
  /// tolerance defines how to determine if the difference could not be seen.
  Tolerance tolerance;

  @override
  String toString() => objectRuntimeType(this, 'Simulation');
}
