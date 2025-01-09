// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/widgets.dart';
library;

import 'dart:math' as math;

import 'package:flutter/foundation.dart';

import 'simulation.dart';
import 'utils.dart';

export 'tolerance.dart' show Tolerance;

// Examples can assume:
// late AnimationController _controller;

/// Structure that describes a spring's constants.
///
/// Used to configure a [SpringSimulation].
class SpringDescription {
  /// Creates a spring given the mass, stiffness, and the damping coefficient.
  ///
  /// See [mass], [stiffness], and [damping] for the units of the arguments.
  const SpringDescription({required this.mass, required this.stiffness, required this.damping});

  /// Creates a spring given the mass (m), stiffness (k), and damping ratio (ζ).
  /// The damping ratio describes a gradual reduction in a spring oscillation.
  /// By using the damping ratio, you can define how rapidly the oscillations
  /// decay from one bounce to the next.
  ///
  /// The damping ratio is especially useful when trying to determining the type
  /// of spring to create. A ratio of 1.0 creates a critically damped
  /// spring, > 1.0 creates an overdamped spring and < 1.0 an underdamped one.
  ///
  /// See [mass] and [stiffness] for the units for those arguments. The damping
  /// ratio is unitless.
  SpringDescription.withDampingRatio({
    required this.mass,
    required this.stiffness,
    double ratio = 1.0,
  }) : damping = ratio * 2.0 * math.sqrt(mass * stiffness);

  /// The mass of the spring (m).
  ///
  /// The units are arbitrary, but all springs within a system should use
  /// the same mass units.
  ///
  /// The greater the mass, the larger the amplitude of oscillation,
  /// and the longer the time to return to the equilibrium position.
  final double mass;

  /// The spring constant (k).
  ///
  /// The units of stiffness are M/T², where M is the mass unit used for the
  /// value of the [mass] property, and T is the time unit used for driving
  /// the [SpringSimulation].
  ///
  /// Stiffness defines the spring constant, which measures the strength of
  /// the spring. A stiff spring applies more force to the object that is
  /// attached for some deviation from the rest position.
  final double stiffness;

  /// The damping coefficient (c).
  ///
  /// It is a pure number without physical meaning and describes the oscillation
  /// and decay of a system after being disturbed. The larger the damping,
  /// the fewer oscillations and smaller the amplitude of the elastic motion.
  ///
  /// Do not confuse the damping _coefficient_ (c) with the damping _ratio_ (ζ).
  /// To create a [SpringDescription] with a damping ratio, use the [
  /// SpringDescription.withDampingRatio] constructor.
  ///
  /// The units of the damping coefficient are M/T, where M is the mass unit
  /// used for the value of the [mass] property, and T is the time unit used for
  /// driving the [SpringSimulation].
  final double damping;

  @override
  String toString() =>
      '${objectRuntimeType(this, 'SpringDescription')}(mass: ${mass.toStringAsFixed(1)}, stiffness: ${stiffness.toStringAsFixed(1)}, damping: ${damping.toStringAsFixed(1)})';
}

/// The kind of spring solution that the [SpringSimulation] is using to simulate the spring.
///
/// See [SpringSimulation.type].
enum SpringType {
  /// A spring that does not bounce and returns to its rest position in the
  /// shortest possible time.
  criticallyDamped,

  /// A spring that bounces.
  underDamped,

  /// A spring that does not bounce but takes longer to return to its rest
  /// position than a [criticallyDamped] one.
  overDamped,
}

/// A spring simulation.
///
/// Models a particle attached to a spring that follows Hooke's law.
///
/// {@tool snippet}
///
/// This method triggers an [AnimationController] (a previously constructed
/// `_controller` field) to simulate a spring oscillation.
///
/// ```dart
/// void _startSpringMotion() {
///   _controller.animateWith(SpringSimulation(
///     const SpringDescription(
///       mass: 1.0,
///       stiffness: 300.0,
///       damping: 15.0,
///     ),
///     0.0, // starting position
///     1.0, // ending position
///     0.0, // starting velocity
///   ));
/// }
/// ```
/// {@end-tool}
///
/// This [AnimationController] could be used with an [AnimatedBuilder] to
/// animate the position of a child as if it were attached to a spring.
class SpringSimulation extends Simulation {
  /// Creates a spring simulation from the provided spring description, start
  /// distance, end distance, and initial velocity.
  ///
  /// The units for the start and end distance arguments are arbitrary, but must
  /// be consistent with the units used for other lengths in the system.
  ///
  /// The units for the velocity are L/T, where L is the aforementioned
  /// arbitrary unit of length, and T is the time unit used for driving the
  /// [SpringSimulation].
  ///
  /// If `snapToEnd` is true, [x] will be set to `end` and [dx] to 0 when
  /// [isDone] returns true. This is useful for transitions that require the
  /// simulation to stop exactly at the end value, since the spring may not
  /// naturally reach the target precisely. Defaults to false.
  SpringSimulation(
    SpringDescription spring,
    double start,
    double end,
    double velocity, {
    bool snapToEnd = false,
    super.tolerance,
  }) : _endPosition = end,
       _solution = _SpringSolution(spring, start - end, velocity),
       _snapToEnd = snapToEnd;

  final double _endPosition;
  final _SpringSolution _solution;
  final bool _snapToEnd;

  /// The kind of spring being simulated, for debugging purposes.
  ///
  /// This is derived from the [SpringDescription] provided to the [
  /// SpringSimulation] constructor.
  SpringType get type => _solution.type;

  @override
  double x(double time) {
    if (_snapToEnd && isDone(time)) {
      return _endPosition;
    } else {
      return _endPosition + _solution.x(time);
    }
  }

  @override
  double dx(double time) {
    if (_snapToEnd && isDone(time)) {
      return 0;
    } else {
      return _solution.dx(time);
    }
  }

  @override
  bool isDone(double time) {
    return nearZero(_solution.x(time), tolerance.distance) &&
        nearZero(_solution.dx(time), tolerance.velocity);
  }

  @override
  String toString() =>
      '${objectRuntimeType(this, 'SpringSimulation')}(end: ${_endPosition.toStringAsFixed(1)}, $type)';
}

/// A [SpringSimulation] where the value of [x] is guaranteed to have exactly the
/// end value when the simulation [isDone].
class ScrollSpringSimulation extends SpringSimulation {
  /// Creates a spring simulation from the provided spring description, start
  /// distance, end distance, and initial velocity.
  ///
  /// See the [SpringSimulation.new] constructor on the superclass for a
  /// discussion of the arguments' units.
  ScrollSpringSimulation(super.spring, super.start, super.end, super.velocity, {super.tolerance});

  @override
  double x(double time) => isDone(time) ? _endPosition : super.x(time);
}

// SPRING IMPLEMENTATIONS

abstract class _SpringSolution {
  factory _SpringSolution(
    SpringDescription spring,
    double initialPosition,
    double initialVelocity,
  ) {
    return switch (spring.damping * spring.damping - 4 * spring.mass * spring.stiffness) {
      > 0.0 => _OverdampedSolution(spring, initialPosition, initialVelocity),
      < 0.0 => _UnderdampedSolution(spring, initialPosition, initialVelocity),
      _ => _CriticalSolution(spring, initialPosition, initialVelocity),
    };
  }

  double x(double time);
  double dx(double time);
  SpringType get type;
}

class _CriticalSolution implements _SpringSolution {
  factory _CriticalSolution(SpringDescription spring, double distance, double velocity) {
    final double r = -spring.damping / (2.0 * spring.mass);
    final double c1 = distance;
    final double c2 = velocity - (r * distance);
    return _CriticalSolution.withArgs(r, c1, c2);
  }

  _CriticalSolution.withArgs(double r, double c1, double c2) : _r = r, _c1 = c1, _c2 = c2;

  final double _r, _c1, _c2;

  @override
  double x(double time) {
    return (_c1 + _c2 * time) * math.pow(math.e, _r * time);
  }

  @override
  double dx(double time) {
    final double power = math.pow(math.e, _r * time) as double;
    return _r * (_c1 + _c2 * time) * power + _c2 * power;
  }

  @override
  SpringType get type => SpringType.criticallyDamped;
}

class _OverdampedSolution implements _SpringSolution {
  factory _OverdampedSolution(SpringDescription spring, double distance, double velocity) {
    final double cmk = spring.damping * spring.damping - 4 * spring.mass * spring.stiffness;
    final double r1 = (-spring.damping - math.sqrt(cmk)) / (2.0 * spring.mass);
    final double r2 = (-spring.damping + math.sqrt(cmk)) / (2.0 * spring.mass);
    final double c2 = (velocity - r1 * distance) / (r2 - r1);
    final double c1 = distance - c2;
    return _OverdampedSolution.withArgs(r1, r2, c1, c2);
  }

  _OverdampedSolution.withArgs(double r1, double r2, double c1, double c2)
    : _r1 = r1,
      _r2 = r2,
      _c1 = c1,
      _c2 = c2;

  final double _r1, _r2, _c1, _c2;

  @override
  double x(double time) {
    return _c1 * math.pow(math.e, _r1 * time) + _c2 * math.pow(math.e, _r2 * time);
  }

  @override
  double dx(double time) {
    return _c1 * _r1 * math.pow(math.e, _r1 * time) + _c2 * _r2 * math.pow(math.e, _r2 * time);
  }

  @override
  SpringType get type => SpringType.overDamped;
}

class _UnderdampedSolution implements _SpringSolution {
  factory _UnderdampedSolution(SpringDescription spring, double distance, double velocity) {
    final double w =
        math.sqrt(4.0 * spring.mass * spring.stiffness - spring.damping * spring.damping) /
        (2.0 * spring.mass);
    final double r = -(spring.damping / 2.0 * spring.mass);
    final double c1 = distance;
    final double c2 = (velocity - r * distance) / w;
    return _UnderdampedSolution.withArgs(w, r, c1, c2);
  }

  _UnderdampedSolution.withArgs(double w, double r, double c1, double c2)
    : _w = w,
      _r = r,
      _c1 = c1,
      _c2 = c2;

  final double _w, _r, _c1, _c2;

  @override
  double x(double time) {
    return (math.pow(math.e, _r * time) as double) *
        (_c1 * math.cos(_w * time) + _c2 * math.sin(_w * time));
  }

  @override
  double dx(double time) {
    final double power = math.pow(math.e, _r * time) as double;
    final double cosine = math.cos(_w * time);
    final double sine = math.sin(_w * time);
    return power * (_c2 * _w * cosine - _c1 * _w * sine) + _r * power * (_c2 * sine + _c1 * cosine);
  }

  @override
  SpringType get type => SpringType.underDamped;
}
