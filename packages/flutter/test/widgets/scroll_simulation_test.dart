// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math' as math;

import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  test('ClampingScrollSimulation has a stable initial conditions', () {
    void checkInitialConditions(double position, double velocity) {
      final ClampingScrollSimulation simulation = ClampingScrollSimulation(
        position: position,
        velocity: velocity,
      );
      expect(simulation.x(0.0), moreOrLessEquals(position));
      expect(simulation.dx(0.0), moreOrLessEquals(velocity));
    }

    checkInitialConditions(51.0, 2866.91537);
    checkInitialConditions(584.0, 2617.294734);
    checkInitialConditions(345.0, 1982.785934);
    checkInitialConditions(0.0, 1831.366634);
    checkInitialConditions(-156.2, 1541.57665);
    checkInitialConditions(4.0, 1139.250439);
    checkInitialConditions(4534.0, 1073.553798);
    checkInitialConditions(75.0, 614.2093);
    checkInitialConditions(5469.0, 182.114534);
  });

  test('ClampingScrollSimulation only decelerates, never speeds up', () {
    // Regression test for https://github.com/flutter/flutter/issues/113424
    final ClampingScrollSimulation simulation = ClampingScrollSimulation(
      position: 0,
      velocity: 8000.0,
    );
    double time = 0.0;
    double velocity = simulation.dx(time);
    while (!simulation.isDone(time)) {
      expect(time, lessThan(3.0));
      time += 1 / 60;
      final double nextVelocity = simulation.dx(time);
      expect(nextVelocity, lessThanOrEqualTo(velocity));
      velocity = nextVelocity;
    }
  });

  test(
    'ClampingScrollSimulation reaches a smooth stop: velocity is continuous and goes to zero',
    () {
      // Regression test for https://github.com/flutter/flutter/issues/113424
      const double initialVelocity = 8000.0;
      const double maxDeceleration = 5130.0; // -acceleration(initialVelocity), from formula below
      final ClampingScrollSimulation simulation = ClampingScrollSimulation(
        position: 0,
        velocity: initialVelocity,
      );

      double time = 0.0;
      double velocity = simulation.dx(time);
      const double delta = 1 / 60;
      do {
        expect(time, lessThan(3.0));
        time += delta;
        final double nextVelocity = simulation.dx(time);
        expect((nextVelocity - velocity).abs(), lessThan(delta * maxDeceleration));
        velocity = nextVelocity;
      } while (!simulation.isDone(time));
      expect(velocity, moreOrLessEquals(0.0));
    },
  );

  test('ClampingScrollSimulation is ballistic', () {
    // Regression test for https://github.com/flutter/flutter/issues/120338
    const double delta = 1 / 90;
    final ClampingScrollSimulation undisturbed = ClampingScrollSimulation(
      position: 0,
      velocity: 8000.0,
    );

    double time = 0.0;
    ClampingScrollSimulation restarted = undisturbed;
    final List<double> xsRestarted = <double>[];
    final List<double> xsUndisturbed = <double>[];
    final List<double> dxsRestarted = <double>[];
    final List<double> dxsUndisturbed = <double>[];
    do {
      expect(time, lessThan(4.0));
      time += delta;
      restarted = ClampingScrollSimulation(
        position: restarted.x(delta),
        velocity: restarted.dx(delta),
      );
      xsRestarted.add(restarted.x(0));
      xsUndisturbed.add(undisturbed.x(time));
      dxsRestarted.add(restarted.dx(0));
      dxsUndisturbed.add(undisturbed.dx(time));
    } while (!restarted.isDone(0) || !undisturbed.isDone(time));

    // Compare the headline number first: the total distances traveled.
    // This way, if the test fails, it shows the big final difference
    // instead of the tiny difference that's in the very first frame.
    expect(xsRestarted.last, moreOrLessEquals(xsUndisturbed.last));

    // The whole trajectories along the way should match too.
    for (int i = 0; i < xsRestarted.length; i++) {
      expect(xsRestarted[i], moreOrLessEquals(xsUndisturbed[i]));
      expect(dxsRestarted[i], moreOrLessEquals(dxsUndisturbed[i]));
    }
  });

  test('ClampingScrollSimulation satisfies a physical acceleration formula', () {
    // Different regression test for https://github.com/flutter/flutter/issues/120338
    //
    // This one provides a formula for the particle's acceleration as a function
    // of its velocity, and checks that it behaves according to that formula.
    // The point isn't that it's this specific formula, but just that there's
    // some formula which depends only on velocity, not time, so that the
    // physical metaphor makes sense.

    // Copied from the implementation.
    final double kDecelerationRate = math.log(0.78) / math.log(0.9);

    // Same as the referenceVelocity in _flingDuration.
    const double referenceVelocity = .015 * 9.80665 * 39.37 * 160.0 * 0.84 / 0.35;

    // The value of _duration when velocity == referenceVelocity.
    final double referenceDuration = kDecelerationRate * 0.35;

    // The rate of deceleration when dx(time) == referenceVelocity.
    final double referenceDeceleration =
        (kDecelerationRate - 1) * referenceVelocity / referenceDuration;

    double acceleration(double velocity) {
      return -velocity.sign *
          referenceDeceleration *
          math.pow(
            velocity.abs() / referenceVelocity,
            (kDecelerationRate - 2) / (kDecelerationRate - 1),
          );
    }

    double jerk(double velocity) {
      return referenceVelocity /
          referenceDuration /
          referenceDuration *
          (kDecelerationRate - 1) *
          (kDecelerationRate - 2) *
          math.pow(
            velocity.abs() / referenceVelocity,
            (kDecelerationRate - 3) / (kDecelerationRate - 1),
          );
    }

    void checkAcceleration(double position, double velocity) {
      final ClampingScrollSimulation simulation = ClampingScrollSimulation(
        position: position,
        velocity: velocity,
      );
      double time = 0.0;
      const double delta = 1 / 60;
      for (; time < 2.0; time += delta) {
        final double difference = simulation.dx(time + delta) - simulation.dx(time);
        final double predictedDifference = delta * acceleration(simulation.dx(time + delta / 2));
        final double maxThirdDerivative = jerk(simulation.dx(time + delta));
        expect(
          (difference - predictedDifference).abs(),
          lessThan(maxThirdDerivative * math.pow(delta, 2) / 2),
        );
      }
    }

    checkAcceleration(51.0, 2866.91537);
    checkAcceleration(584.0, 2617.294734);
    checkAcceleration(345.0, 1982.785934);
    checkAcceleration(0.0, 1831.366634);
    checkAcceleration(-156.2, 1541.57665);
    checkAcceleration(4.0, 1139.250439);
    checkAcceleration(4534.0, 1073.553798);
    checkAcceleration(75.0, 614.2093);
    checkAcceleration(5469.0, 182.114534);
  });
}
