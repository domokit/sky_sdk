// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/physics.dart';
import 'package:flutter_test/flutter_test.dart';

class TestSimulation extends Simulation {
  @override
  double x(double t) => 0.0;

  @override
  double dx(double t) => 0.0;

  @override
  bool isDone(double t) => true;
}

void main() {
  test('Simulation.toString', () {
    expect(
      ClampedSimulation(
        TestSimulation(),
        xMin: -1.0,
        xMax: 2.0,
        dxMin: -3.0,
        dxMax: 4.0,
      ).toString(),
      'ClampedSimulation(simulation: TestSimulation, x: -1.0..2.0, dx: -3.0..4.0)',
    );
    expect(TestSimulation().toString(), 'TestSimulation');
    expect(
      GravitySimulation(1.0, -2.0, 3.0, -4.0).toString(),
      'GravitySimulation(g: 1.0, x₀: -2.0, dx₀: -4.0, xₘₐₓ: ±3.0)',
    );
    expect(
      FrictionSimulation(1.0, -2.0, 3.0).toString(),
      'FrictionSimulation(cₓ: 1.0, x₀: -2.0, dx₀: 3.0)',
    );
    expect(
      BoundedFrictionSimulation(1.0, -2.0, 3.0, -4.0, 5.0).toString(),
      'BoundedFrictionSimulation(cₓ: 1.0, x₀: -2.0, dx₀: 3.0, x: -4.0..5.0)',
    );
    expect(
      const SpringDescription(mass: 1.0, stiffness: -2.0, damping: 3.0).toString(),
      'SpringDescription(mass: 1.0, stiffness: -2.0, damping: 3.0)',
    );
    expect(
      SpringDescription.withDampingRatio(mass: 1.0, stiffness: 9.0).toString(),
      'SpringDescription(mass: 1.0, stiffness: 9.0, damping: 6.0)',
    );
    expect(
      SpringSimulation(
        const SpringDescription(mass: 1.0, stiffness: 2.0, damping: 3.0),
        0.0,
        1.0,
        2.0,
      ).toString(),
      'SpringSimulation(end: 1.0, SpringType.overDamped)',
    );
  });
}
