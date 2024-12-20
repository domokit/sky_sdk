// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/physics.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  test('When snapToEnd is set, value is exactly `end` after completion', () {
    final SpringDescription description = SpringDescription.withDampingRatio(
      mass: 1.0,
      stiffness: 400,
    );
    const double time = 0.4;

    final SpringSimulation regularSimulation = SpringSimulation(
      description,
      0,
      1,
      0,
      tolerance: const Tolerance(distance: 0.1, velocity: 0.1),
    );
    expect(regularSimulation.x(time), lessThan(1));
    expect(regularSimulation.dx(time), greaterThan(0));

    final SpringSimulation snappingSimulation = SpringSimulation(
      description,
      0,
      1,
      0,
      snapToEnd: true,
      tolerance: const Tolerance(distance: 0.1, velocity: 0.1),
    );
    // Must be exactly equal
    expect(snappingSimulation.x(time), 1);
    expect(snappingSimulation.dx(time), 0);
  });
}
