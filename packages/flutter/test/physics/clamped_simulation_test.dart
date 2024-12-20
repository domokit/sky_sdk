// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/physics.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  test('Clamped simulation 1', () {
    final GravitySimulation gravity = GravitySimulation(9.81, 10.0, 0.0, 0.0);
    final ClampedSimulation clamped = ClampedSimulation(
      gravity,
      xMin: 20.0,
      xMax: 100.0,
      dxMin: 7.0,
      dxMax: 11.0,
    );

    expect(clamped.x(0.0), equals(20.0));
    expect(clamped.dx(0.0), equals(7.0));

    expect(clamped.x(100.0), equals(100.0));
    expect(clamped.dx(100.0), equals(11.0));
  });

  test('Clamped simulation 2', () {
    final GravitySimulation gravity = GravitySimulation(-10, 0.0, 6.0, 10.0);
    final ClampedSimulation clamped = ClampedSimulation(
      gravity,
      xMin: 0.0,
      xMax: 2.5,
      dxMin: -1.0,
      dxMax: 1.0,
    );

    expect(clamped.x(0.0), equals(0.0));
    expect(clamped.dx(0.0), equals(1.0));
    expect(clamped.isDone(0.0), isFalse);

    expect(clamped.x(1.0), equals(2.5));
    expect(clamped.dx(1.0), equals(0.0));
    expect(clamped.isDone(0.2), isFalse);

    expect(clamped.x(2.0), equals(0.0));
    expect(clamped.dx(2.0), equals(-1.0));
    expect(clamped.isDone(2.0), isFalse);

    expect(clamped.x(3.0), equals(0.0));
    expect(clamped.dx(3.0), equals(-1.0));
    expect(clamped.isDone(3.0), isTrue);
  });
}
