// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/physics.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  test('Friction simulation positive velocity', () {
    final FrictionSimulation friction = FrictionSimulation(0.135, 100.0, 100.0);

    expect(friction.x(0.0), moreOrLessEquals(100.0));
    expect(friction.dx(0.0), moreOrLessEquals(100.0));

    expect(friction.x(0.1), moreOrLessEquals(110.0, epsilon: 1.0));
    expect(friction.x(0.5), moreOrLessEquals(131.0, epsilon: 1.0));
    expect(friction.x(2.0), moreOrLessEquals(149.0, epsilon: 1.0));

    expect(friction.finalX, moreOrLessEquals(149.0, epsilon: 1.0));

    expect(friction.timeAtX(100.0), 0.0);
    expect(friction.timeAtX(friction.x(0.1)), moreOrLessEquals(0.1));
    expect(friction.timeAtX(friction.x(0.5)), moreOrLessEquals(0.5));
    expect(friction.timeAtX(friction.x(2.0)), moreOrLessEquals(2.0));

    expect(friction.timeAtX(-1.0), double.infinity);
    expect(friction.timeAtX(200.0), double.infinity);
  });

  test('Friction simulation negative velocity', () {
    final FrictionSimulation friction = FrictionSimulation(0.135, 100.0, -100.0);

    expect(friction.x(0.0), moreOrLessEquals(100.0));
    expect(friction.dx(0.0), moreOrLessEquals(-100.0));

    expect(friction.x(0.1), moreOrLessEquals(91.0, epsilon: 1.0));
    expect(friction.x(0.5), moreOrLessEquals(68.0, epsilon: 1.0));
    expect(friction.x(2.0), moreOrLessEquals(51.0, epsilon: 1.0));

    expect(friction.finalX, moreOrLessEquals(50, epsilon: 1.0));

    expect(friction.timeAtX(100.0), 0.0);
    expect(friction.timeAtX(friction.x(0.1)), moreOrLessEquals(0.1));
    expect(friction.timeAtX(friction.x(0.5)), moreOrLessEquals(0.5));
    expect(friction.timeAtX(friction.x(2.0)), moreOrLessEquals(2.0));

    expect(friction.timeAtX(101.0), double.infinity);
    expect(friction.timeAtX(40.0), double.infinity);
  });

  test('Friction simulation constant deceleration', () {
    final FrictionSimulation friction = FrictionSimulation(
      0.135,
      100.0,
      -100.0,
      constantDeceleration: 100,
    );

    expect(friction.x(0.0), moreOrLessEquals(100.0));
    expect(friction.dx(0.0), moreOrLessEquals(-100.0));

    expect(friction.x(0.1), moreOrLessEquals(91.0, epsilon: 1.0));
    expect(friction.x(0.5), moreOrLessEquals(80.0, epsilon: 1.0));
    expect(friction.x(2.0), moreOrLessEquals(80.0, epsilon: 1.0));

    expect(friction.finalX, moreOrLessEquals(80.0, epsilon: 1.0));

    expect(friction.timeAtX(100.0), 0.0);
    expect(friction.timeAtX(friction.x(0.1)), moreOrLessEquals(0.1));
    expect(friction.timeAtX(friction.x(0.2)), moreOrLessEquals(0.2));
    expect(friction.timeAtX(friction.x(0.3)), moreOrLessEquals(0.3));

    expect(friction.timeAtX(101.0), double.infinity);
    expect(friction.timeAtX(40.0), double.infinity);
  });
}
