// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/scheduler.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Does not animate if already at target position', (WidgetTester tester) async {
    final ScrollController controller = ScrollController();
    addTearDown(controller.dispose);
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: ListView(
          controller: controller,
          children: List<Widget>.generate(
            80,
            (int i) => Text('$i', textDirection: TextDirection.ltr),
          ),
        ),
      ),
    );

    expectNoAnimation();
    final double currentPosition = controller.position.pixels;
    controller.position.animateTo(
      currentPosition,
      duration: const Duration(seconds: 10),
      curve: Curves.linear,
    );

    expectNoAnimation();
    expect(controller.position.pixels, currentPosition);
  });

  testWidgets('Does not animate if already at target position within tolerance', (
    WidgetTester tester,
  ) async {
    final ScrollController controller = ScrollController();
    addTearDown(controller.dispose);
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: ListView(
          controller: controller,
          children: List<Widget>.generate(
            80,
            (int i) => Text('$i', textDirection: TextDirection.ltr),
          ),
        ),
      ),
    );

    expectNoAnimation();

    final double halfTolerance =
        controller.position.physics.toleranceFor(controller.position).distance / 2;
    expect(halfTolerance, isNonZero);
    final double targetPosition = controller.position.pixels + halfTolerance;
    controller.position.animateTo(
      targetPosition,
      duration: const Duration(seconds: 10),
      curve: Curves.linear,
    );

    expectNoAnimation();
    expect(controller.position.pixels, targetPosition);
  });

  testWidgets('Animates if going to a position outside of tolerance', (WidgetTester tester) async {
    final ScrollController controller = ScrollController();
    addTearDown(controller.dispose);
    await tester.pumpWidget(
      Directionality(
        textDirection: TextDirection.ltr,
        child: ListView(
          controller: controller,
          children: List<Widget>.generate(
            80,
            (int i) => Text('$i', textDirection: TextDirection.ltr),
          ),
        ),
      ),
    );

    expectNoAnimation();

    final double doubleTolerance =
        controller.position.physics.toleranceFor(controller.position).distance * 2;
    expect(doubleTolerance, isNonZero);
    final double targetPosition = controller.position.pixels + doubleTolerance;
    controller.position.animateTo(
      targetPosition,
      duration: const Duration(seconds: 10),
      curve: Curves.linear,
    );

    expect(
      SchedulerBinding.instance.transientCallbackCount,
      equals(1),
      reason: 'Expected an animation.',
    );
  });
}

void expectNoAnimation() {
  expect(
    SchedulerBinding.instance.transientCallbackCount,
    equals(0),
    reason: 'Expected no animation.',
  );
}
