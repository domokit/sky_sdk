// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/gesture_detector/gesture_detector.2.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  void expectBorders(
    WidgetTester tester, {
    required bool expectGreenHasBorder,
    required bool expectYellowHasBorder,
  }) {
    final Finder containerFinder = find.byType(Container);
    final Finder greenFinder = containerFinder.first;
    final Finder yellowFinder = containerFinder.last;

    final Container greenContainer = tester.firstWidget<Container>(greenFinder);
    final BoxDecoration? greenDecoration = greenContainer.decoration as BoxDecoration?;
    expect(greenDecoration?.border, expectGreenHasBorder ? isNot(null) : null);

    final Container yellowContainer = tester.firstWidget<Container>(yellowFinder);
    final BoxDecoration? yellowDecoration = yellowContainer.decoration as BoxDecoration?;
    expect(yellowDecoration?.border, expectYellowHasBorder ? isNot(null) : null);
  }

  void expectInnerGestureDetectorBehavior(WidgetTester tester, HitTestBehavior behavior) {
    // There is a third GestureDetector added by Scaffold.
    final Finder innerGestureDetectorFinder = find.byType(GestureDetector).at(1);
    final GestureDetector innerGestureDetector = tester.firstWidget<GestureDetector>(
      innerGestureDetectorFinder,
    );
    expect(innerGestureDetector.behavior, behavior);
  }

  testWidgets('Only the green Container shows a red border when tapped', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const example.NestedGestureDetectorsApp());

    final Finder greenFinder = find.byType(Container).first;
    final Offset greenTopLeftCorner = tester.getTopLeft(greenFinder);
    await tester.tapAt(greenTopLeftCorner);
    await tester.pumpAndSettle();
    expectBorders(tester, expectGreenHasBorder: true, expectYellowHasBorder: false);

    // Tap on the button to toggle inner GestureDetector.behavior
    final Finder toggleBehaviorFinder = find.byType(ElevatedButton).last;
    await tester.tap(toggleBehaviorFinder);
    await tester.pump();
    expectInnerGestureDetectorBehavior(tester, HitTestBehavior.translucent);

    // Tap again on the green container, expect nothing changed
    await tester.tapAt(greenTopLeftCorner);
    await tester.pump();
    expectBorders(tester, expectGreenHasBorder: true, expectYellowHasBorder: false);

    // Tap on the reset button
    final Finder resetFinder = find.byType(ElevatedButton).first;
    await tester.tap(resetFinder);
    await tester.pump();
    expectInnerGestureDetectorBehavior(tester, HitTestBehavior.opaque);
  });

  testWidgets('Only the yellow Container shows a red border when tapped', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const example.NestedGestureDetectorsApp());

    final Finder yellowFinder = find.byType(Container).last;
    final Offset yellowTopLeftCorner = tester.getTopLeft(yellowFinder);
    await tester.tapAt(yellowTopLeftCorner);
    await tester.pump();
    expectBorders(tester, expectGreenHasBorder: false, expectYellowHasBorder: true);

    // Tap on the button to toggle inner GestureDetector.behavior
    final Finder toggleBehaviorFinder = find.byType(ElevatedButton).last;
    await tester.tap(toggleBehaviorFinder);
    await tester.pump();
    expectInnerGestureDetectorBehavior(tester, HitTestBehavior.translucent);

    // Tap again on the yellow container, expect nothing changed
    await tester.tapAt(yellowTopLeftCorner);
    await tester.pump();
    expectBorders(tester, expectGreenHasBorder: false, expectYellowHasBorder: true);

    // Tap on the reset button
    final Finder resetFinder = find.byType(ElevatedButton).first;
    await tester.tap(resetFinder);
    await tester.pump();
    expectInnerGestureDetectorBehavior(tester, HitTestBehavior.opaque);
  });
}
