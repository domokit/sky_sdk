// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/gestures/tap_and_drag/tap_and_drag.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Single tap + drag should not change the scale of child', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const example.TapAndDragToZoomApp());

    double getScale() {
      final RenderBox box = tester.renderObject(find.byType(Container).first);
      return box.getTransformTo(null)[0];
    }

    final Finder containerFinder = find.byType(Container).first;
    final Offset centerOfChild = tester.getCenter(containerFinder);

    expect(getScale(), 1.0);

    // Single tap + drag down.
    final TestGesture gesture = await tester.startGesture(centerOfChild);
    await tester.pump();
    await gesture.moveTo(centerOfChild + const Offset(0, 100.0));
    await tester.pump();
    expect(getScale(), 1.0);

    // Single tap + drag up.
    await gesture.moveTo(centerOfChild);
    await tester.pump();
    expect(getScale(), 1.0);
  });

  testWidgets('Double tap + drag should change the scale of the child', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const example.TapAndDragToZoomApp());

    double getScale() {
      final RenderBox box = tester.renderObject(find.byType(Container).first);
      return box.getTransformTo(null)[0];
    }

    final Finder containerFinder = find.byType(Container).first;
    final Offset centerOfChild = tester.getCenter(containerFinder);

    expect(getScale(), 1.0);

    // Double tap + drag down to scale up.
    final TestGesture gesture = await tester.startGesture(centerOfChild);
    await tester.pump();
    await gesture.up();
    await tester.pump();

    await gesture.down(centerOfChild);
    await tester.pump();
    await gesture.moveTo(centerOfChild + const Offset(0, 100.0));
    await tester.pump();
    expect(getScale(), greaterThan(1.0));

    // Scale is reset on drag end.
    await gesture.up();
    await tester.pumpAndSettle();
    expect(getScale(), 1.0);

    // Double tap + drag up to scale down.
    await gesture.down(centerOfChild);
    await tester.pump();
    await gesture.up();
    await tester.pump();

    await gesture.down(centerOfChild);
    await tester.pump();
    await gesture.moveTo(centerOfChild + const Offset(0, -100.0));
    await tester.pump();
    expect(getScale(), lessThan(1.0));

    // Scale is reset on drag end.
    await gesture.up();
    await tester.pumpAndSettle();
    expect(getScale(), 1.0);
  });
}
