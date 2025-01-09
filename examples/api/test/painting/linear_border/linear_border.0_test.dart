// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

import 'package:flutter_api_samples/painting/linear_border/linear_border.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Smoke Test', (WidgetTester tester) async {
    await tester.pumpWidget(const example.ExampleApp());
    expect(find.byType(example.Home), findsOneWidget);

    // Scroll the interpolation example into view

    await tester.scrollUntilVisible(
      find.byIcon(Icons.play_arrow),
      500.0,
      scrollable: find.byType(Scrollable),
    );
    expect(find.byIcon(Icons.play_arrow), findsOneWidget);

    // Run the interpolation example

    await tester.tap(find.byIcon(Icons.play_arrow));
    await tester.pumpAndSettle();

    await tester.tap(find.byIcon(Icons.play_arrow));
    await tester.pumpAndSettle();

    final TestGesture gesture = await tester.startGesture(
      tester.getCenter(find.text('Interpolation')),
    );
    await gesture.moveTo(tester.getCenter(find.text('Hover')));
    await tester.pumpAndSettle();
    await gesture.moveTo(tester.getCenter(find.text('Interpolation')));
    await tester.pumpAndSettle();
  });
}
