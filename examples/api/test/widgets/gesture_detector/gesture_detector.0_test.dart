// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/gesture_detector/gesture_detector.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('GestureDetector updates icon color and text on tap', (WidgetTester tester) async {
    await tester.pumpWidget(const example.GestureDetectorExampleApp());

    Icon icon = tester.widget(find.byIcon(Icons.lightbulb_outline));

    expect(find.text('TURN LIGHT ON'), findsOneWidget);
    expect(icon.color, Colors.black);

    await tester.tap(find.byType(GestureDetector));
    await tester.pump();

    icon = tester.widget(find.byIcon(Icons.lightbulb_outline));

    expect(find.text('TURN LIGHT OFF'), findsOneWidget);
    expect(icon.color, Colors.yellow.shade600);

    await tester.tap(find.byType(GestureDetector));
    await tester.pump();

    icon = tester.widget(find.byIcon(Icons.lightbulb_outline));

    expect(find.text('TURN LIGHT ON'), findsOneWidget);
    expect(icon.color, Colors.black);
  });
}
