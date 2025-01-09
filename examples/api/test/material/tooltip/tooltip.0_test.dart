// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/tooltip/tooltip.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Tooltip is visible when hovering over text', (WidgetTester tester) async {
    const String tooltipText = 'I am a Tooltip';

    await tester.pumpWidget(const example.TooltipExampleApp());

    TestGesture? gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    addTearDown(() async {
      if (gesture != null) {
        return gesture.removePointer();
      }
    });
    await gesture.addPointer();
    await gesture.moveTo(const Offset(1.0, 1.0));
    await tester.pump();
    expect(find.text(tooltipText), findsNothing);
    // Move the mouse over the text and wait for the tooltip to appear.
    final Finder tooltip = find.byType(Tooltip);
    await gesture.moveTo(tester.getCenter(tooltip));
    await tester.pump(const Duration(milliseconds: 10));
    expect(find.text(tooltipText), findsOneWidget);
    // Move the mouse away and wait for the tooltip to disappear.
    await gesture.moveTo(const Offset(1.0, 1.0));
    await tester.pumpAndSettle();
    await gesture.removePointer();
    gesture = null;
    expect(find.text(tooltipText), findsNothing);
  });
}
