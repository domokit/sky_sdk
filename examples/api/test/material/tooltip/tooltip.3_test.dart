// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/tooltip/tooltip.3.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Tooltip is visible when tapping button', (WidgetTester tester) async {
    const String tooltipText = 'I am a Tooltip';

    await tester.pumpWidget(const example.TooltipExampleApp());

    // Tooltip is not visible before tapping the button.
    expect(find.text(tooltipText), findsNothing);
    // Tap on the button and wait for the tooltip to appear.
    await tester.tap(find.byType(FloatingActionButton));
    await tester.pump(const Duration(milliseconds: 10));
    expect(find.text(tooltipText), findsOneWidget);
    // Tap on the tooltip and wait for the tooltip to disappear.
    await tester.tap(find.byTooltip(tooltipText));
    await tester.pump(const Duration(seconds: 1));
    expect(find.text(tooltipText), findsNothing);
  });
}
