// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/material_state/material_state_property.0.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  Color getButtonForegroundColor(WidgetTester tester) {
    final Material widget = tester.widget(
      find.descendant(
        of: find.byType(example.MaterialStatePropertyExample),
        matching: find.widgetWithText(Material, 'TextButton'),
      ),
    );
    return widget.textStyle!.color!;
  }

  testWidgets('The foreground color of the TextButton should be red by default', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const example.MaterialStatePropertyExampleApp());

    expect(getButtonForegroundColor(tester), Colors.red);
  });

  testWidgets('The foreground color of the TextButton should be blue when hovered', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const example.MaterialStatePropertyExampleApp());

    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer(location: tester.getCenter(find.byType(TextButton)));
    await tester.pump();

    expect(getButtonForegroundColor(tester), Colors.blue);
  });
}
