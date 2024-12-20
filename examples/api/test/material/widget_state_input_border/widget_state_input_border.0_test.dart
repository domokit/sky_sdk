// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/widget_state_input_border/widget_state_input_border.0.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('InputBorder appearance matches configuration', (WidgetTester tester) async {
    const WidgetStateInputBorder inputBorder = WidgetStateInputBorder.resolveWith(
      example.WidgetStateInputBorderExample.veryCoolBorder,
    );

    void expectBorderToMatch(Set<WidgetState> states) {
      final RenderBox renderBox = tester.renderObject(
        find.descendant(of: find.byType(TextField), matching: find.byType(CustomPaint)),
      );

      final BorderSide side = inputBorder.resolve(states).borderSide;
      expect(renderBox, paints..line(color: side.color, strokeWidth: side.width));
    }

    await tester.pumpWidget(const example.WidgetStateInputBorderExampleApp());
    expectBorderToMatch(const <WidgetState>{WidgetState.disabled});

    await tester.tap(find.byType(FilterChip));
    await tester.pumpAndSettle();
    expectBorderToMatch(const <WidgetState>{});

    await tester.tap(find.byType(TextField));
    await tester.pumpAndSettle();
    expectBorderToMatch(const <WidgetState>{WidgetState.focused});

    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer(location: tester.getCenter(find.byType(TextField)));
    await tester.pumpAndSettle();
    expectBorderToMatch(const <WidgetState>{WidgetState.focused, WidgetState.hovered});
  });
}
