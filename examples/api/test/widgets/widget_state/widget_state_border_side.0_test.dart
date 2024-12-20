// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui';

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/widget_state/widget_state_border_side.0.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  Finder findByBorderColor(Color color) {
    return find.byWidgetPredicate((Widget widget) {
      if (widget is! Material) {
        return false;
      }

      final ShapeBorder? shape = widget.shape;
      return shape is OutlinedBorder && shape.side.color == color;
    });
  }

  testWidgets('FilterChip displays the blue colored border when hovered', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const example.WidgetStateBorderSideExampleApp());

    // Hover over the FilterChip.
    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.moveTo(tester.getCenter(find.byType(FilterChip)));

    await tester.pumpAndSettle();

    expect(findByBorderColor(Colors.blue), findsOneWidget);
  });

  testWidgets('FilterChip displays the green colored border when pressed', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const example.WidgetStateBorderSideExampleApp());

    // Press on the FilterChip.
    final TestGesture gesture = await tester.createGesture();
    await gesture.down(tester.getCenter(find.byType(FilterChip)));

    await tester.pumpAndSettle();

    expect(findByBorderColor(Colors.green), findsOneWidget);
  });

  testWidgets('FilterChip displays the red colored border when selected', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const example.WidgetStateBorderSideExampleApp());

    expect(findByBorderColor(Colors.red), findsOneWidget);
  });

  testWidgets('FilterChip displays the correct border color when not selected', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const example.WidgetStateBorderSideExampleApp());

    await tester.tap(find.byType(FilterChip));
    await tester.pumpAndSettle();

    final ThemeData theme = Theme.of(tester.element(find.byType(FilterChip)));

    // FilterChip's border color defaults to ColorScheme.outlineVariant.
    expect(findByBorderColor(theme.colorScheme.outlineVariant), findsOneWidget);
  });
}
