// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui';

import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_api_samples/widgets/widget_state/widget_state_mouse_cursor.0.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('ListTile displays correct mouse cursor when disabled', (WidgetTester tester) async {
    await tester.pumpWidget(const example.WidgetStateMouseCursorExampleApp());

    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer(location: tester.getCenter(find.byType(ListTile)));
    addTearDown(gesture.removePointer);

    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.forbidden,
    );
  });

  testWidgets('Switch enables ListTile', (WidgetTester tester) async {
    await tester.pumpWidget(const example.WidgetStateMouseCursorExampleApp());

    ListTile listTile = tester.widget(find.byType(ListTile));
    expect(listTile.enabled, isFalse);

    // Enable ListTile using Switch.
    await tester.tap(find.byType(Switch));
    await tester.pumpAndSettle();

    listTile = tester.widget(find.byType(ListTile));
    expect(listTile.enabled, isTrue);
  });

  testWidgets('ListTile displays correct mouse cursor when enabled', (WidgetTester tester) async {
    await tester.pumpWidget(const example.WidgetStateMouseCursorExampleApp());

    // Enable ListTile using Switch.
    await tester.tap(find.byType(Switch));
    await tester.pumpAndSettle();

    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer(location: tester.getCenter(find.byType(ListTile)));
    addTearDown(gesture.removePointer);

    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.click,
    );
  });
}
