// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui';

import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_api_samples/material/material_state/material_state_mouse_cursor.0.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('MaterialStateMouseCursorExampleApp displays ListTile', (WidgetTester tester) async {
    await tester.pumpWidget(const example.MaterialStateMouseCursorExampleApp());

    expect(find.byType(ListTile), findsOneWidget);
    expect(find.text('ListTile'), findsOneWidget);
  });

  testWidgets('ListTile displays correct mouse cursor when enabled', (WidgetTester tester) async {
    await tester.pumpWidget(
      const MaterialApp(
        home: Scaffold(body: example.MaterialStateMouseCursorExample(enabled: true)),
      ),
    );

    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer(location: tester.getCenter(find.byType(ListTile)));
    addTearDown(gesture.removePointer);

    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.click,
    );
  });

  testWidgets('ListTile displays correct mouse cursor when disabled', (WidgetTester tester) async {
    await tester.pumpWidget(
      const MaterialApp(
        home: Scaffold(body: example.MaterialStateMouseCursorExample(enabled: false)),
      ),
    );

    final TestGesture gesture = await tester.createGesture(kind: PointerDeviceKind.mouse);
    await gesture.addPointer(location: tester.getCenter(find.byType(ListTile)));
    addTearDown(gesture.removePointer);

    expect(
      RendererBinding.instance.mouseTracker.debugDeviceActiveCursor(1),
      SystemMouseCursors.forbidden,
    );
  });
}
