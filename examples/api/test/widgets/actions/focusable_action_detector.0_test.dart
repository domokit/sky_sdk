// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_api_samples/widgets/actions/focusable_action_detector.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  final Finder redContainerFinder = find.byWidgetPredicate(
    (Widget widget) => widget is Container && widget.color == Colors.red,
  );

  testWidgets('Taps on the "And me" button toggle the red box', (WidgetTester tester) async {
    await tester.pumpWidget(const example.FocusableActionDetectorExampleApp());

    expect(find.widgetWithText(AppBar, 'FocusableActionDetector Example'), findsOne);

    expect(redContainerFinder, findsNothing);

    await tester.tap(find.text('And Me'));
    await tester.pump();

    expect(redContainerFinder, findsOne);

    await tester.tap(find.text('And Me'));
    await tester.pump();

    expect(redContainerFinder, findsNothing);
  });

  testWidgets('Hits on the X key when "And me" is focused toggle the red box', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const example.FocusableActionDetectorExampleApp());

    expect(find.widgetWithText(AppBar, 'FocusableActionDetector Example'), findsOne);

    expect(redContainerFinder, findsNothing);

    await tester.sendKeyEvent(LogicalKeyboardKey.tab); // Focuses the "Press Me" button.
    await tester.pump();

    expect(redContainerFinder, findsNothing);

    await tester.sendKeyEvent(LogicalKeyboardKey.keyX);
    await tester.pump();

    expect(redContainerFinder, findsNothing);

    await tester.sendKeyEvent(LogicalKeyboardKey.tab); // Focuses the "And Me" button.
    await tester.pump();

    expect(redContainerFinder, findsNothing);

    await tester.sendKeyEvent(LogicalKeyboardKey.keyX);
    await tester.pump();

    expect(redContainerFinder, findsOne);

    await tester.sendKeyEvent(LogicalKeyboardKey.keyX);
    await tester.pump();

    expect(redContainerFinder, findsNothing);
  });
}
