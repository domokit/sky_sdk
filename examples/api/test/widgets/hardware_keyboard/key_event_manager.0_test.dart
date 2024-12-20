// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_api_samples/widgets/hardware_keyboard/key_event_manager.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('App tracks lifecycle states', (WidgetTester tester) async {
    Future<String> getCapturedKey() async {
      final Widget textWidget = tester.firstWidget(
        find.textContaining('is not handled by shortcuts.'),
      );
      expect(textWidget, isA<Text>());
      return (textWidget as Text).data!.split(' ').first;
    }

    await tester.pumpWidget(
      const MaterialApp(home: Scaffold(body: Center(child: example.FallbackDemo()))),
    );

    // Focus on the first text field.
    await tester.tap(find.byType(TextField).first);

    // Press Q, which is taken as a text input, unhandled by the keyboard system.
    await tester.sendKeyEvent(LogicalKeyboardKey.keyQ);
    await tester.pump();
    expect(await getCapturedKey(), 'Q');

    // Press Ctrl-A, which is taken as a text short cut, handled by the keyboard system.
    await tester.sendKeyDownEvent(LogicalKeyboardKey.controlLeft);
    await tester.sendKeyEvent(LogicalKeyboardKey.keyA);
    await tester.sendKeyUpEvent(LogicalKeyboardKey.controlLeft);
    expect(await getCapturedKey(), 'Q');

    // Press A, which is taken as a text input, handled by the keyboard system.
    await tester.sendKeyEvent(LogicalKeyboardKey.keyA);
    await tester.pump();
    expect(await getCapturedKey(), 'A');

    // Focus on the second text field.
    await tester.tap(find.byType(TextField).last);

    // Press Q, which is taken as a stub shortcut, handled by the keyboard system.
    await tester.sendKeyEvent(LogicalKeyboardKey.keyQ);
    await tester.pump();
    expect(await getCapturedKey(), 'A');

    // Press B, which is taken as a text input, unhandled by the keyboard system.
    await tester.sendKeyEvent(LogicalKeyboardKey.keyB);
    await tester.pump();
    expect(await getCapturedKey(), 'B');

    // Press Ctrl-A, which is taken as a text short cut, handled by the keyboard system.
    await tester.sendKeyDownEvent(LogicalKeyboardKey.controlLeft);
    await tester.sendKeyEvent(LogicalKeyboardKey.keyA);
    await tester.sendKeyUpEvent(LogicalKeyboardKey.controlLeft);
    expect(await getCapturedKey(), 'B');
  });
}
