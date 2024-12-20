// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/services.dart';
import 'package:flutter_api_samples/services/keyboard_key/physical_keyboard_key.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Responds to key', (WidgetTester tester) async {
    await tester.pumpWidget(const example.KeyExampleApp());

    await tester.tap(find.text('Click to focus'));
    await tester.pumpAndSettle();
    expect(find.text('Press a key'), findsOneWidget);
    await tester.sendKeyEvent(LogicalKeyboardKey.keyQ, physicalKey: PhysicalKeyboardKey.keyA);
    await tester.pumpAndSettle();
    expect(find.text('Pressed the key next to CAPS LOCK!'), findsOneWidget);

    await tester.sendKeyEvent(LogicalKeyboardKey.keyB, physicalKey: PhysicalKeyboardKey.keyB);
    await tester.pumpAndSettle();
    expect(find.text('Not the key next to CAPS LOCK: Pressed Key B'), findsOneWidget);
  });
}
