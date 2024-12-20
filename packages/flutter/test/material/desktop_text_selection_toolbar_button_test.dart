// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  TestWidgetsFlutterBinding.ensureInitialized();

  testWidgets('can press', (WidgetTester tester) async {
    bool pressed = false;
    await tester.pumpWidget(
      MaterialApp(
        home: Center(
          child: DesktopTextSelectionToolbarButton(
            onPressed: () {
              pressed = true;
            },
            child: const Text('Tap me'),
          ),
        ),
      ),
    );

    expect(pressed, false);

    await tester.tap(find.byType(DesktopTextSelectionToolbarButton));
    expect(pressed, true);
  });

  testWidgets('passing null to onPressed disables the button', (WidgetTester tester) async {
    await tester.pumpWidget(
      const MaterialApp(
        home: Center(
          child: DesktopTextSelectionToolbarButton(onPressed: null, child: Text('Cannot tap me')),
        ),
      ),
    );

    expect(find.byType(TextButton), findsOneWidget);
    final TextButton button = tester.widget(find.byType(TextButton));
    expect(button.enabled, isFalse);
  });
}
