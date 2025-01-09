// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_api_samples/widgets/focus_traversal/focus_traversal_group.0.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  bool hasFocus(WidgetTester tester, String text) {
    return Focus.of(tester.element(find.text(text))).hasPrimaryFocus;
  }

  testWidgets('The focus updates should follow the focus traversal groups policy', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const example.FocusTraversalGroupExampleApp());

    // Set the focus to the first button.
    Focus.of(tester.element(find.text('num: 0'))).requestFocus();
    await tester.pump();

    expect(hasFocus(tester, 'num: 0'), isTrue);

    const List<String> focusOrder = <String>[
      'num: 1',
      'num: 2',
      'String: A',
      'String: B',
      'String: C',
      'ignored num: 3',
      'ignored num: 2',
      'ignored num: 1',
      'num: 0',
    ];

    for (final String text in focusOrder) {
      await tester.sendKeyEvent(LogicalKeyboardKey.tab);
      await tester.pump();

      expect(hasFocus(tester, text), isTrue);
    }
  });
}
