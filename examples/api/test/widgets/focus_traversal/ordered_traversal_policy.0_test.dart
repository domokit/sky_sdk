// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_api_samples/widgets/focus_traversal/ordered_traversal_policy.0.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  bool hasFocus(WidgetTester tester, String text) {
    return Focus.of(tester.element(find.text(text))).hasPrimaryFocus;
  }

  testWidgets('The focus updates should follow the focus traversal groups policy', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const example.OrderedTraversalPolicyExampleApp());

    expect(hasFocus(tester, 'One'), isTrue);

    const List<String> focusOrder = <String>['Two', 'Three', 'Four', 'Five', 'Six', 'One'];

    for (final String text in focusOrder) {
      await tester.sendKeyEvent(LogicalKeyboardKey.tab);
      await tester.pump();

      expect(hasFocus(tester, text), isTrue);
    }
  });
}
