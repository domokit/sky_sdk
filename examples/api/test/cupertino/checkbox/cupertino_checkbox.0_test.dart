// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';
import 'package:flutter_api_samples/cupertino/checkbox/cupertino_checkbox.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Checkbox can be checked', (WidgetTester tester) async {
    await tester.pumpWidget(const example.CupertinoCheckboxApp());

    CupertinoCheckbox checkbox = tester.widget(find.byType(CupertinoCheckbox));

    // Verify the initial state of the checkbox.
    expect(checkbox.value, isTrue);
    expect(checkbox.tristate, isTrue);

    // Tap the checkbox and verify the state change.
    await tester.tap(find.byType(CupertinoCheckbox));
    await tester.pump();
    checkbox = tester.widget(find.byType(CupertinoCheckbox));

    expect(checkbox.value, isNull);

    // Tap the checkbox and verify the state change.
    await tester.tap(find.byType(CupertinoCheckbox));
    await tester.pump();
    checkbox = tester.widget(find.byType(CupertinoCheckbox));

    expect(checkbox.value, isFalse);

    await tester.tap(find.byType(CupertinoCheckbox));
    await tester.pump();
    checkbox = tester.widget(find.byType(CupertinoCheckbox));

    expect(checkbox.value, isTrue);
  });
}
