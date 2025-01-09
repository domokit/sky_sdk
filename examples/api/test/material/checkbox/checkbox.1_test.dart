// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/checkbox/checkbox.1.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Checkbox can be checked', (WidgetTester tester) async {
    await tester.pumpWidget(const example.CheckboxExampleApp());

    expect(find.byType(Checkbox), findsNWidgets(3));
    Checkbox checkbox = tester.widget(find.byType(Checkbox).first);
    Checkbox checkboxWithError = tester.widget(find.byType(Checkbox).at(1));
    Checkbox checkboxDisabled = tester.widget(find.byType(Checkbox).last);

    // Verify the initial state of the checkboxes.
    expect(checkbox.value, isTrue);
    expect(checkboxWithError.value, isTrue);
    expect(checkboxDisabled.value, isTrue);

    expect(checkboxWithError.isError, isTrue);
    expect(checkboxDisabled.onChanged, null);

    expect(checkbox.tristate, isTrue);
    expect(checkboxWithError.tristate, isTrue);
    expect(checkboxDisabled.tristate, isTrue);

    // Tap the first Checkbox and verify the state change.
    await tester.tap(find.byType(Checkbox).first);
    await tester.pump();
    checkbox = tester.widget(find.byType(Checkbox).first);
    checkboxWithError = tester.widget(find.byType(Checkbox).at(1));
    checkboxDisabled = tester.widget(find.byType(Checkbox).last);

    expect(checkbox.value, isNull);
    expect(checkboxWithError.value, isNull);
    expect(checkboxDisabled.value, isNull);

    // Tap the second Checkbox and verify the state change.
    await tester.tap(find.byType(Checkbox).at(1));
    await tester.pump();

    checkbox = tester.widget(find.byType(Checkbox).first);
    checkboxWithError = tester.widget(find.byType(Checkbox).at(1));
    checkboxDisabled = tester.widget(find.byType(Checkbox).last);

    expect(checkbox.value, isFalse);
    expect(checkboxWithError.value, isFalse);
    expect(checkboxDisabled.value, isFalse);

    // Tap the third Checkbox and verify that should remain unchanged.
    await tester.tap(find.byType(Checkbox).last);
    await tester.pump();

    checkbox = tester.widget(find.byType(Checkbox).first);
    checkboxWithError = tester.widget(find.byType(Checkbox).at(1));
    checkboxDisabled = tester.widget(find.byType(Checkbox).last);

    expect(checkbox.value, isFalse);
    expect(checkboxWithError.value, isFalse);
    expect(checkboxDisabled.value, isFalse);
  });
}
