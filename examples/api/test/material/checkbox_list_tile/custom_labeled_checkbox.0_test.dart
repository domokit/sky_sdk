// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/checkbox_list_tile/custom_labeled_checkbox.0.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('LinkedLabelCheckbox contains RichText and Checkbox', (WidgetTester tester) async {
    await tester.pumpWidget(const example.LabeledCheckboxApp());

    // Label text is in a RichText widget with the correct text.
    final RichText richText = tester.widget(find.byType(RichText).first);
    expect(richText.text.toPlainText(), 'Linked, tappable label text');

    // Checkbox is initially unchecked.
    Checkbox checkbox = tester.widget(find.byType(Checkbox));
    expect(checkbox.value, isFalse);

    // Tap the checkbox to check it.
    await tester.tap(find.byType(Checkbox));
    await tester.pumpAndSettle();

    // Checkbox is now checked.
    checkbox = tester.widget(find.byType(Checkbox));
    expect(checkbox.value, isTrue);
  });
}
