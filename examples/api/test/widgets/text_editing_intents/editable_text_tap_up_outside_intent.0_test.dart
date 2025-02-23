// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/text_editing_intents/editable_text_tap_up_outside_intent.0.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Unfocuses TextField on tap', (WidgetTester tester) async {
    await tester.pumpWidget(const example.SampleApp());

    final Finder finder = find.byType(TextField);
    final TextField textField = tester.firstWidget(finder);

    await tester.tap(finder);

    await tester.pump();

    expect(textField.focusNode!.hasFocus, true);

    // Tap the center of the Scaffold, outside the TextField.
    await tester.tap(find.byType(Scaffold));

    await tester.pump();

    expect(textField.focusNode!.hasFocus, false);
  });

  testWidgets('Does not unfocus TextField on scroll', (WidgetTester tester) async {
    await tester.pumpWidget(const example.SampleApp());

    final Finder finder = find.byType(TextField);
    final TextField textField = tester.firstWidget(finder);

    await tester.tap(finder);

    await tester.pump();

    expect(textField.focusNode!.hasFocus, true);

    // Tap the center of the Scaffold, outside the TextField.
    await tester.drag(find.byType(Scaffold), const Offset(0.0, -100.0));

    await tester.pump();

    expect(textField.focusNode!.hasFocus, true);
  });
}
