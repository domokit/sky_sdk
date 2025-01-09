// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/widgets/editable_text/text_editing_controller.0.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Forces text to be lower case', (WidgetTester tester) async {
    await tester.pumpWidget(const example.TextEditingControllerExampleApp());

    const String input = 'Almost Everything Is a WIDGET! 💙';

    await tester.enterText(find.byType(TextFormField), input);
    await tester.pump();

    final TextFormField textField = tester.widget(find.byType(TextFormField));
    final TextEditingController controller = textField.controller!;

    expect(find.text(input.toLowerCase()), findsOneWidget);
    expect(controller.text, input.toLowerCase());
  });

  testWidgets('Keeps the caret at the end of the input', (WidgetTester tester) async {
    await tester.pumpWidget(const example.TextEditingControllerExampleApp());

    const String input = 'flutter';

    await tester.enterText(find.byType(TextFormField), input);
    await tester.pump();

    expect(find.text(input), findsOneWidget);

    final TextFormField textField = tester.widget(find.byType(TextFormField));
    final TextEditingController controller = textField.controller!;

    // Verify that the caret positioned at the end of the input.
    expect(controller.selection, const TextSelection.collapsed(offset: input.length));

    final RenderBox box = tester.renderObject(find.byType(TextFormField));

    // Calculate the center-left point of the field.
    final Offset centerLeftPoint = box.localToGlobal(Offset(0, box.size.height / 2));

    // Tap on the center-left point of the field to try to change the caret
    // position.
    await tester.tapAt(centerLeftPoint);
    await tester.pump();

    // Verify that the caret position remains unchanged.
    expect(controller.selection, const TextSelection.collapsed(offset: input.length));
  });
}
