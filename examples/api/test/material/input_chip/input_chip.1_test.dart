// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/input_chip/input_chip.1.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  final String replacementChar = String.fromCharCode(
    example.ChipsInputEditingController.kObjectReplacementChar,
  );

  testWidgets('User input generates InputChips', (WidgetTester tester) async {
    await tester.pumpWidget(const example.EditableChipFieldApp());
    await tester.pumpAndSettle();

    expect(find.byType(example.EditableChipFieldApp), findsOneWidget);
    expect(find.byType(example.ChipsInput<String>), findsOneWidget);
    expect(find.byType(InputChip), findsOneWidget);

    example.ChipsInputState<String> state = tester.state(find.byType(example.ChipsInput<String>));
    expect(state.controller.textWithoutReplacements.isEmpty, true);

    await tester.tap(find.byType(example.ChipsInput<String>));
    await tester.pumpAndSettle();
    expect(tester.testTextInput.isVisible, true);
    // Simulating text typing on the input field.
    tester.testTextInput.enterText('${replacementChar}ham');
    await tester.pumpAndSettle();
    expect(find.byType(InputChip), findsOneWidget);

    state = tester.state(find.byType(example.ChipsInput<String>));
    await tester.pumpAndSettle();
    expect(state.controller.textWithoutReplacements, 'ham');

    // Add new InputChip by sending the "done" action.
    await tester.testTextInput.receiveAction(TextInputAction.done);
    await tester.pumpAndSettle();
    expect(state.controller.textWithoutReplacements.isEmpty, true);

    expect(find.byType(InputChip), findsNWidgets(2));

    // Simulate item deletion.
    await tester.tap(
      find.descendant(of: find.byType(InputChip), matching: find.byType(InkWell).last),
    );
    await tester.pumpAndSettle();
    expect(find.byType(InputChip), findsOneWidget);

    await tester.tap(
      find.descendant(of: find.byType(InputChip), matching: find.byType(InkWell).last),
    );
    await tester.pumpAndSettle();
    expect(find.byType(InputChip), findsNothing);
  });
}
