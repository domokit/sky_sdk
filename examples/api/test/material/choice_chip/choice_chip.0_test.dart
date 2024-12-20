// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/choice_chip/choice_chip.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Can choose an item using ChoiceChip', (WidgetTester tester) async {
    await tester.pumpWidget(const example.ChipApp());

    ChoiceChip chosenChip = tester.widget(find.byType(ChoiceChip).at(1));
    expect(chosenChip.selected, true);

    await tester.tap(find.byType(ChoiceChip).at(0));
    await tester.pumpAndSettle();

    chosenChip = tester.widget(find.byType(ChoiceChip).at(0));
    expect(chosenChip.selected, true);
  });
}
