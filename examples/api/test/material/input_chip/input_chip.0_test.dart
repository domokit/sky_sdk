// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/input_chip/input_chip.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('', (WidgetTester tester) async {
    await tester.pumpWidget(const example.ChipApp());

    expect(find.byType(InputChip), findsNWidgets(3));

    await tester.tap(find.byIcon(Icons.clear).at(0));
    await tester.pumpAndSettle();
    expect(find.byType(InputChip), findsNWidgets(2));

    await tester.tap(find.byIcon(Icons.clear).at(0));
    await tester.pumpAndSettle();
    expect(find.byType(InputChip), findsNWidgets(1));

    await tester.tap(find.byIcon(Icons.clear).at(0));
    await tester.pumpAndSettle();
    expect(find.byType(InputChip), findsNWidgets(0));

    await tester.tap(find.text('Reset'));
    await tester.pumpAndSettle();
    expect(find.byType(InputChip), findsNWidgets(3));
  });
}
