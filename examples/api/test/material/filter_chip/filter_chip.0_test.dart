// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/filter_chip/filter_chip.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Filter exercises using FilterChip', (WidgetTester tester) async {
    const String baseText = 'Looking for: ';

    await tester.pumpWidget(const example.ChipApp());

    expect(find.text(baseText), findsOneWidget);

    FilterChip filterChip = tester.widget(find.byType(FilterChip).at(2));
    expect(filterChip.selected, false);

    await tester.tap(find.byType(FilterChip).at(2));
    await tester.pumpAndSettle();
    filterChip = tester.widget(find.byType(FilterChip).at(2));
    expect(filterChip.selected, true);

    expect(find.text('${baseText}cycling'), findsOneWidget);

    await tester.tap(find.byType(FilterChip).at(3));
    await tester.pumpAndSettle();
    filterChip = tester.widget(find.byType(FilterChip).at(3));
    expect(filterChip.selected, true);

    expect(find.text('${baseText}cycling, hiking'), findsOneWidget);
  });
}
