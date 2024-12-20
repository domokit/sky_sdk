// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/selection_area/selection_area.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Texts are descendant of the SelectionArea', (WidgetTester tester) async {
    await tester.pumpWidget(const example.SelectionAreaExampleApp());

    expect(
      find.descendant(of: find.byType(SelectionArea), matching: find.byType(Text)),
      findsExactly(4),
    );

    final List<String> selectableTexts = <String>[
      'SelectionArea Sample',
      'Row 1',
      'Row 2',
      'Row 3',
    ];

    for (final String text in selectableTexts) {
      expect(
        find.descendant(of: find.byType(SelectionArea), matching: find.text(text)),
        findsExactly(1),
      );
    }
  });
}
