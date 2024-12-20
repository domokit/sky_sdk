// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/material_state/material_state_outlined_border.0.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  Finder findBorderShape(OutlinedBorder? shape) {
    return find.descendant(
      of: find.byType(FilterChip),
      matching: find.byWidgetPredicate((Widget widget) {
        if (widget is! Material) {
          return false;
        }
        return widget.shape == shape;
      }),
    );
  }

  testWidgets('FilterChip displays the correct border when selected', (WidgetTester tester) async {
    await tester.pumpWidget(const example.MaterialStateOutlinedBorderExampleApp());

    expect(
      findBorderShape(const RoundedRectangleBorder(side: BorderSide(color: Colors.transparent))),
      findsOne,
    );
  });

  testWidgets('FilterChip displays the correct border when not selected', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const example.MaterialStateOutlinedBorderExampleApp());

    await tester.tap(find.byType(FilterChip));
    await tester.pumpAndSettle();

    expect(
      findBorderShape(
        RoundedRectangleBorder(
          side: const BorderSide(color: Color(0xFFCAC4D0)),
          borderRadius: BorderRadius.circular(8),
        ),
      ),
      findsOne,
    );
  });
}
