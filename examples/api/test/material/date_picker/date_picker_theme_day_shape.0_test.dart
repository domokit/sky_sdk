// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/date_picker/date_picker_theme_day_shape.0.dart'
    as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('DatePickerThemeData.dayShape updates day selection shape decoration', (
    WidgetTester tester,
  ) async {
    final ThemeData theme = ThemeData();
    final OutlinedBorder dayShape = RoundedRectangleBorder(
      borderRadius: BorderRadius.circular(8.0),
    );
    const Color todayBackgroundColor = Colors.amber;
    const Color todayForegroundColor = Colors.black;
    const BorderSide todayBorder = BorderSide(width: 2);

    ShapeDecoration? findDayDecoration(WidgetTester tester, String day) {
      return tester
              .widget<Ink>(find.ancestor(of: find.text(day), matching: find.byType(Ink)))
              .decoration
          as ShapeDecoration?;
    }

    await tester.pumpWidget(const example.DatePickerApp());

    await tester.tap(find.text('Open Date Picker'));
    await tester.pumpAndSettle();

    // Test the current day shape decoration.
    ShapeDecoration dayShapeDecoration = findDayDecoration(tester, '15')!;
    expect(dayShapeDecoration.color, todayBackgroundColor);
    expect(
      dayShapeDecoration.shape,
      dayShape.copyWith(side: todayBorder.copyWith(color: todayForegroundColor)),
    );

    // Test the selected day shape decoration.
    dayShapeDecoration = findDayDecoration(tester, '20')!;
    expect(dayShapeDecoration.color, theme.colorScheme.primary);
    expect(dayShapeDecoration.shape, dayShape);

    // Tap to select current day as the selected day.
    await tester.tap(find.text('15'));
    await tester.pumpAndSettle();

    // Test the selected day shape decoration.
    dayShapeDecoration = findDayDecoration(tester, '15')!;
    expect(dayShapeDecoration.color, todayBackgroundColor);
    expect(
      dayShapeDecoration.shape,
      dayShape.copyWith(side: todayBorder.copyWith(color: todayForegroundColor)),
    );
  });
}
