// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/color_scheme/color_scheme.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('ColorScheme Smoke Test', (WidgetTester tester) async {
    await tester.pumpWidget(const example.ColorSchemeExample());
    expect(find.text('tonalSpot (Default)'), findsOneWidget);

    expect(find.byType(example.ColorChip), findsNWidgets(43 * 9));
  });

  testWidgets('Change color seed', (WidgetTester tester) async {
    await tester.pumpWidget(const example.ColorSchemeExample());

    ColoredBox coloredBox() {
      return tester.widget<ColoredBox>(
        find.descendant(
          of: find.widgetWithText(example.ColorChip, 'primary').first,
          matching: find.byType(ColoredBox),
        ),
      );
    }

    expect(coloredBox().color, const Color(0xff65558f));
    await tester.tap(find.byType(example.SettingsButton));
    await tester.pumpAndSettle();
    expect(find.text('Settings'), findsOneWidget);
    await tester.tap(find.byType(IconButton).at(6));
    await tester.pumpAndSettle();

    expect(coloredBox().color, const Color(0xFF685F12));
  });
}
