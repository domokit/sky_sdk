// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/text_button/text_button.1.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('SelectableButton', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(colorScheme: const ColorScheme.light()),
        home: const example.Home(),
      ),
    );

    final Finder button = find.byType(example.SelectableButton);

    example.SelectableButton buttonWidget() => tester.widget<example.SelectableButton>(button);

    Material buttonMaterial() {
      return tester.widget<Material>(
        find.descendant(of: find.byType(example.SelectableButton), matching: find.byType(Material)),
      );
    }

    expect(buttonWidget().selected, false);
    expect(
      buttonMaterial().textStyle!.color,
      const ColorScheme.light().primary,
    ); // default button foreground color
    expect(buttonMaterial().color, Colors.transparent); // default button background color

    await tester.tap(button); // Toggles the button's selected property.
    await tester.pumpAndSettle();
    expect(buttonWidget().selected, true);
    expect(buttonMaterial().textStyle!.color, Colors.white);
    expect(buttonMaterial().color, Colors.indigo);

    await tester.tap(button); // Toggles the button's selected property.
    await tester.pumpAndSettle();
    expect(buttonWidget().selected, false);
    expect(buttonMaterial().textStyle!.color, const ColorScheme.light().primary);
    expect(buttonMaterial().color, Colors.transparent);
  });
}
