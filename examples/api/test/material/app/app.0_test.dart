// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/app/app.0.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('Theme animation can be customized using AnimationStyle', (
    WidgetTester tester,
  ) async {
    await tester.pumpWidget(const example.MaterialAppExample());

    Material getScaffoldMaterial() {
      return tester.widget<Material>(
        find.descendant(of: find.byType(Scaffold), matching: find.byType(Material).first),
      );
    }

    final ThemeData lightTheme = ThemeData(colorSchemeSeed: Colors.green);
    final ThemeData darkTheme = ThemeData(
      colorSchemeSeed: Colors.green,
      brightness: Brightness.dark,
    );

    // Test the default animation.
    expect(getScaffoldMaterial().color, lightTheme.colorScheme.surface);

    await tester.tap(find.text('Switch Theme Mode'));
    await tester.pump();
    // Advance the animation by half of the default duration.
    await tester.pump(const Duration(milliseconds: 100));

    // The Scaffold background color is updated.
    expect(
      getScaffoldMaterial().color,
      Color.lerp(lightTheme.colorScheme.surface, darkTheme.colorScheme.surface, 0.5),
    );

    await tester.pumpAndSettle();

    // The Scaffold background color is now fully dark.
    expect(getScaffoldMaterial().color, darkTheme.colorScheme.surface);

    // Test the custom animation curve and duration.
    await tester.tap(find.text('Custom'));
    await tester.pumpAndSettle();

    await tester.tap(find.text('Switch Theme Mode'));
    await tester.pump();
    // Advance the animation by half of the custom duration.
    await tester.pump(const Duration(milliseconds: 500));

    // The Scaffold background color is updated.
    expect(getScaffoldMaterial().color, isSameColorAs(const Color(0xff333731)));

    await tester.pumpAndSettle();

    // The Scaffold background color is now fully light.
    expect(getScaffoldMaterial().color, lightTheme.colorScheme.surface);

    // Test the no animation style.
    await tester.tap(find.text('None'));
    await tester.pumpAndSettle();

    await tester.tap(find.text('Switch Theme Mode'));
    // Advance the animation by only one frame.
    await tester.pump();

    // The Scaffold background color is updated immediately.
    expect(getScaffoldMaterial().color, darkTheme.colorScheme.surface);
  });
}
