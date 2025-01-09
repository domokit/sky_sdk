// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_api_samples/material/theme/theme_extension.1.dart' as example;
import 'package:flutter_test/flutter_test.dart';

void main() {
  testWidgets('ThemeExtension can be obtained', (WidgetTester tester) async {
    await tester.pumpWidget(const example.ThemeExtensionExampleApp());

    final ThemeData theme = Theme.of(tester.element(find.byType(example.Home)));
    final example.MyColors colors = theme.extension<example.MyColors>()!;

    expect(colors.brandColor, equals(const Color(0xFF1E88E5)));
    expect(colors.danger, equals(const Color(0xFFE53935)));
  });

  testWidgets('ThemeExtension can be changed', (WidgetTester tester) async {
    await tester.pumpWidget(const example.ThemeExtensionExampleApp());

    ThemeData theme = Theme.of(tester.element(find.byType(example.Home)));
    example.MyColors colors = theme.extension<example.MyColors>()!;

    expect(colors.brandColor, equals(const Color(0xFF1E88E5)));
    expect(colors.danger, equals(const Color(0xFFE53935)));

    // Tap the IconButton to switch theme mode from light to dark.
    await tester.tap(find.byType(IconButton));
    await tester.pumpAndSettle();

    theme = Theme.of(tester.element(find.byType(example.Home)));
    colors = theme.extension<example.MyColors>()!;

    expect(colors.brandColor, equals(const Color(0xFF90CAF9)));
    expect(colors.danger, equals(const Color(0xFFEF9A9A)));
  });

  testWidgets('Home uses MyColors extension correctly', (WidgetTester tester) async {
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData.light().copyWith(
          extensions: <ThemeExtension<dynamic>>[
            const example.MyColors(brandColor: Color(0xFF0000FF), danger: Color(0xFFFF0000)),
          ],
        ),
        home: example.Home(isLightTheme: true, toggleTheme: () {}),
      ),
    );

    expect(
      find.byType(example.Home),
      paints
        ..rect(color: const Color(0xFF0000FF))
        ..rect(color: const Color(0xFFFF0000)),
    );
  });

  testWidgets('Home updates IconButton correctly', (WidgetTester tester) async {
    await tester.pumpWidget(const example.ThemeExtensionExampleApp());

    example.Home home = tester.widget(find.byType(example.Home));
    IconButton iconButton = tester.widget(find.byType(IconButton));
    ThemeData theme = Theme.of(tester.element(find.byType(example.Home)));

    expect(theme.brightness, equals(Brightness.light));
    expect(home.isLightTheme, isTrue);
    expect(
      iconButton.icon,
      isA<Icon>().having((Icon i) => i.icon, 'icon', equals(Icons.nightlight)),
    );

    // Tap the IconButton to switch theme mode from light to dark.
    await tester.tap(find.byType(IconButton));
    await tester.pumpAndSettle();

    home = tester.widget(find.byType(example.Home));
    iconButton = tester.widget(find.byType(IconButton));
    theme = Theme.of(tester.element(find.byType(example.Home)));

    expect(theme.brightness, equals(Brightness.dark));
    expect(home.isLightTheme, isFalse);
    expect(iconButton.icon, isA<Icon>().having((Icon i) => i.icon, 'icon', equals(Icons.wb_sunny)));
  });
}
