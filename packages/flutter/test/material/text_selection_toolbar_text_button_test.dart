// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  TestWidgetsFlutterBinding.ensureInitialized();

  testWidgets('position in the toolbar changes width', (WidgetTester tester) async {
    late StateSetter setState;
    int index = 1;
    int total = 3;

    await tester.pumpWidget(
      MaterialApp(
        home: Scaffold(
          body: Center(
            child: StatefulBuilder(
              builder: (BuildContext context, StateSetter setter) {
                setState = setter;
                return TextSelectionToolbarTextButton(
                  padding: TextSelectionToolbarTextButton.getPadding(index, total),
                  child: const Text('button'),
                );
              },
            ),
          ),
        ),
      ),
    );

    final Size middleSize = tester.getSize(find.byType(TextSelectionToolbarTextButton));

    setState(() {
      index = 0;
      total = 3;
    });
    await tester.pump();
    final Size firstSize = tester.getSize(find.byType(TextSelectionToolbarTextButton));
    expect(firstSize.width, greaterThan(middleSize.width));

    setState(() {
      index = 2;
      total = 3;
    });
    await tester.pump();
    final Size lastSize = tester.getSize(find.byType(TextSelectionToolbarTextButton));
    expect(lastSize.width, greaterThan(middleSize.width));
    expect(lastSize.width, equals(firstSize.width));

    setState(() {
      index = 0;
      total = 1;
    });
    await tester.pump();
    final Size onlySize = tester.getSize(find.byType(TextSelectionToolbarTextButton));
    expect(onlySize.width, greaterThan(middleSize.width));
    expect(onlySize.width, greaterThan(firstSize.width));
    expect(onlySize.width, greaterThan(lastSize.width));
  });

  for (final ColorScheme colorScheme in <ColorScheme>[
    ThemeData.light().colorScheme,
    ThemeData.dark().colorScheme,
  ]) {
    testWidgets('foreground color by default', (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(colorScheme: colorScheme),
          home: Scaffold(
            body: Center(
              child: TextSelectionToolbarTextButton(
                padding: TextSelectionToolbarTextButton.getPadding(0, 1),
                child: const Text('button'),
              ),
            ),
          ),
        ),
      );

      expect(find.byType(TextButton), findsOneWidget);

      final TextButton textButton = tester.widget(find.byType(TextButton));
      // The foreground color is hardcoded to black or white by default, not the
      // default value from ColorScheme.onSurface.
      expect(textButton.style!.foregroundColor!.resolve(<MaterialState>{}), switch (colorScheme
          .brightness) {
        Brightness.light => const Color(0xff000000),
        Brightness.dark => const Color(0xffffffff),
      });
    });

    testWidgets('custom foreground color', (WidgetTester tester) async {
      const Color customForegroundColor = Colors.red;

      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(colorScheme: colorScheme.copyWith(onSurface: customForegroundColor)),
          home: Scaffold(
            body: Center(
              child: TextSelectionToolbarTextButton(
                padding: TextSelectionToolbarTextButton.getPadding(0, 1),
                child: const Text('button'),
              ),
            ),
          ),
        ),
      );

      expect(find.byType(TextButton), findsOneWidget);

      final TextButton textButton = tester.widget(find.byType(TextButton));
      expect(textButton.style!.foregroundColor!.resolve(<MaterialState>{}), customForegroundColor);
    });

    testWidgets('background color by default', (WidgetTester tester) async {
      // Regression test for https://github.com/flutter/flutter/issues/133027
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(colorScheme: colorScheme),
          home: Scaffold(
            body: Center(
              child: TextSelectionToolbarTextButton(
                padding: TextSelectionToolbarTextButton.getPadding(0, 1),
                child: const Text('button'),
              ),
            ),
          ),
        ),
      );

      expect(find.byType(TextButton), findsOneWidget);

      final TextButton textButton = tester.widget(find.byType(TextButton));
      // The background color is hardcoded to transparent by default so the buttons
      // are the color of the container behind them. For example TextSelectionToolbar
      // hardcodes the color value, and TextSelectionToolbarTextButton that are its
      // children should be that color.
      expect(textButton.style!.backgroundColor!.resolve(<MaterialState>{}), Colors.transparent);
    });

    testWidgets('textButtonTheme should not override default background color', (
      WidgetTester tester,
    ) async {
      // Regression test for https://github.com/flutter/flutter/issues/133027
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(
            colorScheme: colorScheme,
            textButtonTheme: const TextButtonThemeData(
              style: ButtonStyle(backgroundColor: MaterialStatePropertyAll<Color>(Colors.blue)),
            ),
          ),
          home: Scaffold(
            body: Center(
              child: TextSelectionToolbarTextButton(
                padding: TextSelectionToolbarTextButton.getPadding(0, 1),
                child: const Text('button'),
              ),
            ),
          ),
        ),
      );

      expect(find.byType(TextButton), findsOneWidget);

      final TextButton textButton = tester.widget(find.byType(TextButton));
      // The background color is hardcoded to transparent by default so the buttons
      // are the color of the container behind them. For example TextSelectionToolbar
      // hardcodes the color value, and TextSelectionToolbarTextButton that are its
      // children should be that color.
      expect(textButton.style!.backgroundColor!.resolve(<MaterialState>{}), Colors.transparent);
    });
  }
}
