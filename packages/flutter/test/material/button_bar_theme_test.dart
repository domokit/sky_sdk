// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/rendering.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  test('ButtonBarThemeData lerp special cases', () {
    expect(ButtonBarThemeData.lerp(null, null, 0), null);
    const ButtonBarThemeData data = ButtonBarThemeData();
    expect(identical(ButtonBarThemeData.lerp(data, data, 0.5), data), true);
  });

  test('ButtonBarThemeData null fields by default', () {
    const ButtonBarThemeData buttonBarTheme = ButtonBarThemeData();
    expect(buttonBarTheme.alignment, null);
    expect(buttonBarTheme.mainAxisSize, null);
    expect(buttonBarTheme.buttonTextTheme, null);
    expect(buttonBarTheme.buttonMinWidth, null);
    expect(buttonBarTheme.buttonHeight, null);
    expect(buttonBarTheme.buttonPadding, null);
    expect(buttonBarTheme.buttonAlignedDropdown, null);
    expect(buttonBarTheme.layoutBehavior, null);
    expect(buttonBarTheme.overflowDirection, null);
  });

  test('ThemeData uses default ButtonBarThemeData', () {
    expect(ThemeData().buttonBarTheme, equals(const ButtonBarThemeData()));
  });

  test('ButtonBarThemeData copyWith, ==, hashCode basics', () {
    expect(const ButtonBarThemeData(), const ButtonBarThemeData().copyWith());
    expect(const ButtonBarThemeData().hashCode, const ButtonBarThemeData().copyWith().hashCode);
  });

  testWidgets('ButtonBarThemeData lerps correctly', (WidgetTester tester) async {
    const ButtonBarThemeData barThemePrimary = ButtonBarThemeData(
      alignment: MainAxisAlignment.end,
      mainAxisSize: MainAxisSize.min,
      buttonTextTheme: ButtonTextTheme.primary,
      buttonMinWidth: 20.0,
      buttonHeight: 20.0,
      buttonPadding: EdgeInsets.symmetric(vertical: 5.0),
      buttonAlignedDropdown: false,
      layoutBehavior: ButtonBarLayoutBehavior.padded,
      overflowDirection: VerticalDirection.down,
    );
    const ButtonBarThemeData barThemeAccent = ButtonBarThemeData(
      alignment: MainAxisAlignment.center,
      mainAxisSize: MainAxisSize.max,
      buttonTextTheme: ButtonTextTheme.accent,
      buttonMinWidth: 10.0,
      buttonHeight: 40.0,
      buttonPadding: EdgeInsets.symmetric(horizontal: 10.0),
      buttonAlignedDropdown: true,
      layoutBehavior: ButtonBarLayoutBehavior.constrained,
      overflowDirection: VerticalDirection.up,
    );

    final ButtonBarThemeData lerp = ButtonBarThemeData.lerp(barThemePrimary, barThemeAccent, 0.5)!;
    expect(lerp.alignment, equals(MainAxisAlignment.center));
    expect(lerp.mainAxisSize, equals(MainAxisSize.max));
    expect(lerp.buttonTextTheme, equals(ButtonTextTheme.accent));
    expect(lerp.buttonMinWidth, equals(15.0));
    expect(lerp.buttonHeight, equals(30.0));
    expect(lerp.buttonPadding, equals(const EdgeInsets.fromLTRB(5.0, 2.5, 5.0, 2.5)));
    expect(lerp.buttonAlignedDropdown, isTrue);
    expect(lerp.layoutBehavior, equals(ButtonBarLayoutBehavior.constrained));
    expect(lerp.overflowDirection, equals(VerticalDirection.up));
  });

  testWidgets('Default ButtonBarThemeData debugFillProperties', (WidgetTester tester) async {
    final DiagnosticPropertiesBuilder builder = DiagnosticPropertiesBuilder();
    const ButtonBarThemeData().debugFillProperties(builder);

    final List<String> description =
        builder.properties
            .where((DiagnosticsNode node) => !node.isFiltered(DiagnosticLevel.info))
            .map((DiagnosticsNode node) => node.toString())
            .toList();

    expect(description, <String>[]);
  });

  testWidgets('ButtonBarThemeData implements debugFillProperties', (WidgetTester tester) async {
    final DiagnosticPropertiesBuilder builder = DiagnosticPropertiesBuilder();
    const ButtonBarThemeData(
      alignment: MainAxisAlignment.center,
      mainAxisSize: MainAxisSize.max,
      buttonTextTheme: ButtonTextTheme.accent,
      buttonMinWidth: 10.0,
      buttonHeight: 42.0,
      buttonPadding: EdgeInsets.symmetric(horizontal: 7.3),
      buttonAlignedDropdown: true,
      layoutBehavior: ButtonBarLayoutBehavior.constrained,
      overflowDirection: VerticalDirection.up,
    ).debugFillProperties(builder);

    final List<String> description =
        builder.properties
            .where((DiagnosticsNode node) => !node.isFiltered(DiagnosticLevel.info))
            .map((DiagnosticsNode node) => node.toString())
            .toList();

    expect(description, <String>[
      'alignment: MainAxisAlignment.center',
      'mainAxisSize: MainAxisSize.max',
      'textTheme: ButtonTextTheme.accent',
      'minWidth: 10.0',
      'height: 42.0',
      'padding: EdgeInsets(7.3, 0.0, 7.3, 0.0)',
      'dropdown width matches button',
      'layoutBehavior: ButtonBarLayoutBehavior.constrained',
      'overflowDirection: VerticalDirection.up',
    ]);
  });

  testWidgets('ButtonBarTheme.of falls back to ThemeData.buttonBarTheme', (
    WidgetTester tester,
  ) async {
    const ButtonBarThemeData buttonBarTheme = ButtonBarThemeData(buttonMinWidth: 42.0);
    late BuildContext capturedContext;
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(buttonBarTheme: buttonBarTheme),
        home: Builder(
          builder: (BuildContext context) {
            capturedContext = context;
            return Container();
          },
        ),
      ),
    );
    expect(ButtonBarTheme.of(capturedContext), equals(buttonBarTheme));
    expect(ButtonBarTheme.of(capturedContext).buttonMinWidth, equals(42.0));
  });

  testWidgets('ButtonBarTheme overrides ThemeData.buttonBarTheme', (WidgetTester tester) async {
    const ButtonBarThemeData defaultBarTheme = ButtonBarThemeData(buttonMinWidth: 42.0);
    const ButtonBarThemeData buttonBarTheme = ButtonBarThemeData(buttonMinWidth: 84.0);
    late BuildContext capturedContext;
    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(buttonBarTheme: defaultBarTheme),
        home: Builder(
          builder: (BuildContext context) {
            return ButtonBarTheme(
              data: buttonBarTheme,
              child: Builder(
                builder: (BuildContext context) {
                  capturedContext = context;
                  return Container();
                },
              ),
            );
          },
        ),
      ),
    );
    expect(ButtonBarTheme.of(capturedContext), equals(buttonBarTheme));
    expect(ButtonBarTheme.of(capturedContext).buttonMinWidth, equals(84.0));
  });
}
