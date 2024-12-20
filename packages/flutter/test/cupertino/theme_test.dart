// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/cupertino.dart';
import 'package:flutter/foundation.dart';
import 'package:flutter_test/flutter_test.dart';

int buildCount = 0;
CupertinoThemeData? actualTheme;
IconThemeData? actualIconTheme;

final Widget singletonThemeSubtree = Builder(
  builder: (BuildContext context) {
    buildCount++;
    actualTheme = CupertinoTheme.of(context);
    actualIconTheme = IconTheme.of(context);
    return const Placeholder();
  },
);

Future<CupertinoThemeData> testTheme(WidgetTester tester, CupertinoThemeData theme) async {
  await tester.pumpWidget(CupertinoTheme(data: theme, child: singletonThemeSubtree));
  return actualTheme!;
}

Future<IconThemeData> testIconTheme(WidgetTester tester, CupertinoThemeData theme) async {
  await tester.pumpWidget(CupertinoTheme(data: theme, child: singletonThemeSubtree));
  return actualIconTheme!;
}

void main() {
  setUp(() {
    buildCount = 0;
    actualTheme = null;
    actualIconTheme = null;
  });

  testWidgets('Default theme has defaults', (WidgetTester tester) async {
    final CupertinoThemeData theme = await testTheme(tester, const CupertinoThemeData());

    expect(theme.brightness, isNull);
    expect(theme.primaryColor, CupertinoColors.activeBlue);
    expect(theme.textTheme.textStyle.fontSize, 17.0);
    expect(theme.applyThemeToAll, false);
  });

  testWidgets('Theme attributes cascade', (WidgetTester tester) async {
    final CupertinoThemeData theme = await testTheme(
      tester,
      const CupertinoThemeData(primaryColor: CupertinoColors.systemRed),
    );

    expect(theme.textTheme.actionTextStyle.color, isSameColorAs(CupertinoColors.systemRed.color));
  });

  testWidgets('Dependent attribute can be overridden from cascaded value', (
    WidgetTester tester,
  ) async {
    final CupertinoThemeData theme = await testTheme(
      tester,
      const CupertinoThemeData(
        brightness: Brightness.dark,
        textTheme: CupertinoTextThemeData(textStyle: TextStyle(color: CupertinoColors.black)),
      ),
    );

    // The brightness still cascaded down to the background color.
    expect(theme.scaffoldBackgroundColor, isSameColorAs(CupertinoColors.black));
    // But not to the font color which we overrode.
    expect(theme.textTheme.textStyle.color, isSameColorAs(CupertinoColors.black));
  });

  testWidgets('Reading themes creates dependencies', (WidgetTester tester) async {
    // Reading the theme creates a dependency.
    CupertinoThemeData theme = await testTheme(
      tester,
      const CupertinoThemeData(
        // Default brightness is light,
        barBackgroundColor: Color(0x11223344),
        textTheme: CupertinoTextThemeData(textStyle: TextStyle(fontFamily: 'Skeuomorphic')),
      ),
    );

    expect(buildCount, 1);
    expect(theme.textTheme.textStyle.fontFamily, 'Skeuomorphic');

    // Changing another property also triggers a rebuild.
    theme = await testTheme(
      tester,
      const CupertinoThemeData(
        brightness: Brightness.light,
        barBackgroundColor: Color(0x11223344),
        textTheme: CupertinoTextThemeData(textStyle: TextStyle(fontFamily: 'Skeuomorphic')),
      ),
    );

    expect(buildCount, 2);
    // Re-reading the same value doesn't change anything.
    expect(theme.textTheme.textStyle.fontFamily, 'Skeuomorphic');

    theme = await testTheme(
      tester,
      const CupertinoThemeData(
        brightness: Brightness.light,
        barBackgroundColor: Color(0x11223344),
        textTheme: CupertinoTextThemeData(textStyle: TextStyle(fontFamily: 'Flat')),
      ),
    );

    expect(buildCount, 3);
    expect(theme.textTheme.textStyle.fontFamily, 'Flat');
  });

  testWidgets('copyWith works', (WidgetTester tester) async {
    const CupertinoThemeData originalTheme = CupertinoThemeData(
      brightness: Brightness.dark,
      applyThemeToAll: true,
    );

    final CupertinoThemeData theme = await testTheme(
      tester,
      originalTheme.copyWith(primaryColor: CupertinoColors.systemGreen, applyThemeToAll: false),
    );

    expect(theme.brightness, Brightness.dark);
    expect(theme.primaryColor, isSameColorAs(CupertinoColors.systemGreen.darkColor));
    // Now check calculated derivatives.
    expect(
      theme.textTheme.actionTextStyle.color,
      isSameColorAs(CupertinoColors.systemGreen.darkColor),
    );
    expect(theme.scaffoldBackgroundColor, isSameColorAs(CupertinoColors.black));

    expect(theme.applyThemeToAll, false);
  });

  testWidgets("Theme has default IconThemeData, which is derived from the theme's primary color", (
    WidgetTester tester,
  ) async {
    const CupertinoDynamicColor primaryColor = CupertinoColors.systemRed;
    const CupertinoThemeData themeData = CupertinoThemeData(primaryColor: primaryColor);

    final IconThemeData resultingIconTheme = await testIconTheme(tester, themeData);

    expect(resultingIconTheme.color, isSameColorAs(primaryColor));

    // Works in dark mode if primaryColor is a CupertinoDynamicColor.
    final Color darkColor =
        (await testIconTheme(tester, themeData.copyWith(brightness: Brightness.dark))).color!;

    expect(darkColor, isSameColorAs(primaryColor.darkColor));
  });

  testWidgets('IconTheme.of creates a dependency on iconTheme', (WidgetTester tester) async {
    IconThemeData iconTheme = await testIconTheme(
      tester,
      const CupertinoThemeData(primaryColor: CupertinoColors.destructiveRed),
    );

    expect(buildCount, 1);
    expect(iconTheme.color, CupertinoColors.destructiveRed);

    iconTheme = await testIconTheme(
      tester,
      const CupertinoThemeData(primaryColor: CupertinoColors.activeOrange),
    );
    expect(buildCount, 2);
    expect(iconTheme.color, CupertinoColors.activeOrange);
  });

  testWidgets('CupertinoTheme diagnostics', (WidgetTester tester) async {
    final DiagnosticPropertiesBuilder builder = DiagnosticPropertiesBuilder();
    const CupertinoThemeData().debugFillProperties(builder);

    final Set<String> description =
        builder.properties.map((DiagnosticsNode node) => node.name.toString()).toSet();

    expect(
      setEquals(description, <String>{
        'brightness',
        'primaryColor',
        'primaryContrastingColor',
        'barBackgroundColor',
        'scaffoldBackgroundColor',
        'applyThemeToAll',
        'textStyle',
        'actionTextStyle',
        'actionSmallTextStyle',
        'tabLabelTextStyle',
        'navTitleTextStyle',
        'navLargeTitleTextStyle',
        'navActionTextStyle',
        'pickerTextStyle',
        'dateTimePickerTextStyle',
      }),
      isTrue,
    );
  });

  testWidgets('CupertinoTheme.toStringDeep uses single-line style', (WidgetTester tester) async {
    // Regression test for https://github.com/flutter/flutter/issues/47651.
    expect(
      const CupertinoTheme(
        data: CupertinoThemeData(primaryColor: CupertinoColors.transparent),
        child: SizedBox(),
      ).toStringDeep().trimRight(),
      isNot(contains('\n')),
    );
  });

  testWidgets('CupertinoThemeData equality', (WidgetTester tester) async {
    const CupertinoThemeData a = CupertinoThemeData(brightness: Brightness.dark);
    final CupertinoThemeData b = a.copyWith();
    final CupertinoThemeData c = a.copyWith(brightness: Brightness.light);
    expect(a, equals(b));
    expect(b, equals(a));
    expect(a, isNot(equals(c)));
    expect(c, isNot(equals(a)));
    expect(b, isNot(equals(c)));
    expect(c, isNot(equals(b)));
  });

  late Brightness currentBrightness;
  void colorMatches(Color? componentColor, Color expectedDynamicColor) {
    if (expectedDynamicColor is CupertinoDynamicColor) {
      switch (currentBrightness) {
        case Brightness.light:
          expect(componentColor, isSameColorAs(expectedDynamicColor.color));
        case Brightness.dark:
          expect(componentColor, isSameColorAs(expectedDynamicColor.darkColor));
      }
    } else {
      expect(componentColor, isSameColorAs(expectedDynamicColor));
    }
  }

  void dynamicColorsTestGroup() {
    testWidgets('CupertinoTheme.of resolves colors', (WidgetTester tester) async {
      final CupertinoThemeData data = CupertinoThemeData(
        brightness: currentBrightness,
        primaryColor: CupertinoColors.systemRed,
      );
      final CupertinoThemeData theme = await testTheme(tester, data);

      expect(data.primaryColor, isSameColorAs(CupertinoColors.systemRed));
      colorMatches(theme.primaryColor, CupertinoColors.systemRed);
    });

    testWidgets('CupertinoTheme.of resolves default values', (WidgetTester tester) async {
      const CupertinoDynamicColor primaryColor = CupertinoColors.systemRed;
      final CupertinoThemeData data = CupertinoThemeData(
        brightness: currentBrightness,
        primaryColor: primaryColor,
      );

      const CupertinoDynamicColor barBackgroundColor = CupertinoDynamicColor.withBrightness(
        color: Color(0xF0F9F9F9),
        darkColor: Color(0xF01D1D1D),
      );

      final CupertinoThemeData theme = await testTheme(tester, data);

      colorMatches(theme.primaryContrastingColor, CupertinoColors.white);
      colorMatches(theme.barBackgroundColor, barBackgroundColor);
      colorMatches(theme.scaffoldBackgroundColor, CupertinoColors.systemBackground);
      colorMatches(theme.textTheme.textStyle.color, CupertinoColors.label);
      colorMatches(theme.textTheme.actionTextStyle.color, primaryColor);
      colorMatches(theme.textTheme.tabLabelTextStyle.color, CupertinoColors.inactiveGray);
      colorMatches(theme.textTheme.navTitleTextStyle.color, CupertinoColors.label);
      colorMatches(theme.textTheme.navLargeTitleTextStyle.color, CupertinoColors.label);
      colorMatches(theme.textTheme.navActionTextStyle.color, primaryColor);
      colorMatches(theme.textTheme.pickerTextStyle.color, CupertinoColors.label);
      colorMatches(theme.textTheme.dateTimePickerTextStyle.color, CupertinoColors.label);
    });
  }

  currentBrightness = Brightness.light;
  group('light colors', dynamicColorsTestGroup);

  currentBrightness = Brightness.dark;
  group('dark colors', dynamicColorsTestGroup);
}
