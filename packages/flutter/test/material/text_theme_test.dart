// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  test('TextTheme copyWith apply, merge basics with const TextTheme()', () {
    expect(const TextTheme(), equals(const TextTheme().copyWith()));
    expect(const TextTheme(), equals(const TextTheme().apply()));
    expect(const TextTheme(), equals(const TextTheme().merge(null)));
    expect(const TextTheme().hashCode, equals(const TextTheme().copyWith().hashCode));
    expect(const TextTheme(), equals(const TextTheme().copyWith()));
  });

  test('TextTheme lerp special cases', () {
    expect(TextTheme.lerp(null, null, 0), const TextTheme());
    const TextTheme theme = TextTheme();
    expect(identical(TextTheme.lerp(theme, theme, 0.5), theme), true);
  });

  test('TextTheme copyWith apply, merge basics with Typography.black', () {
    final Typography typography = Typography.material2018();
    expect(typography.black, equals(typography.black.copyWith()));
    expect(typography.black, equals(typography.black.apply()));
    expect(typography.black, equals(typography.black.merge(null)));
    expect(typography.black, equals(const TextTheme().merge(typography.black)));
    expect(typography.black, equals(typography.black.merge(typography.black)));
    expect(typography.white, equals(typography.black.merge(typography.white)));
    expect(typography.black.hashCode, equals(typography.black.copyWith().hashCode));
    expect(typography.black, isNot(equals(typography.white)));
  });

  test('TextTheme copyWith', () {
    final Typography typography = Typography.material2018();
    final TextTheme whiteCopy = typography.black.copyWith(
      displayLarge: typography.white.displayLarge,
      displayMedium: typography.white.displayMedium,
      displaySmall: typography.white.displaySmall,
      headlineLarge: typography.white.headlineLarge,
      headlineMedium: typography.white.headlineMedium,
      headlineSmall: typography.white.headlineSmall,
      titleLarge: typography.white.titleLarge,
      titleMedium: typography.white.titleMedium,
      titleSmall: typography.white.titleSmall,
      bodyLarge: typography.white.bodyLarge,
      bodyMedium: typography.white.bodyMedium,
      bodySmall: typography.white.bodySmall,
      labelLarge: typography.white.labelLarge,
      labelMedium: typography.white.labelMedium,
      labelSmall: typography.white.labelSmall,
    );
    expect(typography.white, equals(whiteCopy));
  });

  test('TextTheme merges properly in the presence of null fields.', () {
    const TextTheme partialTheme = TextTheme(titleLarge: TextStyle(color: Color(0xcafefeed)));
    final TextTheme fullTheme = ThemeData.fallback().textTheme.merge(partialTheme);
    expect(fullTheme.titleLarge!.color, equals(partialTheme.titleLarge!.color));

    const TextTheme onlyHeadlineSmallAndTitleLarge = TextTheme(
      headlineSmall: TextStyle(color: Color(0xcafefeed)),
      titleLarge: TextStyle(color: Color(0xbeefcafe)),
    );
    const TextTheme onlyBodyMediumAndTitleLarge = TextTheme(
      bodyMedium: TextStyle(color: Color(0xfeedfeed)),
      titleLarge: TextStyle(color: Color(0xdeadcafe)),
    );
    TextTheme merged = onlyHeadlineSmallAndTitleLarge.merge(onlyBodyMediumAndTitleLarge);
    expect(merged.bodyLarge, isNull);
    expect(merged.bodyMedium!.color, equals(onlyBodyMediumAndTitleLarge.bodyMedium!.color));
    expect(
      merged.headlineSmall!.color,
      equals(onlyHeadlineSmallAndTitleLarge.headlineSmall!.color),
    );
    expect(merged.titleLarge!.color, equals(onlyBodyMediumAndTitleLarge.titleLarge!.color));

    merged = onlyHeadlineSmallAndTitleLarge.merge(null);
    expect(merged, equals(onlyHeadlineSmallAndTitleLarge));
  });

  test('TextTheme apply', () {
    // The `displayColor` is applied to [displayLarge], [displayMedium],
    // [displaySmall], [headlineLarge], [headlineMedium], and [bodySmall]. The
    // `bodyColor` is applied to the remaining text styles.
    const Color displayColor = Color(0x00000001);
    const Color bodyColor = Color(0x00000002);
    const String fontFamily = 'fontFamily';
    const List<String> fontFamilyFallback = <String>['font', 'family', 'fallback'];
    const Color decorationColor = Color(0x00000003);
    const TextDecorationStyle decorationStyle = TextDecorationStyle.dashed;
    final TextDecoration decoration = TextDecoration.combine(<TextDecoration>[
      TextDecoration.underline,
      TextDecoration.lineThrough,
    ]);

    final Typography typography = Typography.material2018();
    final TextTheme theme = typography.black.apply(
      fontFamily: fontFamily,
      fontFamilyFallback: fontFamilyFallback,
      displayColor: displayColor,
      bodyColor: bodyColor,
      decoration: decoration,
      decorationColor: decorationColor,
      decorationStyle: decorationStyle,
    );

    expect(theme.displayLarge!.color, displayColor);
    expect(theme.displayMedium!.color, displayColor);
    expect(theme.displaySmall!.color, displayColor);
    expect(theme.headlineLarge!.color, displayColor);
    expect(theme.headlineMedium!.color, displayColor);
    expect(theme.headlineSmall!.color, bodyColor);
    expect(theme.titleLarge!.color, bodyColor);
    expect(theme.titleMedium!.color, bodyColor);
    expect(theme.titleSmall!.color, bodyColor);
    expect(theme.bodyLarge!.color, bodyColor);
    expect(theme.bodyMedium!.color, bodyColor);
    expect(theme.bodySmall!.color, displayColor);
    expect(theme.labelLarge!.color, bodyColor);
    expect(theme.labelMedium!.color, bodyColor);
    expect(theme.labelSmall!.color, bodyColor);

    final List<TextStyle> themeStyles = <TextStyle>[
      theme.displayLarge!,
      theme.displayMedium!,
      theme.displaySmall!,
      theme.headlineLarge!,
      theme.headlineMedium!,
      theme.headlineSmall!,
      theme.titleLarge!,
      theme.titleMedium!,
      theme.titleSmall!,
      theme.bodyLarge!,
      theme.bodyMedium!,
      theme.bodySmall!,
      theme.labelLarge!,
      theme.labelMedium!,
      theme.labelSmall!,
    ];
    expect(themeStyles.every((TextStyle style) => style.fontFamily == fontFamily), true);
    expect(
      themeStyles.every((TextStyle style) => style.fontFamilyFallback == fontFamilyFallback),
      true,
    );
    expect(themeStyles.every((TextStyle style) => style.decorationColor == decorationColor), true);
    expect(themeStyles.every((TextStyle style) => style.decorationStyle == decorationStyle), true);
    expect(themeStyles.every((TextStyle style) => style.decoration == decoration), true);
  });

  test('TextTheme apply fontSizeFactor fontSizeDelta', () {
    final Typography typography = Typography.material2018();
    final TextTheme baseTheme = Typography.englishLike2018.merge(typography.black);
    final TextTheme sizeTheme = baseTheme.apply(fontSizeFactor: 2.0, fontSizeDelta: 5.0);

    expect(sizeTheme.displayLarge!.fontSize, baseTheme.displayLarge!.fontSize! * 2.0 + 5.0);
    expect(sizeTheme.displayMedium!.fontSize, baseTheme.displayMedium!.fontSize! * 2.0 + 5.0);
    expect(sizeTheme.displaySmall!.fontSize, baseTheme.displaySmall!.fontSize! * 2.0 + 5.0);
    expect(sizeTheme.headlineLarge!.fontSize, baseTheme.headlineLarge!.fontSize! * 2.0 + 5.0);
    expect(sizeTheme.headlineMedium!.fontSize, baseTheme.headlineMedium!.fontSize! * 2.0 + 5.0);
    expect(sizeTheme.headlineSmall!.fontSize, baseTheme.headlineSmall!.fontSize! * 2.0 + 5.0);
    expect(sizeTheme.titleLarge!.fontSize, baseTheme.titleLarge!.fontSize! * 2.0 + 5.0);
    expect(sizeTheme.titleMedium!.fontSize, baseTheme.titleMedium!.fontSize! * 2.0 + 5.0);
    expect(sizeTheme.titleSmall!.fontSize, baseTheme.titleSmall!.fontSize! * 2.0 + 5.0);
    expect(sizeTheme.bodyLarge!.fontSize, baseTheme.bodyLarge!.fontSize! * 2.0 + 5.0);
    expect(sizeTheme.bodyMedium!.fontSize, baseTheme.bodyMedium!.fontSize! * 2.0 + 5.0);
    expect(sizeTheme.bodySmall!.fontSize, baseTheme.bodySmall!.fontSize! * 2.0 + 5.0);
    expect(sizeTheme.labelLarge!.fontSize, baseTheme.labelLarge!.fontSize! * 2.0 + 5.0);
    expect(sizeTheme.labelMedium!.fontSize, baseTheme.labelMedium!.fontSize! * 2.0 + 5.0);
    expect(sizeTheme.labelSmall!.fontSize, baseTheme.labelSmall!.fontSize! * 2.0 + 5.0);
  });

  test('TextTheme lerp with second parameter null', () {
    final TextTheme theme = Typography.material2018().black;
    final TextTheme lerped = TextTheme.lerp(theme, null, 0.25);

    expect(lerped.displayLarge, TextStyle.lerp(theme.displayLarge, null, 0.25));
    expect(lerped.displayMedium, TextStyle.lerp(theme.displayMedium, null, 0.25));
    expect(lerped.displaySmall, TextStyle.lerp(theme.displaySmall, null, 0.25));
    expect(lerped.headlineLarge, TextStyle.lerp(theme.headlineLarge, null, 0.25));
    expect(lerped.headlineMedium, TextStyle.lerp(theme.headlineMedium, null, 0.25));
    expect(lerped.headlineSmall, TextStyle.lerp(theme.headlineSmall, null, 0.25));
    expect(lerped.titleLarge, TextStyle.lerp(theme.titleLarge, null, 0.25));
    expect(lerped.titleMedium, TextStyle.lerp(theme.titleMedium, null, 0.25));
    expect(lerped.titleSmall, TextStyle.lerp(theme.titleSmall, null, 0.25));
    expect(lerped.bodyLarge, TextStyle.lerp(theme.bodyLarge, null, 0.25));
    expect(lerped.bodyMedium, TextStyle.lerp(theme.bodyMedium, null, 0.25));
    expect(lerped.bodySmall, TextStyle.lerp(theme.bodySmall, null, 0.25));
    expect(lerped.labelLarge, TextStyle.lerp(theme.labelLarge, null, 0.25));
    expect(lerped.labelMedium, TextStyle.lerp(theme.labelMedium, null, 0.25));
    expect(lerped.labelSmall, TextStyle.lerp(theme.labelSmall, null, 0.25));
  });

  test('TextTheme lerp with first parameter null', () {
    final TextTheme theme = Typography.material2018().black;
    final TextTheme lerped = TextTheme.lerp(null, theme, 0.25);

    expect(lerped.displayLarge, TextStyle.lerp(null, theme.displayLarge, 0.25));
    expect(lerped.displayMedium, TextStyle.lerp(null, theme.displayMedium, 0.25));
    expect(lerped.displaySmall, TextStyle.lerp(null, theme.displaySmall, 0.25));
    expect(lerped.headlineLarge, TextStyle.lerp(null, theme.headlineLarge, 0.25));
    expect(lerped.headlineMedium, TextStyle.lerp(null, theme.headlineMedium, 0.25));
    expect(lerped.headlineSmall, TextStyle.lerp(null, theme.headlineSmall, 0.25));
    expect(lerped.titleLarge, TextStyle.lerp(null, theme.titleLarge, 0.25));
    expect(lerped.titleMedium, TextStyle.lerp(null, theme.titleMedium, 0.25));
    expect(lerped.titleSmall, TextStyle.lerp(null, theme.titleSmall, 0.25));
    expect(lerped.bodyLarge, TextStyle.lerp(null, theme.bodyLarge, 0.25));
    expect(lerped.bodyMedium, TextStyle.lerp(null, theme.bodyMedium, 0.25));
    expect(lerped.bodySmall, TextStyle.lerp(null, theme.bodySmall, 0.25));
    expect(lerped.labelLarge, TextStyle.lerp(null, theme.labelLarge, 0.25));
    expect(lerped.labelMedium, TextStyle.lerp(null, theme.labelMedium, 0.25));
    expect(lerped.labelSmall, TextStyle.lerp(null, theme.labelSmall, 0.25));
  });

  test('TextTheme lerp with null parameters', () {
    final TextTheme lerped = TextTheme.lerp(null, null, 0.25);
    expect(lerped.displayLarge, null);
    expect(lerped.displayMedium, null);
    expect(lerped.displaySmall, null);
    expect(lerped.headlineLarge, null);
    expect(lerped.headlineMedium, null);
    expect(lerped.headlineSmall, null);
    expect(lerped.titleLarge, null);
    expect(lerped.titleMedium, null);
    expect(lerped.titleSmall, null);
    expect(lerped.bodyLarge, null);
    expect(lerped.bodyMedium, null);
    expect(lerped.bodySmall, null);
    expect(lerped.labelLarge, null);
    expect(lerped.labelMedium, null);
    expect(lerped.labelSmall, null);
  });

  test('VisualDensity.lerp', () {
    const VisualDensity a = VisualDensity(horizontal: 1.0, vertical: .5);
    const VisualDensity b = VisualDensity(horizontal: 2.0, vertical: 1.0);

    final VisualDensity noLerp = VisualDensity.lerp(a, b, 0.0);
    expect(noLerp.horizontal, 1.0);
    expect(noLerp.vertical, .5);

    final VisualDensity quarterLerp = VisualDensity.lerp(a, b, .25);
    expect(quarterLerp.horizontal, 1.25);
    expect(quarterLerp.vertical, .625);

    final VisualDensity fullLerp = VisualDensity.lerp(a, b, 1.0);
    expect(fullLerp.horizontal, 2.0);
    expect(fullLerp.vertical, 1.0);
  });

  testWidgets('TextTheme.of(context) is equivalent to Theme.of(context).textTheme', (
    WidgetTester tester,
  ) async {
    const Key sizedBoxKey = Key('sizedBox');

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(
          textTheme: const TextTheme(displayLarge: TextStyle(color: Colors.blue, fontSize: 30.0)),
        ),
        home: const SizedBox(key: sizedBoxKey),
      ),
    );
    final BuildContext context = tester.element(find.byKey(sizedBoxKey));

    final ThemeData themeData = Theme.of(context);
    final TextTheme expectedTextTheme = themeData.textTheme;
    final TextTheme actualTextTheme = TextTheme.of(context);

    expect(actualTextTheme, equals(expectedTextTheme));
  });

  testWidgets('TextTheme.primaryOf(context) is equivalent to Theme.of(context).primaryTextTheme', (
    WidgetTester tester,
  ) async {
    const Key sizedBoxKey = Key('sizedBox');

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData(
          primaryTextTheme: const TextTheme(
            displayLarge: TextStyle(backgroundColor: Colors.green, fontStyle: FontStyle.italic),
          ),
        ),
        home: const SizedBox(key: sizedBoxKey),
      ),
    );

    final BuildContext context = tester.element(find.byKey(sizedBoxKey));
    final ThemeData themeData = Theme.of(context);
    final TextTheme expectedTextTheme = themeData.primaryTextTheme;
    final TextTheme actualTextTheme = TextTheme.primaryOf(context);

    expect(actualTextTheme, equals(expectedTextTheme));
  });
}
