// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  test('Theme data control test', () {
    final ThemeData dark = ThemeData.dark();

    expect(dark, hasOneLineDescription);
    expect(dark, equals(dark.copyWith()));
    expect(dark.hashCode, equals(dark.copyWith().hashCode));

    final ThemeData light = ThemeData.light();
    final ThemeData dawn = ThemeData.lerp(dark, light, 0.25);

    expect(dawn.brightness, Brightness.dark);
    expect(dawn.primaryColor, Color.lerp(dark.primaryColor, light.primaryColor, 0.25));
  });

  test('ThemeData objects with .styleFrom() members are equal', () {
    ThemeData createThemeData() {
      return ThemeData(
        elevatedButtonTheme: ElevatedButtonThemeData(
          style: ElevatedButton.styleFrom(
            foregroundColor: Colors.black,
            backgroundColor: Colors.black,
            elevation: 1.0,
          ),
        ),
        filledButtonTheme: FilledButtonThemeData(
          style: FilledButton.styleFrom(
            foregroundColor: Colors.black,
            disabledForegroundColor: Colors.black,
            backgroundColor: Colors.black,
            disabledBackgroundColor: Colors.black,
            overlayColor: Colors.black,
          ),
        ),
        iconButtonTheme: IconButtonThemeData(
          style: IconButton.styleFrom(
            hoverColor: Colors.black,
            focusColor: Colors.black,
            highlightColor: Colors.black,
          ),
        ),
        textButtonTheme: TextButtonThemeData(
          style: TextButton.styleFrom(
            enabledMouseCursor: MouseCursor.defer,
            disabledMouseCursor: MouseCursor.uncontrolled,
          ),
        ),
      );
    }

    expect(createThemeData() == createThemeData(), isTrue);
  });

  test('Defaults to the default typography for the platform', () {
    for (final TargetPlatform platform in TargetPlatform.values) {
      final ThemeData theme = ThemeData(platform: platform, useMaterial3: false);
      final Typography typography = Typography.material2018(platform: platform);
      expect(
        theme.textTheme,
        typography.black.apply(decoration: TextDecoration.none),
        reason: 'Not using default typography for $platform',
      );
    }
  });

  test('Default text theme contrasts with brightness', () {
    final ThemeData lightTheme = ThemeData(brightness: Brightness.light, useMaterial3: false);
    final ThemeData darkTheme = ThemeData(brightness: Brightness.dark, useMaterial3: false);
    final Typography typography = Typography.material2018(platform: lightTheme.platform);

    expect(lightTheme.textTheme.titleLarge!.color, typography.black.titleLarge!.color);
    expect(darkTheme.textTheme.titleLarge!.color, typography.white.titleLarge!.color);
  });

  test('Default primary text theme contrasts with primary brightness', () {
    final ThemeData lightTheme = ThemeData(primaryColor: Colors.white, useMaterial3: false);
    final ThemeData darkTheme = ThemeData(primaryColor: Colors.black, useMaterial3: false);
    final Typography typography = Typography.material2018(platform: lightTheme.platform);

    expect(lightTheme.primaryTextTheme.titleLarge!.color, typography.black.titleLarge!.color);
    expect(darkTheme.primaryTextTheme.titleLarge!.color, typography.white.titleLarge!.color);
  });

  test('Default icon theme contrasts with brightness', () {
    final ThemeData lightTheme = ThemeData(brightness: Brightness.light, useMaterial3: false);
    final ThemeData darkTheme = ThemeData(brightness: Brightness.dark, useMaterial3: false);
    final Typography typography = Typography.material2018(platform: lightTheme.platform);

    expect(lightTheme.textTheme.titleLarge!.color, typography.black.titleLarge!.color);
    expect(darkTheme.textTheme.titleLarge!.color, typography.white.titleLarge!.color);
  });

  test('Default primary icon theme contrasts with primary brightness', () {
    final ThemeData lightTheme = ThemeData(primaryColor: Colors.white, useMaterial3: false);
    final ThemeData darkTheme = ThemeData(primaryColor: Colors.black, useMaterial3: false);
    final Typography typography = Typography.material2018(platform: lightTheme.platform);

    expect(lightTheme.primaryTextTheme.titleLarge!.color, typography.black.titleLarge!.color);
    expect(darkTheme.primaryTextTheme.titleLarge!.color, typography.white.titleLarge!.color);
  });

  test('light, dark and fallback constructors support useMaterial3', () {
    final ThemeData lightTheme = ThemeData.light(useMaterial3: true);
    expect(lightTheme.useMaterial3, true);
    expect(lightTheme.typography, Typography.material2021(colorScheme: lightTheme.colorScheme));

    final ThemeData darkTheme = ThemeData.dark(useMaterial3: true);
    expect(darkTheme.useMaterial3, true);
    expect(darkTheme.typography, Typography.material2021(colorScheme: darkTheme.colorScheme));

    final ThemeData fallbackTheme = ThemeData.light(useMaterial3: true);
    expect(fallbackTheme.useMaterial3, true);
    expect(
      fallbackTheme.typography,
      Typography.material2021(colorScheme: fallbackTheme.colorScheme),
    );
  });

  testWidgets(
    'Defaults to MaterialTapTargetBehavior.padded on mobile platforms and MaterialTapTargetBehavior.shrinkWrap on desktop',
    (WidgetTester tester) async {
      final ThemeData themeData = ThemeData(platform: defaultTargetPlatform);
      switch (defaultTargetPlatform) {
        case TargetPlatform.android:
        case TargetPlatform.fuchsia:
        case TargetPlatform.iOS:
          expect(themeData.materialTapTargetSize, MaterialTapTargetSize.padded);
        case TargetPlatform.linux:
        case TargetPlatform.macOS:
        case TargetPlatform.windows:
          expect(themeData.materialTapTargetSize, MaterialTapTargetSize.shrinkWrap);
      }
    },
    variant: TargetPlatformVariant.all(),
  );

  test('Can control fontFamily default', () {
    final ThemeData themeData = ThemeData(
      fontFamily: 'FlutterTest',
      textTheme: const TextTheme(titleLarge: TextStyle(fontFamily: 'Roboto')),
    );

    expect(themeData.textTheme.bodyLarge!.fontFamily, equals('FlutterTest'));
    expect(themeData.primaryTextTheme.displaySmall!.fontFamily, equals('FlutterTest'));

    // Shouldn't override the specified style's family
    expect(themeData.textTheme.titleLarge!.fontFamily, equals('Roboto'));
  });

  test('Can estimate brightness - directly', () {
    expect(ThemeData.estimateBrightnessForColor(Colors.white), equals(Brightness.light));
    expect(ThemeData.estimateBrightnessForColor(Colors.black), equals(Brightness.dark));
    expect(ThemeData.estimateBrightnessForColor(Colors.blue), equals(Brightness.dark));
    expect(ThemeData.estimateBrightnessForColor(Colors.yellow), equals(Brightness.light));
    expect(ThemeData.estimateBrightnessForColor(Colors.deepOrange), equals(Brightness.dark));
    expect(ThemeData.estimateBrightnessForColor(Colors.orange), equals(Brightness.light));
    expect(ThemeData.estimateBrightnessForColor(Colors.lime), equals(Brightness.light));
    expect(ThemeData.estimateBrightnessForColor(Colors.grey), equals(Brightness.light));
    expect(ThemeData.estimateBrightnessForColor(Colors.teal), equals(Brightness.dark));
    expect(ThemeData.estimateBrightnessForColor(Colors.indigo), equals(Brightness.dark));
  });

  test('cursorColor', () {
    expect(const TextSelectionThemeData(cursorColor: Colors.red).cursorColor, Colors.red);
  });

  test('If colorSchemeSeed is used colorScheme, primaryColor and primarySwatch should not be.', () {
    expect(
      () => ThemeData(colorSchemeSeed: Colors.blue, colorScheme: const ColorScheme.light()),
      throwsAssertionError,
    );
    expect(
      () => ThemeData(colorSchemeSeed: Colors.blue, primaryColor: Colors.green),
      throwsAssertionError,
    );
    expect(
      () => ThemeData(colorSchemeSeed: Colors.blue, primarySwatch: Colors.green),
      throwsAssertionError,
    );
  });

  test('ThemeData can generate a light colorScheme from colorSchemeSeed', () {
    final ThemeData theme = ThemeData(colorSchemeSeed: Colors.blue);

    expect(theme.colorScheme.primary, const Color(0xff36618e));
    expect(theme.colorScheme.onPrimary, const Color(0xffffffff));
    expect(theme.colorScheme.primaryContainer, const Color(0xffd1e4ff));
    expect(theme.colorScheme.onPrimaryContainer, const Color(0xff001d36));
    expect(theme.colorScheme.primaryFixed, const Color(0xffd1e4ff));
    expect(theme.colorScheme.primaryFixedDim, const Color(0xffa0cafd));
    expect(theme.colorScheme.onPrimaryFixed, const Color(0xff001d36));
    expect(theme.colorScheme.onPrimaryFixedVariant, const Color(0xff194975));
    expect(theme.colorScheme.secondary, const Color(0xff535f70));
    expect(theme.colorScheme.onSecondary, const Color(0xffffffff));
    expect(theme.colorScheme.secondaryContainer, const Color(0xffd7e3f7));
    expect(theme.colorScheme.onSecondaryContainer, const Color(0xff101c2b));
    expect(theme.colorScheme.secondaryFixed, const Color(0xffd7e3f7));
    expect(theme.colorScheme.secondaryFixedDim, const Color(0xffbbc7db));
    expect(theme.colorScheme.onSecondaryFixed, const Color(0xff101c2b));
    expect(theme.colorScheme.onSecondaryFixedVariant, const Color(0xff3b4858));
    expect(theme.colorScheme.tertiary, const Color(0xff6b5778));
    expect(theme.colorScheme.onTertiary, const Color(0xffffffff));
    expect(theme.colorScheme.tertiaryContainer, const Color(0xfff2daff));
    expect(theme.colorScheme.onTertiaryContainer, const Color(0xff251431));
    expect(theme.colorScheme.tertiaryFixed, const Color(0xfff2daff));
    expect(theme.colorScheme.tertiaryFixedDim, const Color(0xffd6bee4));
    expect(theme.colorScheme.onTertiaryFixed, const Color(0xff251431));
    expect(theme.colorScheme.onTertiaryFixedVariant, const Color(0xff523f5f));
    expect(theme.colorScheme.error, const Color(0xffba1a1a));
    expect(theme.colorScheme.onError, const Color(0xffffffff));
    expect(theme.colorScheme.errorContainer, const Color(0xffffdad6));
    expect(theme.colorScheme.onErrorContainer, const Color(0xff410002));
    expect(theme.colorScheme.outline, const Color(0xff73777f));
    expect(theme.colorScheme.outlineVariant, const Color(0xffc3c7cf));
    expect(theme.colorScheme.background, const Color(0xfff8f9ff));
    expect(theme.colorScheme.onBackground, const Color(0xff191c20));
    expect(theme.colorScheme.surface, const Color(0xfff8f9ff));
    expect(theme.colorScheme.surfaceBright, const Color(0xfff8f9ff));
    expect(theme.colorScheme.surfaceDim, const Color(0xffd8dae0));
    expect(theme.colorScheme.surfaceContainerLowest, const Color(0xffffffff));
    expect(theme.colorScheme.surfaceContainerLow, const Color(0xfff2f3fa));
    expect(theme.colorScheme.surfaceContainer, const Color(0xffeceef4));
    expect(theme.colorScheme.surfaceContainerHigh, const Color(0xffe6e8ee));
    expect(theme.colorScheme.surfaceContainerHighest, const Color(0xffe1e2e8));
    expect(theme.colorScheme.onSurface, const Color(0xff191c20));
    expect(theme.colorScheme.surfaceVariant, const Color(0xffdfe2eb));
    expect(theme.colorScheme.onSurfaceVariant, const Color(0xff43474e));
    expect(theme.colorScheme.inverseSurface, const Color(0xff2e3135));
    expect(theme.colorScheme.onInverseSurface, const Color(0xffeff0f7));
    expect(theme.colorScheme.inversePrimary, const Color(0xffa0cafd));
    expect(theme.colorScheme.shadow, const Color(0xff000000));
    expect(theme.colorScheme.scrim, const Color(0xff000000));
    expect(theme.colorScheme.surfaceTint, const Color(0xff36618e));
    expect(theme.colorScheme.brightness, Brightness.light);

    expect(theme.primaryColor, theme.colorScheme.primary);
    expect(theme.canvasColor, theme.colorScheme.surface);
    expect(theme.scaffoldBackgroundColor, theme.colorScheme.surface);
    expect(theme.cardColor, theme.colorScheme.surface);
    expect(theme.dividerColor, theme.colorScheme.outline);
    expect(theme.dialogBackgroundColor, theme.colorScheme.surface);
    expect(theme.indicatorColor, theme.colorScheme.onPrimary);
    expect(theme.applyElevationOverlayColor, false);
  });

  test('ThemeData can generate a dark colorScheme from colorSchemeSeed', () {
    final ThemeData theme = ThemeData(colorSchemeSeed: Colors.blue, brightness: Brightness.dark);

    expect(theme.colorScheme.primary, const Color(0xffa0cafd));
    expect(theme.colorScheme.onPrimary, const Color(0xff003258));
    expect(theme.colorScheme.primaryContainer, const Color(0xff194975));
    expect(theme.colorScheme.onPrimaryContainer, const Color(0xffd1e4ff));
    expect(theme.colorScheme.primaryFixed, const Color(0xffd1e4ff));
    expect(theme.colorScheme.primaryFixedDim, const Color(0xffa0cafd));
    expect(theme.colorScheme.onPrimaryFixed, const Color(0xff001d36));
    expect(theme.colorScheme.onPrimaryFixedVariant, const Color(0xff194975));
    expect(theme.colorScheme.secondary, const Color(0xffbbc7db));
    expect(theme.colorScheme.onSecondary, const Color(0xff253140));
    expect(theme.colorScheme.secondaryContainer, const Color(0xff3b4858));
    expect(theme.colorScheme.onSecondaryContainer, const Color(0xffd7e3f7));
    expect(theme.colorScheme.secondaryFixed, const Color(0xffd7e3f7));
    expect(theme.colorScheme.secondaryFixedDim, const Color(0xffbbc7db));
    expect(theme.colorScheme.onSecondaryFixed, const Color(0xff101c2b));
    expect(theme.colorScheme.onSecondaryFixedVariant, const Color(0xff3b4858));
    expect(theme.colorScheme.tertiary, const Color(0xffd6bee4));
    expect(theme.colorScheme.onTertiary, const Color(0xff3b2948));
    expect(theme.colorScheme.tertiaryContainer, const Color(0xff523f5f));
    expect(theme.colorScheme.onTertiaryContainer, const Color(0xfff2daff));
    expect(theme.colorScheme.tertiaryFixed, const Color(0xfff2daff));
    expect(theme.colorScheme.tertiaryFixedDim, const Color(0xffd6bee4));
    expect(theme.colorScheme.onTertiaryFixed, const Color(0xff251431));
    expect(theme.colorScheme.onTertiaryFixedVariant, const Color(0xff523f5f));
    expect(theme.colorScheme.error, const Color(0xffffb4ab));
    expect(theme.colorScheme.onError, const Color(0xff690005));
    expect(theme.colorScheme.errorContainer, const Color(0xff93000a));
    expect(theme.colorScheme.onErrorContainer, const Color(0xffffdad6));
    expect(theme.colorScheme.outline, const Color(0xff8d9199));
    expect(theme.colorScheme.outlineVariant, const Color(0xff43474e));
    expect(theme.colorScheme.background, const Color(0xff111418));
    expect(theme.colorScheme.onBackground, const Color(0xffe1e2e8));
    expect(theme.colorScheme.surface, const Color(0xff111418));
    expect(theme.colorScheme.surfaceDim, const Color(0xff111418));
    expect(theme.colorScheme.surfaceBright, const Color(0xff36393e));
    expect(theme.colorScheme.surfaceContainerLowest, const Color(0xff0b0e13));
    expect(theme.colorScheme.surfaceContainerLow, const Color(0xff191c20));
    expect(theme.colorScheme.surfaceContainer, const Color(0xff1d2024));
    expect(theme.colorScheme.surfaceContainerHigh, const Color(0xff272a2f));
    expect(theme.colorScheme.surfaceContainerHighest, const Color(0xff32353a));
    expect(theme.colorScheme.onSurface, const Color(0xffe1e2e8));
    expect(theme.colorScheme.surfaceVariant, const Color(0xff43474e));
    expect(theme.colorScheme.onSurfaceVariant, const Color(0xffc3c7cf));
    expect(theme.colorScheme.inverseSurface, const Color(0xffe1e2e8));
    expect(theme.colorScheme.onInverseSurface, const Color(0xff2e3135));
    expect(theme.colorScheme.inversePrimary, const Color(0xff36618e));
    expect(theme.colorScheme.shadow, const Color(0xff000000));
    expect(theme.colorScheme.scrim, const Color(0xff000000));
    expect(theme.colorScheme.surfaceTint, const Color(0xffa0cafd));
    expect(theme.colorScheme.brightness, Brightness.dark);

    expect(theme.primaryColor, theme.colorScheme.surface);
    expect(theme.canvasColor, theme.colorScheme.surface);
    expect(theme.scaffoldBackgroundColor, theme.colorScheme.surface);
    expect(theme.cardColor, theme.colorScheme.surface);
    expect(theme.dividerColor, theme.colorScheme.outline);
    expect(theme.dialogBackgroundColor, theme.colorScheme.surface);
    expect(theme.indicatorColor, theme.colorScheme.onSurface);
    expect(theme.applyElevationOverlayColor, true);
  });

  test('ThemeData can generate a default M3 light colorScheme when useMaterial3 is true', () {
    final ThemeData theme = ThemeData(useMaterial3: true);

    expect(theme.colorScheme.primary, const Color(0xff6750a4));
    expect(theme.colorScheme.onPrimary, const Color(0xffffffff));
    expect(theme.colorScheme.primaryContainer, const Color(0xffeaddff));
    expect(theme.colorScheme.onPrimaryContainer, const Color(0xff4f378b));
    expect(theme.colorScheme.primaryFixed, const Color(0xffeaddff));
    expect(theme.colorScheme.primaryFixedDim, const Color(0xffd0bcff));
    expect(theme.colorScheme.onPrimaryFixed, const Color(0xff21005d));
    expect(theme.colorScheme.onPrimaryFixedVariant, const Color(0xff4f378b));
    expect(theme.colorScheme.secondary, const Color(0xff625b71));
    expect(theme.colorScheme.onSecondary, const Color(0xffffffff));
    expect(theme.colorScheme.secondaryContainer, const Color(0xffe8def8));
    expect(theme.colorScheme.onSecondaryContainer, const Color(0xff4a4458));
    expect(theme.colorScheme.secondaryFixed, const Color(0xffe8def8));
    expect(theme.colorScheme.secondaryFixedDim, const Color(0xffccc2dc));
    expect(theme.colorScheme.onSecondaryFixed, const Color(0xff1d192b));
    expect(theme.colorScheme.onSecondaryFixedVariant, const Color(0xff4a4458));
    expect(theme.colorScheme.tertiary, const Color(0xff7d5260));
    expect(theme.colorScheme.onTertiary, const Color(0xffffffff));
    expect(theme.colorScheme.tertiaryContainer, const Color(0xffffd8e4));
    expect(theme.colorScheme.onTertiaryContainer, const Color(0xff633b48));
    expect(theme.colorScheme.tertiaryFixed, const Color(0xffffd8e4));
    expect(theme.colorScheme.tertiaryFixedDim, const Color(0xffefb8c8));
    expect(theme.colorScheme.onTertiaryFixed, const Color(0xff31111d));
    expect(theme.colorScheme.onTertiaryFixedVariant, const Color(0xff633b48));
    expect(theme.colorScheme.error, const Color(0xffb3261e));
    expect(theme.colorScheme.onError, const Color(0xffffffff));
    expect(theme.colorScheme.errorContainer, const Color(0xfff9dedc));
    expect(theme.colorScheme.onErrorContainer, const Color(0xff8c1d18));
    expect(theme.colorScheme.outline, const Color(0xff79747e));
    expect(theme.colorScheme.background, const Color(0xfffef7ff));
    expect(theme.colorScheme.onBackground, const Color(0xff1d1b20));
    expect(theme.colorScheme.surface, const Color(0xfffef7ff));
    expect(theme.colorScheme.onSurface, const Color(0xff1d1b20));
    expect(theme.colorScheme.surfaceVariant, const Color(0xffe7e0ec));
    expect(theme.colorScheme.onSurfaceVariant, const Color(0xff49454f));
    expect(theme.colorScheme.surfaceBright, const Color(0xfffef7ff));
    expect(theme.colorScheme.surfaceDim, const Color(0xffded8e1));
    expect(theme.colorScheme.surfaceContainer, const Color(0xfff3edf7));
    expect(theme.colorScheme.surfaceContainerHighest, const Color(0xffe6e0e9));
    expect(theme.colorScheme.surfaceContainerHigh, const Color(0xffece6f0));
    expect(theme.colorScheme.surfaceContainerLowest, const Color(0xffffffff));
    expect(theme.colorScheme.surfaceContainerLow, const Color(0xfff7f2fa));
    expect(theme.colorScheme.inverseSurface, const Color(0xff322f35));
    expect(theme.colorScheme.onInverseSurface, const Color(0xfff5eff7));
    expect(theme.colorScheme.inversePrimary, const Color(0xffd0bcff));
    expect(theme.colorScheme.shadow, const Color(0xff000000));
    expect(theme.colorScheme.surfaceTint, const Color(0xff6750a4));
    expect(theme.colorScheme.brightness, Brightness.light);

    expect(theme.primaryColor, theme.colorScheme.primary);
    expect(theme.canvasColor, theme.colorScheme.surface);
    expect(theme.scaffoldBackgroundColor, theme.colorScheme.surface);
    expect(theme.cardColor, theme.colorScheme.surface);
    expect(theme.dividerColor, theme.colorScheme.outline);
    expect(theme.dialogBackgroundColor, theme.colorScheme.surface);
    expect(theme.indicatorColor, theme.colorScheme.onPrimary);
    expect(theme.applyElevationOverlayColor, false);
  });

  test(
    'ThemeData.light() can generate a default M3 light colorScheme when useMaterial3 is true',
    () {
      final ThemeData theme = ThemeData.light(useMaterial3: true);

      expect(theme.colorScheme.primary, const Color(0xff6750a4));
      expect(theme.colorScheme.onPrimary, const Color(0xffffffff));
      expect(theme.colorScheme.primaryContainer, const Color(0xffeaddff));
      expect(theme.colorScheme.onPrimaryContainer, const Color(0xff4f378b));
      expect(theme.colorScheme.primaryFixed, const Color(0xffeaddff));
      expect(theme.colorScheme.primaryFixedDim, const Color(0xffd0bcff));
      expect(theme.colorScheme.onPrimaryFixed, const Color(0xff21005d));
      expect(theme.colorScheme.onPrimaryFixedVariant, const Color(0xff4f378b));
      expect(theme.colorScheme.secondary, const Color(0xff625b71));
      expect(theme.colorScheme.onSecondary, const Color(0xffffffff));
      expect(theme.colorScheme.secondaryContainer, const Color(0xffe8def8));
      expect(theme.colorScheme.onSecondaryContainer, const Color(0xff4a4458));
      expect(theme.colorScheme.secondaryFixed, const Color(0xffe8def8));
      expect(theme.colorScheme.secondaryFixedDim, const Color(0xffccc2dc));
      expect(theme.colorScheme.onSecondaryFixed, const Color(0xff1d192b));
      expect(theme.colorScheme.onSecondaryFixedVariant, const Color(0xff4a4458));
      expect(theme.colorScheme.tertiary, const Color(0xff7d5260));
      expect(theme.colorScheme.onTertiary, const Color(0xffffffff));
      expect(theme.colorScheme.tertiaryContainer, const Color(0xffffd8e4));
      expect(theme.colorScheme.onTertiaryContainer, const Color(0xff633b48));
      expect(theme.colorScheme.tertiaryFixed, const Color(0xffffd8e4));
      expect(theme.colorScheme.tertiaryFixedDim, const Color(0xffefb8c8));
      expect(theme.colorScheme.onTertiaryFixed, const Color(0xff31111d));
      expect(theme.colorScheme.onTertiaryFixedVariant, const Color(0xff633b48));
      expect(theme.colorScheme.error, const Color(0xffb3261e));
      expect(theme.colorScheme.onError, const Color(0xffffffff));
      expect(theme.colorScheme.errorContainer, const Color(0xfff9dedc));
      expect(theme.colorScheme.onErrorContainer, const Color(0xff8c1d18));
      expect(theme.colorScheme.outline, const Color(0xff79747e));
      expect(theme.colorScheme.background, const Color(0xfffef7ff));
      expect(theme.colorScheme.onBackground, const Color(0xff1d1b20));
      expect(theme.colorScheme.surface, const Color(0xfffef7ff));
      expect(theme.colorScheme.onSurface, const Color(0xff1d1b20));
      expect(theme.colorScheme.surfaceVariant, const Color(0xffe7e0ec));
      expect(theme.colorScheme.onSurfaceVariant, const Color(0xff49454f));
      expect(theme.colorScheme.surfaceBright, const Color(0xfffef7ff));
      expect(theme.colorScheme.surfaceDim, const Color(0xffded8e1));
      expect(theme.colorScheme.surfaceContainer, const Color(0xfff3edf7));
      expect(theme.colorScheme.surfaceContainerHighest, const Color(0xffe6e0e9));
      expect(theme.colorScheme.surfaceContainerHigh, const Color(0xffece6f0));
      expect(theme.colorScheme.surfaceContainerLowest, const Color(0xffffffff));
      expect(theme.colorScheme.surfaceContainerLow, const Color(0xfff7f2fa));
      expect(theme.colorScheme.inverseSurface, const Color(0xff322f35));
      expect(theme.colorScheme.onInverseSurface, const Color(0xfff5eff7));
      expect(theme.colorScheme.inversePrimary, const Color(0xffd0bcff));
      expect(theme.colorScheme.shadow, const Color(0xff000000));
      expect(theme.colorScheme.surfaceTint, const Color(0xff6750a4));
      expect(theme.colorScheme.brightness, Brightness.light);

      expect(theme.primaryColor, theme.colorScheme.primary);
      expect(theme.canvasColor, theme.colorScheme.surface);
      expect(theme.scaffoldBackgroundColor, theme.colorScheme.surface);
      expect(theme.cardColor, theme.colorScheme.surface);
      expect(theme.dividerColor, theme.colorScheme.outline);
      expect(theme.dialogBackgroundColor, theme.colorScheme.surface);
      expect(theme.indicatorColor, theme.colorScheme.onPrimary);
      expect(theme.applyElevationOverlayColor, false);
    },
  );

  test('ThemeData.dark() can generate a default M3 dark colorScheme when useMaterial3 is true', () {
    final ThemeData theme = ThemeData.dark(useMaterial3: true);
    expect(theme.colorScheme.primary, const Color(0xffd0bcff));
    expect(theme.colorScheme.onPrimary, const Color(0xff381e72));
    expect(theme.colorScheme.primaryContainer, const Color(0xff4f378b));
    expect(theme.colorScheme.onPrimaryContainer, const Color(0xffeaddff));
    expect(theme.colorScheme.primaryFixed, const Color(0xffeaddff));
    expect(theme.colorScheme.primaryFixedDim, const Color(0xffd0bcff));
    expect(theme.colorScheme.onPrimaryFixed, const Color(0xff21005d));
    expect(theme.colorScheme.onPrimaryFixedVariant, const Color(0xff4f378b));
    expect(theme.colorScheme.secondary, const Color(0xffccc2dc));
    expect(theme.colorScheme.onSecondary, const Color(0xff332d41));
    expect(theme.colorScheme.secondaryContainer, const Color(0xff4a4458));
    expect(theme.colorScheme.onSecondaryContainer, const Color(0xffe8def8));
    expect(theme.colorScheme.secondaryFixed, const Color(0xffe8def8));
    expect(theme.colorScheme.secondaryFixedDim, const Color(0xffccc2dc));
    expect(theme.colorScheme.onSecondaryFixed, const Color(0xff1d192b));
    expect(theme.colorScheme.onSecondaryFixedVariant, const Color(0xff4a4458));
    expect(theme.colorScheme.tertiary, const Color(0xffefb8c8));
    expect(theme.colorScheme.onTertiary, const Color(0xff492532));
    expect(theme.colorScheme.tertiaryContainer, const Color(0xff633b48));
    expect(theme.colorScheme.onTertiaryContainer, const Color(0xffffd8e4));
    expect(theme.colorScheme.tertiaryFixed, const Color(0xffffd8e4));
    expect(theme.colorScheme.tertiaryFixedDim, const Color(0xffefb8c8));
    expect(theme.colorScheme.onTertiaryFixed, const Color(0xff31111d));
    expect(theme.colorScheme.onTertiaryFixedVariant, const Color(0xff633b48));
    expect(theme.colorScheme.error, const Color(0xfff2b8b5));
    expect(theme.colorScheme.onError, const Color(0xff601410));
    expect(theme.colorScheme.errorContainer, const Color(0xff8c1d18));
    expect(theme.colorScheme.onErrorContainer, const Color(0xfff9dedc));
    expect(theme.colorScheme.outline, const Color(0xff938f99));
    expect(theme.colorScheme.background, const Color(0xff141218));
    expect(theme.colorScheme.onBackground, const Color(0xffe6e0e9));
    expect(theme.colorScheme.surface, const Color(0xff141218));
    expect(theme.colorScheme.onSurface, const Color(0xffe6e0e9));
    expect(theme.colorScheme.surfaceVariant, const Color(0xff49454f));
    expect(theme.colorScheme.onSurfaceVariant, const Color(0xffcac4d0));
    expect(theme.colorScheme.surfaceBright, const Color(0xff3b383e));
    expect(theme.colorScheme.surfaceDim, const Color(0xff141218));
    expect(theme.colorScheme.surfaceContainer, const Color(0xff211f26));
    expect(theme.colorScheme.surfaceContainerHighest, const Color(0xff36343b));
    expect(theme.colorScheme.surfaceContainerHigh, const Color(0xff2b2930));
    expect(theme.colorScheme.surfaceContainerLowest, const Color(0xff0f0d13));
    expect(theme.colorScheme.surfaceContainerLow, const Color(0xff1d1b20));
    expect(theme.colorScheme.inverseSurface, const Color(0xffe6e0e9));
    expect(theme.colorScheme.onInverseSurface, const Color(0xff322f35));
    expect(theme.colorScheme.inversePrimary, const Color(0xff6750a4));
    expect(theme.colorScheme.shadow, const Color(0xff000000));
    expect(theme.colorScheme.surfaceTint, const Color(0xffd0bcff));
    expect(theme.colorScheme.brightness, Brightness.dark);

    expect(theme.primaryColor, theme.colorScheme.surface);
    expect(theme.canvasColor, theme.colorScheme.surface);
    expect(theme.scaffoldBackgroundColor, theme.colorScheme.surface);
    expect(theme.cardColor, theme.colorScheme.surface);
    expect(theme.dividerColor, theme.colorScheme.outline);
    expect(theme.dialogBackgroundColor, theme.colorScheme.surface);
    expect(theme.indicatorColor, theme.colorScheme.onSurface);
    expect(theme.applyElevationOverlayColor, true);
  });

  testWidgets('ThemeData.from a light color scheme sets appropriate values', (
    WidgetTester tester,
  ) async {
    const ColorScheme lightColors = ColorScheme.light();
    final ThemeData theme = ThemeData.from(colorScheme: lightColors);

    expect(theme.brightness, equals(Brightness.light));
    expect(theme.primaryColor, equals(lightColors.primary));
    expect(theme.cardColor, equals(lightColors.surface));
    expect(theme.canvasColor, equals(lightColors.surface));
    expect(theme.scaffoldBackgroundColor, equals(lightColors.surface));
    expect(theme.dialogBackgroundColor, equals(lightColors.surface));
    expect(theme.applyElevationOverlayColor, isFalse);
  });

  testWidgets('ThemeData.from a dark color scheme sets appropriate values', (
    WidgetTester tester,
  ) async {
    const ColorScheme darkColors = ColorScheme.dark();
    final ThemeData theme = ThemeData.from(colorScheme: darkColors);

    expect(theme.brightness, equals(Brightness.dark));
    // in dark theme's the color used for main components is surface instead of primary
    expect(theme.primaryColor, equals(darkColors.surface));
    expect(theme.cardColor, equals(darkColors.surface));
    expect(theme.canvasColor, equals(darkColors.surface));
    expect(theme.scaffoldBackgroundColor, equals(darkColors.surface));
    expect(theme.dialogBackgroundColor, equals(darkColors.surface));
    expect(theme.applyElevationOverlayColor, isTrue);
  });

  testWidgets(
    'splashFactory is InkSparkle only for Android non-web when useMaterial3 is true',
    (WidgetTester tester) async {
      final ThemeData theme = ThemeData(useMaterial3: true);

      // Basic check that this theme is in fact using material 3.
      expect(theme.useMaterial3, true);

      switch (debugDefaultTargetPlatformOverride!) {
        case TargetPlatform.android:
          if (kIsWeb) {
            expect(theme.splashFactory, equals(InkRipple.splashFactory));
          } else {
            expect(theme.splashFactory, equals(InkSparkle.splashFactory));
          }
        case TargetPlatform.iOS:
        case TargetPlatform.fuchsia:
        case TargetPlatform.linux:
        case TargetPlatform.macOS:
        case TargetPlatform.windows:
          expect(theme.splashFactory, equals(InkRipple.splashFactory));
      }
    },
    variant: TargetPlatformVariant.all(),
  );

  testWidgets(
    'splashFactory is InkSplash for every platform scenario, including Android non-web, when useMaterial3 is false',
    (WidgetTester tester) async {
      final ThemeData theme = ThemeData(useMaterial3: false);

      switch (debugDefaultTargetPlatformOverride!) {
        case TargetPlatform.android:
        case TargetPlatform.iOS:
        case TargetPlatform.fuchsia:
        case TargetPlatform.linux:
        case TargetPlatform.macOS:
        case TargetPlatform.windows:
          expect(theme.splashFactory, equals(InkSplash.splashFactory));
      }
    },
    variant: TargetPlatformVariant.all(),
  );

  testWidgets(
    'VisualDensity.adaptivePlatformDensity returns adaptive values',
    (WidgetTester tester) async {
      switch (debugDefaultTargetPlatformOverride!) {
        case TargetPlatform.android:
        case TargetPlatform.iOS:
        case TargetPlatform.fuchsia:
          expect(VisualDensity.adaptivePlatformDensity, equals(VisualDensity.standard));
        case TargetPlatform.linux:
        case TargetPlatform.macOS:
        case TargetPlatform.windows:
          expect(VisualDensity.adaptivePlatformDensity, equals(VisualDensity.compact));
      }
    },
    variant: TargetPlatformVariant.all(),
  );

  testWidgets(
    'VisualDensity.getDensityForPlatform returns adaptive values',
    (WidgetTester tester) async {
      switch (debugDefaultTargetPlatformOverride!) {
        case TargetPlatform.android:
        case TargetPlatform.iOS:
        case TargetPlatform.fuchsia:
          expect(
            VisualDensity.defaultDensityForPlatform(debugDefaultTargetPlatformOverride!),
            equals(VisualDensity.standard),
          );
        case TargetPlatform.linux:
        case TargetPlatform.macOS:
        case TargetPlatform.windows:
          expect(
            VisualDensity.defaultDensityForPlatform(debugDefaultTargetPlatformOverride!),
            equals(VisualDensity.compact),
          );
      }
    },
    variant: TargetPlatformVariant.all(),
  );

  testWidgets(
    'VisualDensity in ThemeData defaults to "compact" on desktop and "standard" on mobile',
    (WidgetTester tester) async {
      final ThemeData themeData = ThemeData();
      switch (debugDefaultTargetPlatformOverride!) {
        case TargetPlatform.android:
        case TargetPlatform.iOS:
        case TargetPlatform.fuchsia:
          expect(themeData.visualDensity, equals(VisualDensity.standard));
        case TargetPlatform.linux:
        case TargetPlatform.macOS:
        case TargetPlatform.windows:
          expect(themeData.visualDensity, equals(VisualDensity.compact));
      }
    },
    variant: TargetPlatformVariant.all(),
  );

  testWidgets(
    'VisualDensity in ThemeData defaults to the right thing when a platform is supplied to it',
    (WidgetTester tester) async {
      final ThemeData themeData = ThemeData(
        platform:
            debugDefaultTargetPlatformOverride! == TargetPlatform.android
                ? TargetPlatform.linux
                : TargetPlatform.android,
      );
      switch (debugDefaultTargetPlatformOverride!) {
        case TargetPlatform.iOS:
        case TargetPlatform.fuchsia:
        case TargetPlatform.linux:
        case TargetPlatform.macOS:
        case TargetPlatform.windows:
          expect(themeData.visualDensity, equals(VisualDensity.standard));
        case TargetPlatform.android:
          expect(themeData.visualDensity, equals(VisualDensity.compact));
      }
    },
    variant: TargetPlatformVariant.all(),
  );

  testWidgets('Ensure Visual Density effective constraints are clamped', (
    WidgetTester tester,
  ) async {
    const BoxConstraints square = BoxConstraints.tightFor(width: 35, height: 35);
    BoxConstraints expanded = const VisualDensity(
      horizontal: 4.0,
      vertical: 4.0,
    ).effectiveConstraints(square);
    expect(expanded.minWidth, equals(35));
    expect(expanded.minHeight, equals(35));
    expect(expanded.maxWidth, equals(35));
    expect(expanded.maxHeight, equals(35));

    BoxConstraints contracted = const VisualDensity(
      horizontal: -4.0,
      vertical: -4.0,
    ).effectiveConstraints(square);
    expect(contracted.minWidth, equals(19));
    expect(contracted.minHeight, equals(19));
    expect(expanded.maxWidth, equals(35));
    expect(expanded.maxHeight, equals(35));

    const BoxConstraints small = BoxConstraints.tightFor(width: 4, height: 4);
    expanded = const VisualDensity(horizontal: 4.0, vertical: 4.0).effectiveConstraints(small);
    expect(expanded.minWidth, equals(4));
    expect(expanded.minHeight, equals(4));
    expect(expanded.maxWidth, equals(4));
    expect(expanded.maxHeight, equals(4));

    contracted = const VisualDensity(horizontal: -4.0, vertical: -4.0).effectiveConstraints(small);
    expect(contracted.minWidth, equals(0));
    expect(contracted.minHeight, equals(0));
    expect(expanded.maxWidth, equals(4));
    expect(expanded.maxHeight, equals(4));
  });

  testWidgets('Ensure Visual Density effective constraints expand and contract', (
    WidgetTester tester,
  ) async {
    const BoxConstraints square = BoxConstraints();
    final BoxConstraints expanded = const VisualDensity(
      horizontal: 4.0,
      vertical: 4.0,
    ).effectiveConstraints(square);
    expect(expanded.minWidth, equals(16));
    expect(expanded.minHeight, equals(16));
    expect(expanded.maxWidth, equals(double.infinity));
    expect(expanded.maxHeight, equals(double.infinity));

    final BoxConstraints contracted = const VisualDensity(
      horizontal: -4.0,
      vertical: -4.0,
    ).effectiveConstraints(square);
    expect(contracted.minWidth, equals(0));
    expect(contracted.minHeight, equals(0));
    expect(expanded.maxWidth, equals(double.infinity));
    expect(expanded.maxHeight, equals(double.infinity));
  });

  group('Theme extensions', () {
    const Key containerKey = Key('container');

    testWidgets('can be obtained', (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(
            extensions: const <ThemeExtension<dynamic>>{
              MyThemeExtensionA(color1: Colors.black, color2: Colors.amber),
              MyThemeExtensionB(textStyle: TextStyle(fontSize: 50)),
            },
          ),
          home: Container(key: containerKey),
        ),
      );

      final ThemeData theme = Theme.of(tester.element(find.byKey(containerKey)));

      expect(theme.extension<MyThemeExtensionA>()!.color1, Colors.black);
      expect(theme.extension<MyThemeExtensionA>()!.color2, Colors.amber);
      expect(theme.extension<MyThemeExtensionB>()!.textStyle, const TextStyle(fontSize: 50));
    });

    testWidgets('can use copyWith', (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(
            extensions: <ThemeExtension<dynamic>>{
              const MyThemeExtensionA(
                color1: Colors.black,
                color2: Colors.amber,
              ).copyWith(color1: Colors.blue),
            },
          ),
          home: Container(key: containerKey),
        ),
      );

      final ThemeData theme = Theme.of(tester.element(find.byKey(containerKey)));

      expect(theme.extension<MyThemeExtensionA>()!.color1, Colors.blue);
      expect(theme.extension<MyThemeExtensionA>()!.color2, Colors.amber);
    });

    testWidgets('can lerp', (WidgetTester tester) async {
      const MyThemeExtensionA extensionA1 = MyThemeExtensionA(
        color1: Colors.black,
        color2: Colors.amber,
      );
      const MyThemeExtensionA extensionA2 = MyThemeExtensionA(
        color1: Colors.white,
        color2: Colors.blue,
      );
      const MyThemeExtensionB extensionB1 = MyThemeExtensionB(textStyle: TextStyle(fontSize: 50));
      const MyThemeExtensionB extensionB2 = MyThemeExtensionB(textStyle: TextStyle(fontSize: 100));

      // Both ThemeData arguments include both extensions.
      ThemeData lerped = ThemeData.lerp(
        ThemeData(extensions: const <ThemeExtension<dynamic>>[extensionA1, extensionB1]),
        ThemeData(extensions: const <ThemeExtension<dynamic>>{extensionA2, extensionB2}),
        0.5,
      );

      expect(lerped.extension<MyThemeExtensionA>()!.color1, isSameColorAs(const Color(0xff7f7f7f)));
      expect(lerped.extension<MyThemeExtensionA>()!.color2, isSameColorAs(const Color(0xff90ab7d)));
      expect(lerped.extension<MyThemeExtensionB>()!.textStyle, const TextStyle(fontSize: 75));

      // Missing from 2nd ThemeData
      lerped = ThemeData.lerp(
        ThemeData(extensions: const <ThemeExtension<dynamic>>{extensionA1, extensionB1}),
        ThemeData(extensions: const <ThemeExtension<dynamic>>{extensionB2}),
        0.5,
      );
      expect(
        lerped.extension<MyThemeExtensionA>()!.color1,
        isSameColorAs(Colors.black),
      ); // Not lerped
      expect(
        lerped.extension<MyThemeExtensionA>()!.color2,
        isSameColorAs(Colors.amber),
      ); // Not lerped
      expect(lerped.extension<MyThemeExtensionB>()!.textStyle, const TextStyle(fontSize: 75));

      // Missing from 1st ThemeData
      lerped = ThemeData.lerp(
        ThemeData(extensions: const <ThemeExtension<dynamic>>{extensionA1}),
        ThemeData(extensions: const <ThemeExtension<dynamic>>{extensionA2, extensionB2}),
        0.5,
      );
      expect(lerped.extension<MyThemeExtensionA>()!.color1, isSameColorAs(const Color(0xff7f7f7f)));
      expect(lerped.extension<MyThemeExtensionA>()!.color2, isSameColorAs(const Color(0xff90ab7d)));
      expect(
        lerped.extension<MyThemeExtensionB>()!.textStyle,
        const TextStyle(fontSize: 100),
      ); // Not lerped
    });

    testWidgets('should return null on extension not found', (WidgetTester tester) async {
      final ThemeData theme = ThemeData(extensions: const <ThemeExtension<dynamic>>{});

      expect(theme.extension<MyThemeExtensionA>(), isNull);
    });
  });

  test('copyWith, ==, hashCode basics', () {
    expect(ThemeData(), ThemeData().copyWith());
    expect(ThemeData().hashCode, ThemeData().copyWith().hashCode);
  });

  test('== and hashCode include focusColor and hoverColor', () {
    // regression test for https://github.com/flutter/flutter/issues/91587

    // Focus color and hover color are used in the default button theme, so
    // use an empty one to ensure that just focus and hover colors are tested.
    const ButtonThemeData buttonTheme = ButtonThemeData();

    final ThemeData focusColorBlack = ThemeData(focusColor: Colors.black, buttonTheme: buttonTheme);
    final ThemeData focusColorWhite = ThemeData(focusColor: Colors.white, buttonTheme: buttonTheme);
    expect(focusColorBlack != focusColorWhite, true);
    expect(focusColorBlack.hashCode != focusColorWhite.hashCode, true);

    final ThemeData hoverColorBlack = ThemeData(hoverColor: Colors.black, buttonTheme: buttonTheme);
    final ThemeData hoverColorWhite = ThemeData(hoverColor: Colors.white, buttonTheme: buttonTheme);
    expect(hoverColorBlack != hoverColorWhite, true);
    expect(hoverColorBlack.hashCode != hoverColorWhite.hashCode, true);
  });

  testWidgets('ThemeData.copyWith correctly creates new ThemeData with all copied arguments', (
    WidgetTester tester,
  ) async {
    final SliderThemeData sliderTheme = SliderThemeData.fromPrimaryColors(
      primaryColor: Colors.black,
      primaryColorDark: Colors.black,
      primaryColorLight: Colors.black,
      valueIndicatorTextStyle: const TextStyle(color: Colors.black),
    );

    final ChipThemeData chipTheme = ChipThemeData.fromDefaults(
      primaryColor: Colors.black,
      secondaryColor: Colors.white,
      labelStyle: const TextStyle(color: Colors.black),
    );

    const PageTransitionsTheme pageTransitionTheme = PageTransitionsTheme(
      builders: <TargetPlatform, PageTransitionsBuilder>{
        TargetPlatform.iOS: CupertinoPageTransitionsBuilder(),
        TargetPlatform.macOS: CupertinoPageTransitionsBuilder(),
      },
    );

    final ThemeData theme = ThemeData.raw(
      // For the sanity of the reader, make sure these properties are in the same
      // order everywhere that they are separated by section comments (e.g.
      // GENERAL CONFIGURATION). Each section except for deprecations should be
      // alphabetical by symbol name.

      // GENERAL CONFIGURATION
      adaptationMap: const <Type, Adaptation<Object>>{},
      applyElevationOverlayColor: false,
      cupertinoOverrideTheme: null,
      extensions: const <Object, ThemeExtension<dynamic>>{},
      inputDecorationTheme: ThemeData.dark().inputDecorationTheme.copyWith(
        border: const OutlineInputBorder(),
      ),
      materialTapTargetSize: MaterialTapTargetSize.shrinkWrap,
      pageTransitionsTheme: pageTransitionTheme,
      platform: TargetPlatform.iOS,
      scrollbarTheme: const ScrollbarThemeData(radius: Radius.circular(10.0)),
      splashFactory: InkRipple.splashFactory,
      useMaterial3: false,
      visualDensity: VisualDensity.standard,
      // COLOR
      canvasColor: Colors.black,
      cardColor: Colors.black,
      colorScheme: const ColorScheme.light(),
      dialogBackgroundColor: Colors.black,
      disabledColor: Colors.black,
      dividerColor: Colors.black,
      focusColor: Colors.black,
      highlightColor: Colors.black,
      hintColor: Colors.black,
      hoverColor: Colors.black,
      indicatorColor: Colors.black,
      primaryColor: Colors.black,
      primaryColorDark: Colors.black,
      primaryColorLight: Colors.black,
      scaffoldBackgroundColor: Colors.black,
      secondaryHeaderColor: Colors.black,
      shadowColor: Colors.black,
      splashColor: Colors.black,
      unselectedWidgetColor: Colors.black,
      // TYPOGRAPHY & ICONOGRAPHY
      iconTheme: ThemeData.dark().iconTheme,
      primaryIconTheme: ThemeData.dark().iconTheme,
      primaryTextTheme: ThemeData.dark().textTheme,
      textTheme: ThemeData.dark().textTheme,
      typography: Typography.material2018(),
      // COMPONENT THEMES
      actionIconTheme: const ActionIconThemeData(),
      appBarTheme: const AppBarTheme(backgroundColor: Colors.black),
      badgeTheme: const BadgeThemeData(backgroundColor: Colors.black),
      bannerTheme: const MaterialBannerThemeData(backgroundColor: Colors.black),
      bottomAppBarTheme: const BottomAppBarTheme(color: Colors.black),
      bottomNavigationBarTheme: const BottomNavigationBarThemeData(
        type: BottomNavigationBarType.fixed,
      ),
      bottomSheetTheme: const BottomSheetThemeData(backgroundColor: Colors.black),
      buttonBarTheme: const ButtonBarThemeData(alignment: MainAxisAlignment.start),
      buttonTheme: const ButtonThemeData(colorScheme: ColorScheme.dark()),
      cardTheme: const CardThemeData(color: Colors.black),
      checkboxTheme: const CheckboxThemeData(),
      chipTheme: chipTheme,
      dataTableTheme: const DataTableThemeData(),
      datePickerTheme: const DatePickerThemeData(),
      dialogTheme: const DialogThemeData(backgroundColor: Colors.black),
      dividerTheme: const DividerThemeData(color: Colors.black),
      drawerTheme: const DrawerThemeData(),
      dropdownMenuTheme: const DropdownMenuThemeData(),
      elevatedButtonTheme: ElevatedButtonThemeData(
        style: ElevatedButton.styleFrom(backgroundColor: Colors.green),
      ),
      expansionTileTheme: const ExpansionTileThemeData(backgroundColor: Colors.black),
      filledButtonTheme: FilledButtonThemeData(
        style: FilledButton.styleFrom(foregroundColor: Colors.green),
      ),
      floatingActionButtonTheme: const FloatingActionButtonThemeData(backgroundColor: Colors.black),
      iconButtonTheme: IconButtonThemeData(
        style: IconButton.styleFrom(foregroundColor: Colors.pink),
      ),
      listTileTheme: const ListTileThemeData(),
      menuBarTheme: const MenuBarThemeData(
        style: MenuStyle(backgroundColor: MaterialStatePropertyAll<Color>(Colors.black)),
      ),
      menuButtonTheme: MenuButtonThemeData(
        style: MenuItemButton.styleFrom(backgroundColor: Colors.black),
      ),
      menuTheme: const MenuThemeData(
        style: MenuStyle(backgroundColor: MaterialStatePropertyAll<Color>(Colors.black)),
      ),
      navigationBarTheme: const NavigationBarThemeData(backgroundColor: Colors.black),
      navigationDrawerTheme: const NavigationDrawerThemeData(backgroundColor: Colors.black),
      navigationRailTheme: const NavigationRailThemeData(backgroundColor: Colors.black),
      outlinedButtonTheme: OutlinedButtonThemeData(
        style: OutlinedButton.styleFrom(foregroundColor: Colors.blue),
      ),
      popupMenuTheme: const PopupMenuThemeData(color: Colors.black),
      progressIndicatorTheme: const ProgressIndicatorThemeData(),
      radioTheme: const RadioThemeData(),
      searchBarTheme: const SearchBarThemeData(),
      searchViewTheme: const SearchViewThemeData(),
      segmentedButtonTheme: const SegmentedButtonThemeData(),
      sliderTheme: sliderTheme,
      snackBarTheme: const SnackBarThemeData(backgroundColor: Colors.black),
      switchTheme: const SwitchThemeData(),
      tabBarTheme: const TabBarThemeData(labelColor: Colors.black),
      textButtonTheme: TextButtonThemeData(
        style: TextButton.styleFrom(foregroundColor: Colors.red),
      ),
      textSelectionTheme: const TextSelectionThemeData(cursorColor: Colors.black),
      timePickerTheme: const TimePickerThemeData(backgroundColor: Colors.black),
      toggleButtonsTheme: const ToggleButtonsThemeData(textStyle: TextStyle(color: Colors.black)),
      tooltipTheme: const TooltipThemeData(height: 100),
    );

    final SliderThemeData otherSliderTheme = SliderThemeData.fromPrimaryColors(
      primaryColor: Colors.white,
      primaryColorDark: Colors.white,
      primaryColorLight: Colors.white,
      valueIndicatorTextStyle: const TextStyle(color: Colors.white),
    );

    final ChipThemeData otherChipTheme = ChipThemeData.fromDefaults(
      primaryColor: Colors.white,
      secondaryColor: Colors.black,
      labelStyle: const TextStyle(color: Colors.white),
    );

    final ThemeData otherTheme = ThemeData.raw(
      // For the sanity of the reader, make sure these properties are in the same
      // order everywhere that they are separated by section comments (e.g.
      // GENERAL CONFIGURATION). Each section except for deprecations should be
      // alphabetical by symbol name.

      // GENERAL CONFIGURATION
      adaptationMap: const <Type, Adaptation<Object>>{SwitchThemeData: SwitchThemeAdaptation()},
      applyElevationOverlayColor: true,
      cupertinoOverrideTheme: ThemeData.light().cupertinoOverrideTheme,
      extensions: const <Object, ThemeExtension<dynamic>>{
        MyThemeExtensionB: MyThemeExtensionB(textStyle: TextStyle()),
      },
      inputDecorationTheme: ThemeData.light().inputDecorationTheme.copyWith(
        border: InputBorder.none,
      ),
      materialTapTargetSize: MaterialTapTargetSize.padded,
      pageTransitionsTheme: const PageTransitionsTheme(),
      platform: TargetPlatform.android,
      scrollbarTheme: const ScrollbarThemeData(radius: Radius.circular(10.0)),
      splashFactory: InkRipple.splashFactory,
      useMaterial3: true,
      visualDensity: VisualDensity.standard,

      // COLOR
      canvasColor: Colors.white,
      cardColor: Colors.white,
      colorScheme: const ColorScheme.light(),
      dialogBackgroundColor: Colors.white,
      disabledColor: Colors.white,
      dividerColor: Colors.white,
      focusColor: Colors.white,
      highlightColor: Colors.white,
      hintColor: Colors.white,
      hoverColor: Colors.white,
      indicatorColor: Colors.white,
      primaryColor: Colors.white,
      primaryColorDark: Colors.white,
      primaryColorLight: Colors.white,
      scaffoldBackgroundColor: Colors.white,
      secondaryHeaderColor: Colors.white,
      shadowColor: Colors.white,
      splashColor: Colors.white,
      unselectedWidgetColor: Colors.white,

      // TYPOGRAPHY & ICONOGRAPHY
      iconTheme: ThemeData.light().iconTheme,
      primaryIconTheme: ThemeData.light().iconTheme,
      primaryTextTheme: ThemeData.light().textTheme,
      textTheme: ThemeData.light().textTheme,
      typography: Typography.material2018(platform: TargetPlatform.iOS),

      // COMPONENT THEMES
      actionIconTheme: const ActionIconThemeData(),
      appBarTheme: const AppBarTheme(backgroundColor: Colors.white),
      badgeTheme: const BadgeThemeData(backgroundColor: Colors.black),
      bannerTheme: const MaterialBannerThemeData(backgroundColor: Colors.white),
      bottomAppBarTheme: const BottomAppBarTheme(color: Colors.white),
      bottomNavigationBarTheme: const BottomNavigationBarThemeData(
        type: BottomNavigationBarType.shifting,
      ),
      bottomSheetTheme: const BottomSheetThemeData(backgroundColor: Colors.white),
      buttonBarTheme: const ButtonBarThemeData(alignment: MainAxisAlignment.end),
      buttonTheme: const ButtonThemeData(colorScheme: ColorScheme.light()),
      cardTheme: const CardThemeData(color: Colors.white),
      checkboxTheme: const CheckboxThemeData(),
      chipTheme: otherChipTheme,
      dataTableTheme: const DataTableThemeData(),
      datePickerTheme: const DatePickerThemeData(backgroundColor: Colors.amber),
      dialogTheme: const DialogThemeData(backgroundColor: Colors.white),
      dividerTheme: const DividerThemeData(color: Colors.white),
      drawerTheme: const DrawerThemeData(),
      dropdownMenuTheme: const DropdownMenuThemeData(),
      elevatedButtonTheme: const ElevatedButtonThemeData(),
      expansionTileTheme: const ExpansionTileThemeData(backgroundColor: Colors.black),
      filledButtonTheme: const FilledButtonThemeData(),
      floatingActionButtonTheme: const FloatingActionButtonThemeData(backgroundColor: Colors.white),
      iconButtonTheme: const IconButtonThemeData(),
      listTileTheme: const ListTileThemeData(),
      menuBarTheme: const MenuBarThemeData(
        style: MenuStyle(backgroundColor: MaterialStatePropertyAll<Color>(Colors.white)),
      ),
      menuButtonTheme: MenuButtonThemeData(
        style: MenuItemButton.styleFrom(backgroundColor: Colors.black),
      ),
      menuTheme: const MenuThemeData(
        style: MenuStyle(backgroundColor: MaterialStatePropertyAll<Color>(Colors.white)),
      ),
      navigationBarTheme: const NavigationBarThemeData(backgroundColor: Colors.white),
      navigationDrawerTheme: const NavigationDrawerThemeData(backgroundColor: Colors.white),
      navigationRailTheme: const NavigationRailThemeData(backgroundColor: Colors.white),
      outlinedButtonTheme: const OutlinedButtonThemeData(),
      popupMenuTheme: const PopupMenuThemeData(color: Colors.white),
      progressIndicatorTheme: const ProgressIndicatorThemeData(),
      radioTheme: const RadioThemeData(),
      searchBarTheme: const SearchBarThemeData(),
      searchViewTheme: const SearchViewThemeData(),
      segmentedButtonTheme: const SegmentedButtonThemeData(),
      sliderTheme: otherSliderTheme,
      snackBarTheme: const SnackBarThemeData(backgroundColor: Colors.white),
      switchTheme: const SwitchThemeData(),
      tabBarTheme: const TabBarThemeData(labelColor: Colors.white),
      textButtonTheme: const TextButtonThemeData(),
      textSelectionTheme: const TextSelectionThemeData(cursorColor: Colors.white),
      timePickerTheme: const TimePickerThemeData(backgroundColor: Colors.white),
      toggleButtonsTheme: const ToggleButtonsThemeData(textStyle: TextStyle(color: Colors.white)),
      tooltipTheme: const TooltipThemeData(height: 100),
    );

    final ThemeData themeDataCopy = theme.copyWith(
      // For the sanity of the reader, make sure these properties are in the same
      // order everywhere that they are separated by section comments (e.g.
      // GENERAL CONFIGURATION). Each section except for deprecations should be
      // alphabetical by symbol name.

      // GENERAL CONFIGURATION
      adaptations: otherTheme.adaptationMap.values,
      applyElevationOverlayColor: otherTheme.applyElevationOverlayColor,
      cupertinoOverrideTheme: otherTheme.cupertinoOverrideTheme,
      extensions: otherTheme.extensions.values,
      inputDecorationTheme: otherTheme.inputDecorationTheme,
      materialTapTargetSize: otherTheme.materialTapTargetSize,
      pageTransitionsTheme: otherTheme.pageTransitionsTheme,
      platform: otherTheme.platform,
      scrollbarTheme: otherTheme.scrollbarTheme,
      splashFactory: otherTheme.splashFactory,
      useMaterial3: otherTheme.useMaterial3,
      visualDensity: otherTheme.visualDensity,

      // COLOR
      canvasColor: otherTheme.canvasColor,
      cardColor: otherTheme.cardColor,
      colorScheme: otherTheme.colorScheme,
      dialogBackgroundColor: otherTheme.dialogBackgroundColor,
      disabledColor: otherTheme.disabledColor,
      dividerColor: otherTheme.dividerColor,
      focusColor: otherTheme.focusColor,
      highlightColor: otherTheme.highlightColor,
      hintColor: otherTheme.hintColor,
      hoverColor: otherTheme.hoverColor,
      indicatorColor: otherTheme.indicatorColor,
      primaryColor: otherTheme.primaryColor,
      primaryColorDark: otherTheme.primaryColorDark,
      primaryColorLight: otherTheme.primaryColorLight,
      scaffoldBackgroundColor: otherTheme.scaffoldBackgroundColor,
      secondaryHeaderColor: otherTheme.secondaryHeaderColor,
      shadowColor: otherTheme.shadowColor,
      splashColor: otherTheme.splashColor,
      unselectedWidgetColor: otherTheme.unselectedWidgetColor,

      // TYPOGRAPHY & ICONOGRAPHY
      iconTheme: otherTheme.iconTheme,
      primaryIconTheme: otherTheme.primaryIconTheme,
      primaryTextTheme: otherTheme.primaryTextTheme,
      textTheme: otherTheme.textTheme,
      typography: otherTheme.typography,

      // COMPONENT THEMES
      actionIconTheme: otherTheme.actionIconTheme,
      appBarTheme: otherTheme.appBarTheme,
      badgeTheme: otherTheme.badgeTheme,
      bannerTheme: otherTheme.bannerTheme,
      bottomAppBarTheme: otherTheme.bottomAppBarTheme,
      bottomNavigationBarTheme: otherTheme.bottomNavigationBarTheme,
      bottomSheetTheme: otherTheme.bottomSheetTheme,
      buttonBarTheme: otherTheme.buttonBarTheme,
      buttonTheme: otherTheme.buttonTheme,
      cardTheme: otherTheme.cardTheme,
      checkboxTheme: otherTheme.checkboxTheme,
      chipTheme: otherTheme.chipTheme,
      dataTableTheme: otherTheme.dataTableTheme,
      dialogTheme: otherTheme.dialogTheme,
      datePickerTheme: otherTheme.datePickerTheme,
      dividerTheme: otherTheme.dividerTheme,
      drawerTheme: otherTheme.drawerTheme,
      elevatedButtonTheme: otherTheme.elevatedButtonTheme,
      expansionTileTheme: otherTheme.expansionTileTheme,
      filledButtonTheme: otherTheme.filledButtonTheme,
      floatingActionButtonTheme: otherTheme.floatingActionButtonTheme,
      iconButtonTheme: otherTheme.iconButtonTheme,
      listTileTheme: otherTheme.listTileTheme,
      menuBarTheme: otherTheme.menuBarTheme,
      menuButtonTheme: otherTheme.menuButtonTheme,
      menuTheme: otherTheme.menuTheme,
      navigationBarTheme: otherTheme.navigationBarTheme,
      navigationDrawerTheme: otherTheme.navigationDrawerTheme,
      navigationRailTheme: otherTheme.navigationRailTheme,
      outlinedButtonTheme: otherTheme.outlinedButtonTheme,
      popupMenuTheme: otherTheme.popupMenuTheme,
      progressIndicatorTheme: otherTheme.progressIndicatorTheme,
      radioTheme: otherTheme.radioTheme,
      searchBarTheme: otherTheme.searchBarTheme,
      searchViewTheme: otherTheme.searchViewTheme,
      sliderTheme: otherTheme.sliderTheme,
      snackBarTheme: otherTheme.snackBarTheme,
      switchTheme: otherTheme.switchTheme,
      tabBarTheme: otherTheme.tabBarTheme,
      textButtonTheme: otherTheme.textButtonTheme,
      textSelectionTheme: otherTheme.textSelectionTheme,
      timePickerTheme: otherTheme.timePickerTheme,
      toggleButtonsTheme: otherTheme.toggleButtonsTheme,
      tooltipTheme: otherTheme.tooltipTheme,
    );

    // For the sanity of the reader, make sure these properties are in the same
    // order everywhere that they are separated by section comments (e.g.
    // GENERAL CONFIGURATION). Each section except for deprecations should be
    // alphabetical by symbol name.

    // GENERAL CONFIGURATION
    expect(themeDataCopy.adaptationMap, equals(otherTheme.adaptationMap));
    expect(themeDataCopy.applyElevationOverlayColor, equals(otherTheme.applyElevationOverlayColor));
    expect(themeDataCopy.cupertinoOverrideTheme, equals(otherTheme.cupertinoOverrideTheme));
    expect(themeDataCopy.extensions, equals(otherTheme.extensions));
    expect(themeDataCopy.inputDecorationTheme, equals(otherTheme.inputDecorationTheme));
    expect(themeDataCopy.materialTapTargetSize, equals(otherTheme.materialTapTargetSize));
    expect(themeDataCopy.pageTransitionsTheme, equals(otherTheme.pageTransitionsTheme));
    expect(themeDataCopy.platform, equals(otherTheme.platform));
    expect(themeDataCopy.scrollbarTheme, equals(otherTheme.scrollbarTheme));
    expect(themeDataCopy.splashFactory, equals(otherTheme.splashFactory));
    expect(themeDataCopy.useMaterial3, equals(otherTheme.useMaterial3));
    expect(themeDataCopy.visualDensity, equals(otherTheme.visualDensity));

    // COLOR
    expect(themeDataCopy.canvasColor, equals(otherTheme.canvasColor));
    expect(themeDataCopy.cardColor, equals(otherTheme.cardColor));
    expect(themeDataCopy.colorScheme, equals(otherTheme.colorScheme));
    expect(themeDataCopy.dialogBackgroundColor, equals(otherTheme.dialogBackgroundColor));
    expect(themeDataCopy.disabledColor, equals(otherTheme.disabledColor));
    expect(themeDataCopy.dividerColor, equals(otherTheme.dividerColor));
    expect(themeDataCopy.focusColor, equals(otherTheme.focusColor));
    expect(themeDataCopy.highlightColor, equals(otherTheme.highlightColor));
    expect(themeDataCopy.hintColor, equals(otherTheme.hintColor));
    expect(themeDataCopy.hoverColor, equals(otherTheme.hoverColor));
    expect(themeDataCopy.indicatorColor, equals(otherTheme.indicatorColor));
    expect(themeDataCopy.primaryColor, equals(otherTheme.primaryColor));
    expect(themeDataCopy.primaryColorDark, equals(otherTheme.primaryColorDark));
    expect(themeDataCopy.primaryColorLight, equals(otherTheme.primaryColorLight));
    expect(themeDataCopy.scaffoldBackgroundColor, equals(otherTheme.scaffoldBackgroundColor));
    expect(themeDataCopy.secondaryHeaderColor, equals(otherTheme.secondaryHeaderColor));
    expect(themeDataCopy.shadowColor, equals(otherTheme.shadowColor));
    expect(themeDataCopy.splashColor, equals(otherTheme.splashColor));
    expect(themeDataCopy.unselectedWidgetColor, equals(otherTheme.unselectedWidgetColor));

    // TYPOGRAPHY & ICONOGRAPHY
    expect(themeDataCopy.iconTheme, equals(otherTheme.iconTheme));
    expect(themeDataCopy.primaryIconTheme, equals(otherTheme.primaryIconTheme));
    expect(themeDataCopy.primaryTextTheme, equals(otherTheme.primaryTextTheme));
    expect(themeDataCopy.textTheme, equals(otherTheme.textTheme));
    expect(themeDataCopy.typography, equals(otherTheme.typography));

    // COMPONENT THEMES
    expect(themeDataCopy.actionIconTheme, equals(otherTheme.actionIconTheme));
    expect(themeDataCopy.appBarTheme, equals(otherTheme.appBarTheme));
    expect(themeDataCopy.badgeTheme, equals(otherTheme.badgeTheme));
    expect(themeDataCopy.bannerTheme, equals(otherTheme.bannerTheme));
    expect(themeDataCopy.bottomAppBarTheme, equals(otherTheme.bottomAppBarTheme));
    expect(themeDataCopy.bottomNavigationBarTheme, equals(otherTheme.bottomNavigationBarTheme));
    expect(themeDataCopy.bottomSheetTheme, equals(otherTheme.bottomSheetTheme));
    expect(themeDataCopy.buttonBarTheme, equals(otherTheme.buttonBarTheme));
    expect(themeDataCopy.buttonTheme, equals(otherTheme.buttonTheme));
    expect(themeDataCopy.cardTheme, equals(otherTheme.cardTheme));
    expect(themeDataCopy.checkboxTheme, equals(otherTheme.checkboxTheme));
    expect(themeDataCopy.chipTheme, equals(otherTheme.chipTheme));
    expect(themeDataCopy.dataTableTheme, equals(otherTheme.dataTableTheme));
    expect(themeDataCopy.datePickerTheme, equals(otherTheme.datePickerTheme));
    expect(themeDataCopy.dialogTheme, equals(otherTheme.dialogTheme));
    expect(themeDataCopy.dividerTheme, equals(otherTheme.dividerTheme));
    expect(themeDataCopy.drawerTheme, equals(otherTheme.drawerTheme));
    expect(themeDataCopy.elevatedButtonTheme, equals(otherTheme.elevatedButtonTheme));
    expect(themeDataCopy.expansionTileTheme, equals(otherTheme.expansionTileTheme));
    expect(themeDataCopy.filledButtonTheme, equals(otherTheme.filledButtonTheme));
    expect(themeDataCopy.floatingActionButtonTheme, equals(otherTheme.floatingActionButtonTheme));
    expect(themeDataCopy.iconButtonTheme, equals(otherTheme.iconButtonTheme));
    expect(themeDataCopy.listTileTheme, equals(otherTheme.listTileTheme));
    expect(themeDataCopy.menuBarTheme, equals(otherTheme.menuBarTheme));
    expect(themeDataCopy.menuButtonTheme, equals(otherTheme.menuButtonTheme));
    expect(themeDataCopy.menuTheme, equals(otherTheme.menuTheme));
    expect(themeDataCopy.navigationBarTheme, equals(otherTheme.navigationBarTheme));
    expect(themeDataCopy.navigationRailTheme, equals(otherTheme.navigationRailTheme));
    expect(themeDataCopy.outlinedButtonTheme, equals(otherTheme.outlinedButtonTheme));
    expect(themeDataCopy.popupMenuTheme, equals(otherTheme.popupMenuTheme));
    expect(themeDataCopy.progressIndicatorTheme, equals(otherTheme.progressIndicatorTheme));
    expect(themeDataCopy.radioTheme, equals(otherTheme.radioTheme));
    expect(themeDataCopy.searchBarTheme, equals(otherTheme.searchBarTheme));
    expect(themeDataCopy.searchViewTheme, equals(otherTheme.searchViewTheme));
    expect(themeDataCopy.sliderTheme, equals(otherTheme.sliderTheme));
    expect(themeDataCopy.snackBarTheme, equals(otherTheme.snackBarTheme));
    expect(themeDataCopy.switchTheme, equals(otherTheme.switchTheme));
    expect(themeDataCopy.tabBarTheme, equals(otherTheme.tabBarTheme));
    expect(themeDataCopy.textButtonTheme, equals(otherTheme.textButtonTheme));
    expect(themeDataCopy.textSelectionTheme, equals(otherTheme.textSelectionTheme));
    expect(
      themeDataCopy.textSelectionTheme.selectionColor,
      equals(otherTheme.textSelectionTheme.selectionColor),
    );
    expect(
      themeDataCopy.textSelectionTheme.cursorColor,
      equals(otherTheme.textSelectionTheme.cursorColor),
    );
    expect(
      themeDataCopy.textSelectionTheme.selectionHandleColor,
      equals(otherTheme.textSelectionTheme.selectionHandleColor),
    );
    expect(themeDataCopy.timePickerTheme, equals(otherTheme.timePickerTheme));
    expect(themeDataCopy.toggleButtonsTheme, equals(otherTheme.toggleButtonsTheme));
    expect(themeDataCopy.tooltipTheme, equals(otherTheme.tooltipTheme));
  });

  testWidgets('ThemeData.toString has less than 200 characters output', (
    WidgetTester tester,
  ) async {
    // This test makes sure that the ThemeData debug output doesn't get too
    // verbose, which has been a problem in the past.

    const ColorScheme darkColors = ColorScheme.dark();
    final ThemeData darkTheme = ThemeData.from(colorScheme: darkColors);

    expect(darkTheme.toString().length, lessThan(200));

    const ColorScheme lightColors = ColorScheme.light();
    final ThemeData lightTheme = ThemeData.from(colorScheme: lightColors);

    expect(lightTheme.toString().length, lessThan(200));
  });

  testWidgets('ThemeData brightness parameter overrides ColorScheme brightness', (
    WidgetTester tester,
  ) async {
    const ColorScheme lightColors = ColorScheme.light();
    expect(
      () => ThemeData(colorScheme: lightColors, brightness: Brightness.dark),
      throwsAssertionError,
    );
  });

  testWidgets('ThemeData.copyWith brightness parameter overrides ColorScheme brightness', (
    WidgetTester tester,
  ) async {
    const ColorScheme lightColors = ColorScheme.light();
    final ThemeData theme = ThemeData.from(
      colorScheme: lightColors,
    ).copyWith(brightness: Brightness.dark);

    // The brightness parameter only overrides ColorScheme.brightness.
    expect(theme.brightness, equals(Brightness.dark));
    expect(theme.colorScheme.brightness, equals(Brightness.dark));
    expect(theme.primaryColor, equals(lightColors.primary));
    expect(theme.cardColor, equals(lightColors.surface));
    expect(theme.canvasColor, equals(lightColors.surface));
    expect(theme.scaffoldBackgroundColor, equals(lightColors.surface));
    expect(theme.dialogBackgroundColor, equals(lightColors.surface));
    expect(theme.applyElevationOverlayColor, isFalse);
  });

  test('ThemeData diagnostics include all properties', () {
    // List of properties must match the properties in ThemeData.hashCode()
    final Set<String> expectedPropertyNames = <String>{
      // GENERAL CONFIGURATION
      'adaptations',
      'applyElevationOverlayColor',
      'cupertinoOverrideTheme',
      'extensions',
      'inputDecorationTheme',
      'materialTapTargetSize',
      'pageTransitionsTheme',
      'platform',
      'scrollbarTheme',
      'splashFactory',
      'visualDensity',
      'useMaterial3',
      // COLOR
      'colorScheme',
      'primaryColor',
      'primaryColorLight',
      'primaryColorDark',
      'focusColor',
      'hoverColor',
      'shadowColor',
      'canvasColor',
      'scaffoldBackgroundColor',
      'cardColor',
      'dividerColor',
      'highlightColor',
      'splashColor',
      'unselectedWidgetColor',
      'disabledColor',
      'secondaryHeaderColor',
      'dialogBackgroundColor',
      'indicatorColor',
      'hintColor',
      // TYPOGRAPHY & ICONOGRAPHY
      'typography',
      'textTheme',
      'primaryTextTheme',
      'iconTheme',
      'primaryIconTheme',
      // COMPONENT THEMES
      'actionIconTheme',
      'appBarTheme',
      'badgeTheme',
      'bannerTheme',
      'bottomAppBarTheme',
      'bottomNavigationBarTheme',
      'bottomSheetTheme',
      'buttonBarTheme',
      'buttonTheme',
      'cardTheme',
      'checkboxTheme',
      'chipTheme',
      'dataTableTheme',
      'datePickerTheme',
      'dialogTheme',
      'dividerTheme',
      'drawerTheme',
      'dropdownMenuTheme',
      'elevatedButtonTheme',
      'expansionTileTheme',
      'filledButtonTheme',
      'floatingActionButtonTheme',
      'iconButtonTheme',
      'listTileTheme',
      'menuBarTheme',
      'menuButtonTheme',
      'menuTheme',
      'navigationBarTheme',
      'navigationDrawerTheme',
      'navigationRailTheme',
      'outlinedButtonTheme',
      'popupMenuTheme',
      'progressIndicatorTheme',
      'radioTheme',
      'searchBarTheme',
      'searchViewTheme',
      'segmentedButtonTheme',
      'sliderTheme',
      'snackBarTheme',
      'switchTheme',
      'tabBarTheme',
      'textButtonTheme',
      'textSelectionTheme',
      'timePickerTheme',
      'toggleButtonsTheme',
      'tooltipTheme',
    };

    final DiagnosticPropertiesBuilder properties = DiagnosticPropertiesBuilder();
    ThemeData.light().debugFillProperties(properties);
    final List<String> propertyNameList =
        properties.properties
            .map((final DiagnosticsNode node) => node.name)
            .whereType<String>()
            .toList();
    final Set<String> propertyNames = propertyNameList.toSet();

    // Ensure there are no duplicates.
    expect(propertyNameList.length, propertyNames.length);

    // Ensure they are all there.
    expect(propertyNames, expectedPropertyNames);
  });

  group('Theme adaptationMap', () {
    const Key containerKey = Key('container');

    testWidgets('can be obtained', (WidgetTester tester) async {
      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData(
            adaptations: const <Adaptation<Object>>[StringAdaptation(), SwitchThemeAdaptation()],
          ),
          home: Container(key: containerKey),
        ),
      );

      final ThemeData theme = Theme.of(tester.element(find.byKey(containerKey)));
      final String adaptiveString = theme.getAdaptation<String>()!.adapt(theme, 'Default theme');
      final SwitchThemeData adaptiveSwitchTheme = theme.getAdaptation<SwitchThemeData>()!.adapt(
        theme,
        theme.switchTheme,
      );

      expect(adaptiveString, 'Adaptive theme.');
      expect(
        adaptiveSwitchTheme.thumbColor?.resolve(<MaterialState>{}),
        isSameColorAs(Colors.brown),
      );
    });

    testWidgets('should return null on extension not found', (WidgetTester tester) async {
      final ThemeData theme = ThemeData(
        adaptations: const <Adaptation<Object>>[StringAdaptation()],
      );

      expect(theme.extension<SwitchThemeAdaptation>(), isNull);
    });
  });

  testWidgets(
    'ThemeData.brightness not matching ColorScheme.brightness throws a helpful error message',
    (WidgetTester tester) async {
      AssertionError? error;

      // Test `ColorScheme.light()` and `ThemeData.brightness == Brightness.dark`.
      try {
        await tester.pumpWidget(
          MaterialApp(
            theme: ThemeData(colorScheme: const ColorScheme.light(), brightness: Brightness.dark),
            home: const Placeholder(),
          ),
        );
      } on AssertionError catch (e) {
        error = e;
      } finally {
        expect(error, isNotNull);
        expect(
          error?.message,
          contains(
            'ThemeData.brightness does not match ColorScheme.brightness. '
            'Either override ColorScheme.brightness or ThemeData.brightness to '
            'match the other.',
          ),
        );
      }

      // Test `ColorScheme.dark()` and `ThemeData.brightness == Brightness.light`.
      try {
        await tester.pumpWidget(
          MaterialApp(
            theme: ThemeData(colorScheme: const ColorScheme.dark(), brightness: Brightness.light),
            home: const Placeholder(),
          ),
        );
      } on AssertionError catch (e) {
        error = e;
      } finally {
        expect(error, isNotNull);
        expect(
          error?.message,
          contains(
            'ThemeData.brightness does not match ColorScheme.brightness. '
            'Either override ColorScheme.brightness or ThemeData.brightness to '
            'match the other.',
          ),
        );
      }

      // Test `ColorScheme.fromSeed()` and `ThemeData.brightness == Brightness.dark`.
      try {
        await tester.pumpWidget(
          MaterialApp(
            theme: ThemeData(
              colorScheme: ColorScheme.fromSeed(seedColor: const Color(0xffff0000)),
              brightness: Brightness.dark,
            ),
            home: const Placeholder(),
          ),
        );
      } on AssertionError catch (e) {
        error = e;
      } finally {
        expect(error, isNotNull);
        expect(
          error?.message,
          contains(
            'ThemeData.brightness does not match ColorScheme.brightness. '
            'Either override ColorScheme.brightness or ThemeData.brightness to '
            'match the other.',
          ),
        );
      }

      // Test `ColorScheme.fromSeed()` using `Brightness.dark` and `ThemeData.brightness == Brightness.light`.
      try {
        await tester.pumpWidget(
          MaterialApp(
            theme: ThemeData(
              colorScheme: ColorScheme.fromSeed(
                seedColor: const Color(0xffff0000),
                brightness: Brightness.dark,
              ),
              brightness: Brightness.light,
            ),
            home: const Placeholder(),
          ),
        );
      } on AssertionError catch (e) {
        error = e;
      } finally {
        expect(error, isNotNull);
        expect(
          error?.message,
          contains(
            'ThemeData.brightness does not match ColorScheme.brightness. '
            'Either override ColorScheme.brightness or ThemeData.brightness to '
            'match the other.',
          ),
        );
      }
    },
  );
}

@immutable
class MyThemeExtensionA extends ThemeExtension<MyThemeExtensionA> {
  const MyThemeExtensionA({required this.color1, required this.color2});

  final Color? color1;
  final Color? color2;

  @override
  MyThemeExtensionA copyWith({Color? color1, Color? color2}) {
    return MyThemeExtensionA(color1: color1 ?? this.color1, color2: color2 ?? this.color2);
  }

  @override
  MyThemeExtensionA lerp(MyThemeExtensionA? other, double t) {
    if (other is! MyThemeExtensionA) {
      return this;
    }
    return MyThemeExtensionA(
      color1: Color.lerp(color1, other.color1, t),
      color2: Color.lerp(color2, other.color2, t),
    );
  }
}

@immutable
class MyThemeExtensionB extends ThemeExtension<MyThemeExtensionB> {
  const MyThemeExtensionB({required this.textStyle});

  final TextStyle? textStyle;

  @override
  MyThemeExtensionB copyWith({Color? color, TextStyle? textStyle}) {
    return MyThemeExtensionB(textStyle: textStyle ?? this.textStyle);
  }

  @override
  MyThemeExtensionB lerp(MyThemeExtensionB? other, double t) {
    if (other is! MyThemeExtensionB) {
      return this;
    }
    return MyThemeExtensionB(textStyle: TextStyle.lerp(textStyle, other.textStyle, t));
  }
}

class SwitchThemeAdaptation extends Adaptation<SwitchThemeData> {
  const SwitchThemeAdaptation();

  @override
  SwitchThemeData adapt(ThemeData theme, SwitchThemeData defaultValue) =>
      const SwitchThemeData(thumbColor: MaterialStatePropertyAll<Color>(Colors.brown));
}

class StringAdaptation extends Adaptation<String> {
  const StringAdaptation();

  @override
  String adapt(ThemeData theme, String defaultValue) => 'Adaptive theme.';
}
