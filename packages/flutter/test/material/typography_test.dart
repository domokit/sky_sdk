// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  test('Typography is defined for all target platforms', () {
    for (final TargetPlatform platform in TargetPlatform.values) {
      final Typography typography = Typography.material2018(platform: platform);
      expect(typography, isNotNull, reason: 'null typography for $platform');
      expect(typography.black, isNotNull, reason: 'null black typography for $platform');
      expect(typography.white, isNotNull, reason: 'null white typography for $platform');
    }
  });

  test('Typography lerp special cases', () {
    final Typography typography = Typography();
    expect(identical(Typography.lerp(typography, typography, 0.5), typography), true);
  });

  test('Typography on non-Apple platforms defaults to the correct font', () {
    expect(Typography.material2018().black.titleLarge!.fontFamily, 'Roboto');
    expect(
      Typography.material2018(platform: TargetPlatform.fuchsia).black.titleLarge!.fontFamily,
      'Roboto',
    );
    expect(
      Typography.material2018(platform: TargetPlatform.linux).black.titleLarge!.fontFamily,
      'Roboto',
    );
    expect(
      Typography.material2018(platform: TargetPlatform.linux).black.titleLarge!.fontFamilyFallback,
      <String>['Ubuntu', 'Cantarell', 'DejaVu Sans', 'Liberation Sans', 'Arial'],
    );
    expect(
      Typography.material2018(platform: TargetPlatform.windows).black.titleLarge!.fontFamily,
      'Segoe UI',
    );
    expect(Typography.material2018().white.titleLarge!.fontFamily, 'Roboto');
    expect(
      Typography.material2018(platform: TargetPlatform.fuchsia).white.titleLarge!.fontFamily,
      'Roboto',
    );
    expect(
      Typography.material2018(platform: TargetPlatform.linux).white.titleLarge!.fontFamily,
      'Roboto',
    );
    expect(
      Typography.material2018(platform: TargetPlatform.linux).white.titleLarge!.fontFamilyFallback,
      <String>['Ubuntu', 'Cantarell', 'DejaVu Sans', 'Liberation Sans', 'Arial'],
    );
    expect(
      Typography.material2018(platform: TargetPlatform.windows).white.titleLarge!.fontFamily,
      'Segoe UI',
    );
  });

  // Ref: https://developer.apple.com/design/human-interface-guidelines/typography/
  final Matcher isSanFranciscoDisplayFont = predicate((TextStyle s) {
    return s.fontFamily == 'CupertinoSystemDisplay';
  }, 'Uses SF Display font');

  final Matcher isSanFranciscoTextFont = predicate((TextStyle s) {
    return s.fontFamily == 'CupertinoSystemText';
  }, 'Uses SF Text font');

  final Matcher isMacOSSanFranciscoMetaFont = predicate((TextStyle s) {
    return s.fontFamily == '.AppleSystemUIFont';
  }, 'Uses macOS system meta-font');

  test('Typography on iOS defaults to the correct SF font family based on size', () {
    final Typography typography = Typography.material2018(platform: TargetPlatform.iOS);
    for (final TextTheme textTheme in <TextTheme>[typography.black, typography.white]) {
      expect(textTheme.displayLarge, isSanFranciscoDisplayFont);
      expect(textTheme.displayMedium, isSanFranciscoDisplayFont);
      expect(textTheme.displaySmall, isSanFranciscoDisplayFont);
      expect(textTheme.headlineLarge, isSanFranciscoDisplayFont);
      expect(textTheme.headlineMedium, isSanFranciscoDisplayFont);
      expect(textTheme.headlineSmall, isSanFranciscoDisplayFont);
      expect(textTheme.titleLarge, isSanFranciscoDisplayFont);
      expect(textTheme.titleMedium, isSanFranciscoTextFont);
      expect(textTheme.titleSmall, isSanFranciscoTextFont);
      expect(textTheme.bodyLarge, isSanFranciscoTextFont);
      expect(textTheme.bodyMedium, isSanFranciscoTextFont);
      expect(textTheme.bodySmall, isSanFranciscoTextFont);
      expect(textTheme.labelLarge, isSanFranciscoTextFont);
      expect(textTheme.labelMedium, isSanFranciscoTextFont);
      expect(textTheme.labelSmall, isSanFranciscoTextFont);
    }
  });

  test('Typography on macOS defaults to the system UI meta-font', () {
    final Typography typography = Typography.material2018(platform: TargetPlatform.macOS);
    for (final TextTheme textTheme in <TextTheme>[typography.black, typography.white]) {
      expect(textTheme.displayLarge, isMacOSSanFranciscoMetaFont);
      expect(textTheme.displayMedium, isMacOSSanFranciscoMetaFont);
      expect(textTheme.displaySmall, isMacOSSanFranciscoMetaFont);
      expect(textTheme.headlineLarge, isMacOSSanFranciscoMetaFont);
      expect(textTheme.headlineMedium, isMacOSSanFranciscoMetaFont);
      expect(textTheme.headlineSmall, isMacOSSanFranciscoMetaFont);
      expect(textTheme.titleLarge, isMacOSSanFranciscoMetaFont);
      expect(textTheme.titleMedium, isMacOSSanFranciscoMetaFont);
      expect(textTheme.titleSmall, isMacOSSanFranciscoMetaFont);
      expect(textTheme.bodyLarge, isMacOSSanFranciscoMetaFont);
      expect(textTheme.bodyMedium, isMacOSSanFranciscoMetaFont);
      expect(textTheme.bodySmall, isMacOSSanFranciscoMetaFont);
      expect(textTheme.labelLarge, isMacOSSanFranciscoMetaFont);
      expect(textTheme.labelMedium, isMacOSSanFranciscoMetaFont);
      expect(textTheme.labelSmall, isMacOSSanFranciscoMetaFont);
    }
  });

  testWidgets('Typography implements debugFillProperties', (WidgetTester tester) async {
    final DiagnosticPropertiesBuilder builder = DiagnosticPropertiesBuilder();
    Typography.material2014(
      black: Typography.blackCupertino,
      white: Typography.whiteCupertino,
      englishLike: Typography.englishLike2018,
      dense: Typography.dense2018,
      tall: Typography.tall2018,
    ).debugFillProperties(builder);

    final List<String> nonDefaultPropertyNames =
        builder.properties
            .where((DiagnosticsNode node) => !node.isFiltered(DiagnosticLevel.info))
            .map((DiagnosticsNode node) => node.name!)
            .toList();

    expect(nonDefaultPropertyNames, <String>['black', 'white', 'englishLike', 'dense', 'tall']);
  });

  test('Can lerp between different typographies', () {
    final List<Typography> all = <Typography>[
      for (final TargetPlatform platform in TargetPlatform.values)
        Typography.material2014(platform: platform),
      for (final TargetPlatform platform in TargetPlatform.values)
        Typography.material2018(platform: platform),
      for (final TargetPlatform platform in TargetPlatform.values)
        Typography.material2021(platform: platform),
    ];

    for (final Typography fromTypography in all) {
      for (final Typography toTypography in all) {
        Object? error;
        try {
          Typography.lerp(fromTypography, toTypography, 0.5);
        } catch (e) {
          error = e;
        }
        expect(error, isNull);
      }
    }
  });

  test('englishLike2018 TextTheme matches Material Design spec', () {
    // Check the default material text theme against the style values
    // shown https://material.io/design/typography/#type-scale.

    final TextTheme theme = Typography.englishLike2018.merge(Typography.blackMountainView);
    const FontWeight light = FontWeight.w300;
    const FontWeight regular = FontWeight.w400;
    const FontWeight medium = FontWeight.w500;

    // Display Large Roboto light 96 -1.5
    expect(theme.displayLarge!.fontFamily, 'Roboto');
    expect(theme.displayLarge!.fontWeight, light);
    expect(theme.displayLarge!.fontSize, 96);
    expect(theme.displayLarge!.letterSpacing, -1.5);

    // Display Medium Roboto light 60 -0.5
    expect(theme.displayMedium!.fontFamily, 'Roboto');
    expect(theme.displayMedium!.fontWeight, light);
    expect(theme.displayMedium!.fontSize, 60);
    expect(theme.displayMedium!.letterSpacing, -0.5);

    // Display Small Roboto regular 48 0
    expect(theme.displaySmall!.fontFamily, 'Roboto');
    expect(theme.displaySmall!.fontWeight, regular);
    expect(theme.displaySmall!.fontSize, 48);
    expect(theme.displaySmall!.letterSpacing, 0);

    // Headline Large (from Material 3 for backwards compatibility) Roboto regular 40 0.25
    expect(theme.headlineLarge!.fontFamily, 'Roboto');
    expect(theme.headlineLarge!.fontWeight, regular);
    expect(theme.headlineLarge!.fontSize, 40);
    expect(theme.headlineLarge!.letterSpacing, 0.25);

    // Headline Medium Roboto regular 34 0.25
    expect(theme.headlineMedium!.fontFamily, 'Roboto');
    expect(theme.headlineMedium!.fontWeight, regular);
    expect(theme.headlineMedium!.fontSize, 34);
    expect(theme.headlineMedium!.letterSpacing, 0.25);

    // Headline Small Roboto regular 24 0
    expect(theme.headlineSmall!.fontFamily, 'Roboto');
    expect(theme.headlineSmall!.fontWeight, regular);
    expect(theme.headlineSmall!.fontSize, 24);
    expect(theme.headlineSmall!.letterSpacing, 0);

    // Title Large Roboto medium 20 0.15
    expect(theme.titleLarge!.fontFamily, 'Roboto');
    expect(theme.titleLarge!.fontWeight, medium);
    expect(theme.titleLarge!.fontSize, 20);
    expect(theme.titleLarge!.letterSpacing, 0.15);

    // Title Medium Roboto regular 16 0.15
    expect(theme.titleMedium!.fontFamily, 'Roboto');
    expect(theme.titleMedium!.fontWeight, regular);
    expect(theme.titleMedium!.fontSize, 16);
    expect(theme.titleMedium!.letterSpacing, 0.15);

    // Title Small Roboto medium 14 0.1
    expect(theme.titleSmall!.fontFamily, 'Roboto');
    expect(theme.titleSmall!.fontWeight, medium);
    expect(theme.titleSmall!.fontSize, 14);
    expect(theme.titleSmall!.letterSpacing, 0.1);

    // Body Large Roboto regular 16 0.5
    expect(theme.bodyLarge!.fontFamily, 'Roboto');
    expect(theme.bodyLarge!.fontWeight, regular);
    expect(theme.bodyLarge!.fontSize, 16);
    expect(theme.bodyLarge!.letterSpacing, 0.5);

    // Body Medium Roboto regular 14 0.25
    expect(theme.bodyMedium!.fontFamily, 'Roboto');
    expect(theme.bodyMedium!.fontWeight, regular);
    expect(theme.bodyMedium!.fontSize, 14);
    expect(theme.bodyMedium!.letterSpacing, 0.25);

    // Body Small Roboto regular 12 0.4
    expect(theme.bodySmall!.fontFamily, 'Roboto');
    expect(theme.bodySmall!.fontWeight, regular);
    expect(theme.bodySmall!.fontSize, 12);
    expect(theme.bodySmall!.letterSpacing, 0.4);

    // Label Large Roboto medium 14 1.25
    expect(theme.labelLarge!.fontFamily, 'Roboto');
    expect(theme.labelLarge!.fontWeight, medium);
    expect(theme.labelLarge!.fontSize, 14);
    expect(theme.labelLarge!.letterSpacing, 1.25);

    // Label Medium (from Material 3 for backwards compatibility) Roboto regular 11 1.5
    expect(theme.labelMedium!.fontFamily, 'Roboto');
    expect(theme.labelMedium!.fontWeight, regular);
    expect(theme.labelMedium!.fontSize, 11);
    expect(theme.labelMedium!.letterSpacing, 1.5);

    // Label Small Roboto regular 10 1.5
    expect(theme.labelSmall!.fontFamily, 'Roboto');
    expect(theme.labelSmall!.fontWeight, regular);
    expect(theme.labelSmall!.fontSize, 10);
    expect(theme.labelSmall!.letterSpacing, 1.5);
  });

  test('englishLike2021 TextTheme matches Material Design 3 spec', () {
    // Check the default material text theme against the style values
    // shown https://m3.material.io/styles/typography/tokens.
    //
    // This may need to be updated if the token values change.
    final TextTheme theme = Typography.englishLike2021.merge(Typography.blackMountainView);

    // Display large
    expect(theme.displayLarge!.fontFamily, 'Roboto');
    expect(theme.displayLarge!.fontSize, 57.0);
    expect(theme.displayLarge!.fontWeight, FontWeight.w400);
    expect(theme.displayLarge!.letterSpacing, -0.25);
    expect(theme.displayLarge!.height, 1.12);
    expect(theme.displayLarge!.textBaseline, TextBaseline.alphabetic);
    expect(theme.displayLarge!.leadingDistribution, TextLeadingDistribution.even);

    // Display medium
    expect(theme.displayMedium!.fontFamily, 'Roboto');
    expect(theme.displayMedium!.fontSize, 45.0);
    expect(theme.displayMedium!.fontWeight, FontWeight.w400);
    expect(theme.displayMedium!.letterSpacing, 0.0);
    expect(theme.displayMedium!.height, 1.16);
    expect(theme.displayMedium!.textBaseline, TextBaseline.alphabetic);
    expect(theme.displayMedium!.leadingDistribution, TextLeadingDistribution.even);

    // Display small
    expect(theme.displaySmall!.fontFamily, 'Roboto');
    expect(theme.displaySmall!.fontSize, 36.0);
    expect(theme.displaySmall!.fontWeight, FontWeight.w400);
    expect(theme.displaySmall!.letterSpacing, 0.0);
    expect(theme.displaySmall!.height, 1.22);
    expect(theme.displaySmall!.textBaseline, TextBaseline.alphabetic);
    expect(theme.displaySmall!.leadingDistribution, TextLeadingDistribution.even);

    // Headline large
    expect(theme.headlineLarge!.fontFamily, 'Roboto');
    expect(theme.headlineLarge!.fontSize, 32.0);
    expect(theme.headlineLarge!.fontWeight, FontWeight.w400);
    expect(theme.headlineLarge!.letterSpacing, 0.0);
    expect(theme.headlineLarge!.height, 1.25);
    expect(theme.headlineLarge!.textBaseline, TextBaseline.alphabetic);
    expect(theme.headlineLarge!.leadingDistribution, TextLeadingDistribution.even);

    // Headline medium
    expect(theme.headlineMedium!.fontFamily, 'Roboto');
    expect(theme.headlineMedium!.fontSize, 28.0);
    expect(theme.headlineMedium!.fontWeight, FontWeight.w400);
    expect(theme.headlineMedium!.letterSpacing, 0.0);
    expect(theme.headlineMedium!.height, 1.29);
    expect(theme.headlineMedium!.textBaseline, TextBaseline.alphabetic);
    expect(theme.headlineMedium!.leadingDistribution, TextLeadingDistribution.even);

    // Headline small
    expect(theme.headlineSmall!.fontFamily, 'Roboto');
    expect(theme.headlineSmall!.fontSize, 24.0);
    expect(theme.headlineSmall!.fontWeight, FontWeight.w400);
    expect(theme.headlineSmall!.letterSpacing, 0.0);
    expect(theme.headlineSmall!.height, 1.33);
    expect(theme.headlineSmall!.textBaseline, TextBaseline.alphabetic);
    expect(theme.headlineSmall!.leadingDistribution, TextLeadingDistribution.even);

    // Title large
    expect(theme.titleLarge!.fontFamily, 'Roboto');
    expect(theme.titleLarge!.fontSize, 22.0);
    expect(theme.titleLarge!.fontWeight, FontWeight.w400);
    expect(theme.titleLarge!.letterSpacing, 0.0);
    expect(theme.titleLarge!.height, 1.27);
    expect(theme.titleLarge!.textBaseline, TextBaseline.alphabetic);
    expect(theme.titleLarge!.leadingDistribution, TextLeadingDistribution.even);

    // Title medium
    expect(theme.titleMedium!.fontFamily, 'Roboto');
    expect(theme.titleMedium!.fontSize, 16.0);
    expect(theme.titleMedium!.fontWeight, FontWeight.w500);
    expect(theme.titleMedium!.letterSpacing, 0.15);
    expect(theme.titleMedium!.height, 1.50);
    expect(theme.titleMedium!.textBaseline, TextBaseline.alphabetic);
    expect(theme.titleMedium!.leadingDistribution, TextLeadingDistribution.even);

    // Title small
    expect(theme.titleSmall!.fontFamily, 'Roboto');
    expect(theme.titleSmall!.fontSize, 14.0);
    expect(theme.titleSmall!.fontWeight, FontWeight.w500);
    expect(theme.titleSmall!.letterSpacing, 0.1);
    expect(theme.titleSmall!.height, 1.43);
    expect(theme.titleSmall!.textBaseline, TextBaseline.alphabetic);
    expect(theme.titleSmall!.leadingDistribution, TextLeadingDistribution.even);

    // Label large
    expect(theme.labelLarge!.fontFamily, 'Roboto');
    expect(theme.labelLarge!.fontSize, 14.0);
    expect(theme.labelLarge!.fontWeight, FontWeight.w500);
    expect(theme.labelLarge!.letterSpacing, 0.1);
    expect(theme.labelLarge!.height, 1.43);
    expect(theme.labelLarge!.textBaseline, TextBaseline.alphabetic);
    expect(theme.labelLarge!.leadingDistribution, TextLeadingDistribution.even);

    // Label medium
    expect(theme.labelMedium!.fontFamily, 'Roboto');
    expect(theme.labelMedium!.fontSize, 12.0);
    expect(theme.labelMedium!.fontWeight, FontWeight.w500);
    expect(theme.labelMedium!.letterSpacing, 0.5);
    expect(theme.labelMedium!.height, 1.33);
    expect(theme.labelMedium!.textBaseline, TextBaseline.alphabetic);
    expect(theme.labelMedium!.leadingDistribution, TextLeadingDistribution.even);

    // Label small
    expect(theme.labelSmall!.fontFamily, 'Roboto');
    expect(theme.labelSmall!.fontSize, 11.0);
    expect(theme.labelSmall!.fontWeight, FontWeight.w500);
    expect(theme.labelSmall!.letterSpacing, 0.5);
    expect(theme.labelSmall!.height, 1.45);
    expect(theme.labelSmall!.textBaseline, TextBaseline.alphabetic);
    expect(theme.labelSmall!.leadingDistribution, TextLeadingDistribution.even);

    // Body large
    expect(theme.bodyLarge!.fontFamily, 'Roboto');
    expect(theme.bodyLarge!.fontSize, 16.0);
    expect(theme.bodyLarge!.fontWeight, FontWeight.w400);
    expect(theme.bodyLarge!.letterSpacing, 0.5);
    expect(theme.bodyLarge!.height, 1.50);
    expect(theme.bodyLarge!.textBaseline, TextBaseline.alphabetic);
    expect(theme.bodyLarge!.leadingDistribution, TextLeadingDistribution.even);

    // Body medium
    expect(theme.bodyMedium!.fontFamily, 'Roboto');
    expect(theme.bodyMedium!.fontSize, 14.0);
    expect(theme.bodyMedium!.fontWeight, FontWeight.w400);
    expect(theme.bodyMedium!.letterSpacing, 0.25);
    expect(theme.bodyMedium!.height, 1.43);
    expect(theme.bodyMedium!.textBaseline, TextBaseline.alphabetic);
    expect(theme.bodyMedium!.leadingDistribution, TextLeadingDistribution.even);

    // Body small
    expect(theme.bodySmall!.fontFamily, 'Roboto');
    expect(theme.bodySmall!.fontSize, 12.0);
    expect(theme.bodySmall!.fontWeight, FontWeight.w400);
    expect(theme.bodySmall!.letterSpacing, 0.4);
    expect(theme.bodySmall!.height, 1.33);
    expect(theme.bodySmall!.textBaseline, TextBaseline.alphabetic);
    expect(theme.bodySmall!.leadingDistribution, TextLeadingDistribution.even);
  });

  test('Default M3 light textTheme styles all use onSurface', () {
    final ThemeData theme = ThemeData(useMaterial3: true);
    final TextTheme textTheme = theme.textTheme;
    final Color dark = theme.colorScheme.onSurface;
    expect(textTheme.displayLarge!.color, dark);
    expect(textTheme.displayMedium!.color, dark);
    expect(textTheme.displaySmall!.color, dark);
    expect(textTheme.headlineLarge!.color, dark);
    expect(textTheme.headlineMedium!.color, dark);
    expect(textTheme.headlineSmall!.color, dark);
    expect(textTheme.titleLarge!.color, dark);
    expect(textTheme.titleMedium!.color, dark);
    expect(textTheme.titleSmall!.color, dark);
    expect(textTheme.bodyLarge!.color, dark);
    expect(textTheme.bodyMedium!.color, dark);
    expect(textTheme.bodySmall!.color, dark);
    expect(textTheme.labelLarge!.color, dark);
    expect(textTheme.labelMedium!.color, dark);
    expect(textTheme.labelSmall!.color, dark);
  });

  test('Default M3 dark textTheme styles all use onSurface', () {
    final ThemeData theme = ThemeData(useMaterial3: true, brightness: Brightness.dark);
    final TextTheme textTheme = theme.textTheme;
    final Color light = theme.colorScheme.onSurface;
    expect(textTheme.displayLarge!.color, light);
    expect(textTheme.displayMedium!.color, light);
    expect(textTheme.displaySmall!.color, light);
    expect(textTheme.headlineLarge!.color, light);
    expect(textTheme.headlineMedium!.color, light);
    expect(textTheme.headlineSmall!.color, light);
    expect(textTheme.titleLarge!.color, light);
    expect(textTheme.titleMedium!.color, light);
    expect(textTheme.titleSmall!.color, light);
    expect(textTheme.bodyLarge!.color, light);
    expect(textTheme.bodyMedium!.color, light);
    expect(textTheme.bodySmall!.color, light);
    expect(textTheme.labelLarge!.color, light);
    expect(textTheme.labelMedium!.color, light);
    expect(textTheme.labelSmall!.color, light);
  });
}
