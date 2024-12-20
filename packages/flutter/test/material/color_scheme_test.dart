// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:material_color_utilities/material_color_utilities.dart';

import '../image_data.dart';

void main() {
  test('ColorScheme lerp special cases', () {
    const ColorScheme scheme = ColorScheme.light();
    expect(identical(ColorScheme.lerp(scheme, scheme, 0.5), scheme), true);
  });

  test('light scheme matches the spec', () {
    // Colors should match the Material Design baseline default theme:
    // https://material.io/design/color/dark-theme.html#ui-application
    // with the new Material 3 colors defaulting to values from the M2
    // baseline.
    const ColorScheme scheme = ColorScheme.light();
    expect(scheme.brightness, Brightness.light);
    expect(scheme.primary, const Color(0xff6200ee));
    expect(scheme.onPrimary, const Color(0xffffffff));
    expect(scheme.primaryContainer, scheme.primary);
    expect(scheme.onPrimaryContainer, scheme.onPrimary);
    expect(scheme.primaryFixed, scheme.primary);
    expect(scheme.primaryFixedDim, scheme.primary);
    expect(scheme.onPrimaryFixed, scheme.onPrimary);
    expect(scheme.onPrimaryFixedVariant, scheme.onPrimary);
    expect(scheme.secondary, const Color(0xff03dac6));
    expect(scheme.onSecondary, const Color(0xff000000));
    expect(scheme.secondaryContainer, scheme.secondary);
    expect(scheme.onSecondaryContainer, scheme.onSecondary);
    expect(scheme.secondaryFixed, scheme.secondary);
    expect(scheme.secondaryFixedDim, scheme.secondary);
    expect(scheme.onSecondaryFixed, scheme.onSecondary);
    expect(scheme.onSecondaryFixedVariant, scheme.onSecondary);
    expect(scheme.tertiary, scheme.secondary);
    expect(scheme.onTertiary, scheme.onSecondary);
    expect(scheme.tertiaryContainer, scheme.tertiary);
    expect(scheme.onTertiaryContainer, scheme.onTertiary);
    expect(scheme.tertiaryFixed, scheme.tertiary);
    expect(scheme.tertiaryFixedDim, scheme.tertiary);
    expect(scheme.onTertiaryFixed, scheme.onTertiary);
    expect(scheme.onTertiaryFixedVariant, scheme.onTertiary);
    expect(scheme.error, const Color(0xffb00020));
    expect(scheme.onError, const Color(0xffffffff));
    expect(scheme.errorContainer, scheme.error);
    expect(scheme.onErrorContainer, scheme.onError);
    expect(scheme.background, const Color(0xffffffff));
    expect(scheme.onBackground, const Color(0xff000000));
    expect(scheme.surface, const Color(0xffffffff));
    expect(scheme.surfaceBright, scheme.surface);
    expect(scheme.surfaceDim, scheme.surface);
    expect(scheme.surfaceContainerLowest, scheme.surface);
    expect(scheme.surfaceContainerLow, scheme.surface);
    expect(scheme.surfaceContainer, scheme.surface);
    expect(scheme.surfaceContainerHigh, scheme.surface);
    expect(scheme.surfaceContainerHighest, scheme.surface);
    expect(scheme.onSurface, const Color(0xff000000));
    expect(scheme.surfaceVariant, scheme.surface);
    expect(scheme.onSurfaceVariant, scheme.onSurface);
    expect(scheme.outline, scheme.onBackground);
    expect(scheme.outlineVariant, scheme.onBackground);
    expect(scheme.shadow, const Color(0xff000000));
    expect(scheme.scrim, const Color(0xff000000));
    expect(scheme.inverseSurface, scheme.onSurface);
    expect(scheme.onInverseSurface, scheme.surface);
    expect(scheme.inversePrimary, scheme.onPrimary);
    expect(scheme.surfaceTint, scheme.primary);
  });

  test('dark scheme matches the spec', () {
    // Colors should match the Material Design baseline dark theme:
    // https://material.io/design/color/dark-theme.html#ui-application
    // with the new Material 3 colors defaulting to values from the M2
    // baseline.
    const ColorScheme scheme = ColorScheme.dark();
    expect(scheme.brightness, Brightness.dark);
    expect(scheme.primary, const Color(0xffbb86fc));
    expect(scheme.onPrimary, const Color(0xff000000));
    expect(scheme.primaryContainer, scheme.primary);
    expect(scheme.onPrimaryContainer, scheme.onPrimary);
    expect(scheme.primaryFixed, scheme.primary);
    expect(scheme.primaryFixedDim, scheme.primary);
    expect(scheme.onPrimaryFixed, scheme.onPrimary);
    expect(scheme.onPrimaryFixedVariant, scheme.onPrimary);
    expect(scheme.secondary, const Color(0xff03dac6));
    expect(scheme.onSecondary, const Color(0xff000000));
    expect(scheme.secondaryContainer, scheme.secondary);
    expect(scheme.onSecondaryContainer, scheme.onSecondary);
    expect(scheme.secondaryFixed, scheme.secondary);
    expect(scheme.secondaryFixedDim, scheme.secondary);
    expect(scheme.onSecondaryFixed, scheme.onSecondary);
    expect(scheme.onSecondaryFixedVariant, scheme.onSecondary);
    expect(scheme.tertiary, scheme.secondary);
    expect(scheme.onTertiary, scheme.onSecondary);
    expect(scheme.tertiaryContainer, scheme.tertiary);
    expect(scheme.onTertiaryContainer, scheme.onTertiary);
    expect(scheme.tertiaryFixed, scheme.tertiary);
    expect(scheme.tertiaryFixedDim, scheme.tertiary);
    expect(scheme.onTertiaryFixed, scheme.onTertiary);
    expect(scheme.onTertiaryFixedVariant, scheme.onTertiary);
    expect(scheme.error, const Color(0xffcf6679));
    expect(scheme.onError, const Color(0xff000000));
    expect(scheme.errorContainer, scheme.error);
    expect(scheme.onErrorContainer, scheme.onError);
    expect(scheme.background, const Color(0xff121212));
    expect(scheme.onBackground, const Color(0xffffffff));
    expect(scheme.surface, const Color(0xff121212));
    expect(scheme.surfaceBright, scheme.surface);
    expect(scheme.surfaceDim, scheme.surface);
    expect(scheme.surfaceContainerLowest, scheme.surface);
    expect(scheme.surfaceContainerLow, scheme.surface);
    expect(scheme.surfaceContainer, scheme.surface);
    expect(scheme.surfaceContainerHigh, scheme.surface);
    expect(scheme.surfaceContainerHighest, scheme.surface);
    expect(scheme.onSurface, const Color(0xffffffff));
    expect(scheme.surfaceVariant, scheme.surface);
    expect(scheme.onSurfaceVariant, scheme.onSurface);
    expect(scheme.outline, scheme.onBackground);
    expect(scheme.outlineVariant, scheme.onBackground);
    expect(scheme.shadow, const Color(0xff000000));
    expect(scheme.scrim, const Color(0xff000000));
    expect(scheme.inverseSurface, scheme.onSurface);
    expect(scheme.onInverseSurface, scheme.surface);
    expect(scheme.inversePrimary, scheme.onPrimary);
    expect(scheme.surfaceTint, scheme.primary);
  });

  test('high contrast light scheme matches the spec', () {
    // Colors are based off of the Material Design baseline default theme:
    // https://material.io/design/color/dark-theme.html#ui-application
    // with the new Material 3 colors defaulting to values from the M2
    // baseline.
    const ColorScheme scheme = ColorScheme.highContrastLight();
    expect(scheme.brightness, Brightness.light);
    expect(scheme.primary, const Color(0xff0000ba));
    expect(scheme.onPrimary, const Color(0xffffffff));
    expect(scheme.primaryContainer, scheme.primary);
    expect(scheme.onPrimaryContainer, scheme.onPrimary);
    expect(scheme.primaryFixed, scheme.primary);
    expect(scheme.primaryFixedDim, scheme.primary);
    expect(scheme.onPrimaryFixed, scheme.onPrimary);
    expect(scheme.onPrimaryFixedVariant, scheme.onPrimary);
    expect(scheme.secondary, const Color(0xff66fff9));
    expect(scheme.onSecondary, const Color(0xff000000));
    expect(scheme.secondaryContainer, scheme.secondary);
    expect(scheme.onSecondaryContainer, scheme.onSecondary);
    expect(scheme.secondaryFixed, scheme.secondary);
    expect(scheme.secondaryFixedDim, scheme.secondary);
    expect(scheme.onSecondaryFixed, scheme.onSecondary);
    expect(scheme.onSecondaryFixedVariant, scheme.onSecondary);
    expect(scheme.tertiary, scheme.secondary);
    expect(scheme.onTertiary, scheme.onSecondary);
    expect(scheme.tertiaryContainer, scheme.tertiary);
    expect(scheme.onTertiaryContainer, scheme.onTertiary);
    expect(scheme.tertiaryFixed, scheme.tertiary);
    expect(scheme.tertiaryFixedDim, scheme.tertiary);
    expect(scheme.onTertiaryFixed, scheme.onTertiary);
    expect(scheme.onTertiaryFixedVariant, scheme.onTertiary);
    expect(scheme.error, const Color(0xff790000));
    expect(scheme.onError, const Color(0xffffffff));
    expect(scheme.errorContainer, scheme.error);
    expect(scheme.onErrorContainer, scheme.onError);
    expect(scheme.background, const Color(0xffffffff));
    expect(scheme.onBackground, const Color(0xff000000));
    expect(scheme.surface, const Color(0xffffffff));
    expect(scheme.surfaceBright, scheme.surface);
    expect(scheme.surfaceDim, scheme.surface);
    expect(scheme.surfaceContainerLowest, scheme.surface);
    expect(scheme.surfaceContainerLow, scheme.surface);
    expect(scheme.surfaceContainer, scheme.surface);
    expect(scheme.surfaceContainerHigh, scheme.surface);
    expect(scheme.surfaceContainerHighest, scheme.surface);
    expect(scheme.onSurface, const Color(0xff000000));
    expect(scheme.surfaceVariant, scheme.surface);
    expect(scheme.onSurfaceVariant, scheme.onSurface);
    expect(scheme.outline, scheme.onBackground);
    expect(scheme.outlineVariant, scheme.onBackground);
    expect(scheme.shadow, const Color(0xff000000));
    expect(scheme.scrim, const Color(0xff000000));
    expect(scheme.inverseSurface, scheme.onSurface);
    expect(scheme.onInverseSurface, scheme.surface);
    expect(scheme.inversePrimary, scheme.onPrimary);
    expect(scheme.surfaceTint, scheme.primary);
  });

  test('high contrast dark scheme matches the spec', () {
    // Colors are based off of the Material Design baseline dark theme:
    // https://material.io/design/color/dark-theme.html#ui-application
    // with the new Material 3 colors defaulting to values from the M2
    // baseline.
    const ColorScheme scheme = ColorScheme.highContrastDark();
    expect(scheme.brightness, Brightness.dark);
    expect(scheme.primary, const Color(0xffefb7ff));
    expect(scheme.onPrimary, const Color(0xff000000));
    expect(scheme.primaryContainer, scheme.primary);
    expect(scheme.onPrimaryContainer, scheme.onPrimary);
    expect(scheme.primaryFixed, scheme.primary);
    expect(scheme.primaryFixedDim, scheme.primary);
    expect(scheme.onPrimaryFixed, scheme.onPrimary);
    expect(scheme.onPrimaryFixedVariant, scheme.onPrimary);
    expect(scheme.secondary, const Color(0xff66fff9));
    expect(scheme.onSecondary, const Color(0xff000000));
    expect(scheme.secondaryContainer, scheme.secondary);
    expect(scheme.onSecondaryContainer, scheme.onSecondary);
    expect(scheme.secondaryFixed, scheme.secondary);
    expect(scheme.secondaryFixedDim, scheme.secondary);
    expect(scheme.onSecondaryFixed, scheme.onSecondary);
    expect(scheme.onSecondaryFixedVariant, scheme.onSecondary);
    expect(scheme.tertiary, scheme.secondary);
    expect(scheme.onTertiary, scheme.onSecondary);
    expect(scheme.tertiaryContainer, scheme.tertiary);
    expect(scheme.onTertiaryContainer, scheme.onTertiary);
    expect(scheme.tertiaryFixed, scheme.tertiary);
    expect(scheme.tertiaryFixedDim, scheme.tertiary);
    expect(scheme.onTertiaryFixed, scheme.onTertiary);
    expect(scheme.onTertiaryFixedVariant, scheme.onTertiary);
    expect(scheme.error, const Color(0xff9b374d));
    expect(scheme.onError, const Color(0xff000000));
    expect(scheme.errorContainer, scheme.error);
    expect(scheme.onErrorContainer, scheme.onError);
    expect(scheme.background, const Color(0xff121212));
    expect(scheme.onBackground, const Color(0xffffffff));
    expect(scheme.surface, const Color(0xff121212));
    expect(scheme.surfaceBright, scheme.surface);
    expect(scheme.surfaceDim, scheme.surface);
    expect(scheme.surfaceContainerLowest, scheme.surface);
    expect(scheme.surfaceContainerLow, scheme.surface);
    expect(scheme.surfaceContainer, scheme.surface);
    expect(scheme.surfaceContainerHigh, scheme.surface);
    expect(scheme.surfaceContainerHighest, scheme.surface);
    expect(scheme.onSurface, const Color(0xffffffff));
    expect(scheme.surfaceVariant, scheme.surface);
    expect(scheme.onSurfaceVariant, scheme.onSurface);
    expect(scheme.outline, scheme.onBackground);
    expect(scheme.outlineVariant, scheme.onBackground);
    expect(scheme.shadow, const Color(0xff000000));
    expect(scheme.scrim, const Color(0xff000000));
    expect(scheme.inverseSurface, scheme.onSurface);
    expect(scheme.onInverseSurface, scheme.surface);
    expect(scheme.inversePrimary, scheme.onPrimary);
    expect(scheme.surfaceTint, scheme.primary);
  });

  test('can generate a light scheme from a seed color', () {
    final ColorScheme scheme = ColorScheme.fromSeed(seedColor: Colors.blue);
    expect(scheme.primary, const Color(0xff36618e));
    expect(scheme.onPrimary, const Color(0xffffffff));
    expect(scheme.primaryContainer, const Color(0xffd1e4ff));
    expect(scheme.onPrimaryContainer, const Color(0xff001d36));
    expect(scheme.primaryFixed, const Color(0xffd1e4ff));
    expect(scheme.primaryFixedDim, const Color(0xffa0cafd));
    expect(scheme.onPrimaryFixed, const Color(0xff001d36));
    expect(scheme.onPrimaryFixedVariant, const Color(0xff194975));
    expect(scheme.secondary, const Color(0xff535f70));
    expect(scheme.onSecondary, const Color(0xffffffff));
    expect(scheme.secondaryContainer, const Color(0xffd7e3f7));
    expect(scheme.onSecondaryContainer, const Color(0xff101c2b));
    expect(scheme.secondaryFixed, const Color(0xffd7e3f7));
    expect(scheme.secondaryFixedDim, const Color(0xffbbc7db));
    expect(scheme.onSecondaryFixed, const Color(0xff101c2b));
    expect(scheme.onSecondaryFixedVariant, const Color(0xff3b4858));
    expect(scheme.tertiary, const Color(0xff6b5778));
    expect(scheme.onTertiary, const Color(0xffffffff));
    expect(scheme.tertiaryContainer, const Color(0xfff2daff));
    expect(scheme.onTertiaryContainer, const Color(0xff251431));
    expect(scheme.tertiaryFixed, const Color(0xfff2daff));
    expect(scheme.tertiaryFixedDim, const Color(0xffd6bee4));
    expect(scheme.onTertiaryFixed, const Color(0xff251431));
    expect(scheme.onTertiaryFixedVariant, const Color(0xff523f5f));
    expect(scheme.error, const Color(0xffba1a1a));
    expect(scheme.onError, const Color(0xffffffff));
    expect(scheme.errorContainer, const Color(0xffffdad6));
    expect(scheme.onErrorContainer, const Color(0xff410002));
    expect(scheme.outline, const Color(0xff73777f));
    expect(scheme.outlineVariant, const Color(0xffc3c7cf));
    expect(scheme.background, const Color(0xfff8f9ff));
    expect(scheme.onBackground, const Color(0xff191c20));
    expect(scheme.surface, const Color(0xfff8f9ff));
    expect(scheme.surfaceBright, const Color(0xfff8f9ff));
    expect(scheme.surfaceDim, const Color(0xffd8dae0));
    expect(scheme.surfaceContainerLowest, const Color(0xffffffff));
    expect(scheme.surfaceContainerLow, const Color(0xfff2f3fa));
    expect(scheme.surfaceContainer, const Color(0xffeceef4));
    expect(scheme.surfaceContainerHigh, const Color(0xffe6e8ee));
    expect(scheme.surfaceContainerHighest, const Color(0xffe1e2e8));
    expect(scheme.onSurface, const Color(0xff191c20));
    expect(scheme.surfaceVariant, const Color(0xffdfe2eb));
    expect(scheme.onSurfaceVariant, const Color(0xff43474e));
    expect(scheme.inverseSurface, const Color(0xff2e3135));
    expect(scheme.onInverseSurface, const Color(0xffeff0f7));
    expect(scheme.inversePrimary, const Color(0xffa0cafd));
    expect(scheme.shadow, const Color(0xff000000));
    expect(scheme.scrim, const Color(0xff000000));
    expect(scheme.surfaceTint, const Color(0xff36618e));
    expect(scheme.brightness, Brightness.light);
  });

  test('copyWith overrides given colors', () {
    final ColorScheme scheme = const ColorScheme.light().copyWith(
      brightness: Brightness.dark,
      primary: const Color(0x00000001),
      onPrimary: const Color(0x00000002),
      primaryContainer: const Color(0x00000003),
      onPrimaryContainer: const Color(0x00000004),
      primaryFixed: const Color(0x0000001D),
      primaryFixedDim: const Color(0x0000001E),
      onPrimaryFixed: const Color(0x0000001F),
      onPrimaryFixedVariant: const Color(0x00000020),
      secondary: const Color(0x00000005),
      onSecondary: const Color(0x00000006),
      secondaryContainer: const Color(0x00000007),
      onSecondaryContainer: const Color(0x00000008),
      secondaryFixed: const Color(0x00000021),
      secondaryFixedDim: const Color(0x00000022),
      onSecondaryFixed: const Color(0x00000023),
      onSecondaryFixedVariant: const Color(0x00000024),
      tertiary: const Color(0x00000009),
      onTertiary: const Color(0x0000000A),
      tertiaryContainer: const Color(0x0000000B),
      onTertiaryContainer: const Color(0x0000000C),
      tertiaryFixed: const Color(0x00000025),
      tertiaryFixedDim: const Color(0x00000026),
      onTertiaryFixed: const Color(0x00000027),
      onTertiaryFixedVariant: const Color(0x00000028),
      error: const Color(0x0000000D),
      onError: const Color(0x0000000E),
      errorContainer: const Color(0x0000000F),
      onErrorContainer: const Color(0x00000010),
      background: const Color(0x00000011),
      onBackground: const Color(0x00000012),
      surface: const Color(0x00000013),
      surfaceDim: const Color(0x00000029),
      surfaceBright: const Color(0x0000002A),
      surfaceContainerLowest: const Color(0x0000002B),
      surfaceContainerLow: const Color(0x0000002C),
      surfaceContainer: const Color(0x0000002D),
      surfaceContainerHigh: const Color(0x0000002E),
      surfaceContainerHighest: const Color(0x0000002F),
      onSurface: const Color(0x00000014),
      surfaceVariant: const Color(0x00000015),
      onSurfaceVariant: const Color(0x00000016),
      outline: const Color(0x00000017),
      outlineVariant: const Color(0x00000117),
      shadow: const Color(0x00000018),
      scrim: const Color(0x00000118),
      inverseSurface: const Color(0x00000019),
      onInverseSurface: const Color(0x0000001A),
      inversePrimary: const Color(0x0000001B),
      surfaceTint: const Color(0x0000001C),
    );

    expect(scheme.brightness, Brightness.dark);
    expect(scheme.primary, const Color(0x00000001));
    expect(scheme.onPrimary, const Color(0x00000002));
    expect(scheme.primaryContainer, const Color(0x00000003));
    expect(scheme.onPrimaryContainer, const Color(0x00000004));
    expect(scheme.primaryFixed, const Color(0x0000001D));
    expect(scheme.primaryFixedDim, const Color(0x0000001E));
    expect(scheme.onPrimaryFixed, const Color(0x0000001F));
    expect(scheme.onPrimaryFixedVariant, const Color(0x00000020));
    expect(scheme.secondary, const Color(0x00000005));
    expect(scheme.onSecondary, const Color(0x00000006));
    expect(scheme.secondaryContainer, const Color(0x00000007));
    expect(scheme.onSecondaryContainer, const Color(0x00000008));
    expect(scheme.secondaryFixed, const Color(0x00000021));
    expect(scheme.secondaryFixedDim, const Color(0x00000022));
    expect(scheme.onSecondaryFixed, const Color(0x00000023));
    expect(scheme.onSecondaryFixedVariant, const Color(0x00000024));
    expect(scheme.tertiary, const Color(0x00000009));
    expect(scheme.onTertiary, const Color(0x0000000A));
    expect(scheme.tertiaryContainer, const Color(0x0000000B));
    expect(scheme.onTertiaryContainer, const Color(0x0000000C));
    expect(scheme.tertiaryFixed, const Color(0x00000025));
    expect(scheme.tertiaryFixedDim, const Color(0x00000026));
    expect(scheme.onTertiaryFixed, const Color(0x00000027));
    expect(scheme.onTertiaryFixedVariant, const Color(0x00000028));
    expect(scheme.error, const Color(0x0000000D));
    expect(scheme.onError, const Color(0x0000000E));
    expect(scheme.errorContainer, const Color(0x0000000F));
    expect(scheme.onErrorContainer, const Color(0x00000010));
    expect(scheme.background, const Color(0x00000011));
    expect(scheme.onBackground, const Color(0x00000012));
    expect(scheme.surface, const Color(0x00000013));
    expect(scheme.surfaceDim, const Color(0x00000029));
    expect(scheme.surfaceBright, const Color(0x0000002A));
    expect(scheme.surfaceContainerLowest, const Color(0x0000002B));
    expect(scheme.surfaceContainerLow, const Color(0x0000002C));
    expect(scheme.surfaceContainer, const Color(0x0000002D));
    expect(scheme.surfaceContainerHigh, const Color(0x0000002E));
    expect(scheme.surfaceContainerHighest, const Color(0x0000002F));
    expect(scheme.onSurface, const Color(0x00000014));
    expect(scheme.surfaceVariant, const Color(0x00000015));
    expect(scheme.onSurfaceVariant, const Color(0x00000016));
    expect(scheme.outline, const Color(0x00000017));
    expect(scheme.outlineVariant, const Color(0x00000117));
    expect(scheme.shadow, const Color(0x00000018));
    expect(scheme.scrim, const Color(0x00000118));
    expect(scheme.inverseSurface, const Color(0x00000019));
    expect(scheme.onInverseSurface, const Color(0x0000001A));
    expect(scheme.inversePrimary, const Color(0x0000001B));
    expect(scheme.surfaceTint, const Color(0x0000001C));
  });

  test('can generate a dark scheme from a seed color', () {
    final ColorScheme scheme = ColorScheme.fromSeed(
      seedColor: Colors.blue,
      brightness: Brightness.dark,
    );
    expect(scheme.primary, const Color(0xffa0cafd));
    expect(scheme.onPrimary, const Color(0xff003258));
    expect(scheme.primaryContainer, const Color(0xff194975));
    expect(scheme.onPrimaryContainer, const Color(0xffd1e4ff));
    expect(scheme.primaryFixed, const Color(0xffd1e4ff));
    expect(scheme.primaryFixedDim, const Color(0xffa0cafd));
    expect(scheme.onPrimaryFixed, const Color(0xff001d36));
    expect(scheme.onPrimaryFixedVariant, const Color(0xff194975));
    expect(scheme.secondary, const Color(0xffbbc7db));
    expect(scheme.onSecondary, const Color(0xff253140));
    expect(scheme.secondaryContainer, const Color(0xff3b4858));
    expect(scheme.onSecondaryContainer, const Color(0xffd7e3f7));
    expect(scheme.secondaryFixed, const Color(0xffd7e3f7));
    expect(scheme.secondaryFixedDim, const Color(0xffbbc7db));
    expect(scheme.onSecondaryFixed, const Color(0xff101c2b));
    expect(scheme.onSecondaryFixedVariant, const Color(0xff3b4858));
    expect(scheme.tertiary, const Color(0xffd6bee4));
    expect(scheme.onTertiary, const Color(0xff3b2948));
    expect(scheme.tertiaryContainer, const Color(0xff523f5f));
    expect(scheme.onTertiaryContainer, const Color(0xfff2daff));
    expect(scheme.tertiaryFixed, const Color(0xfff2daff));
    expect(scheme.tertiaryFixedDim, const Color(0xffd6bee4));
    expect(scheme.onTertiaryFixed, const Color(0xff251431));
    expect(scheme.onTertiaryFixedVariant, const Color(0xff523f5f));
    expect(scheme.error, const Color(0xffffb4ab));
    expect(scheme.onError, const Color(0xff690005));
    expect(scheme.errorContainer, const Color(0xff93000a));
    expect(scheme.onErrorContainer, const Color(0xffffdad6));
    expect(scheme.outline, const Color(0xff8d9199));
    expect(scheme.outlineVariant, const Color(0xff43474e));
    expect(scheme.background, const Color(0xff111418));
    expect(scheme.onBackground, const Color(0xffe1e2e8));
    expect(scheme.surface, const Color(0xff111418));
    expect(scheme.surfaceDim, const Color(0xff111418));
    expect(scheme.surfaceBright, const Color(0xff36393e));
    expect(scheme.surfaceContainerLowest, const Color(0xff0b0e13));
    expect(scheme.surfaceContainerLow, const Color(0xff191c20));
    expect(scheme.surfaceContainer, const Color(0xff1d2024));
    expect(scheme.surfaceContainerHigh, const Color(0xff272a2f));
    expect(scheme.surfaceContainerHighest, const Color(0xff32353a));
    expect(scheme.onSurface, const Color(0xffe1e2e8));
    expect(scheme.surfaceVariant, const Color(0xff43474e));
    expect(scheme.onSurfaceVariant, const Color(0xffc3c7cf));
    expect(scheme.inverseSurface, const Color(0xffe1e2e8));
    expect(scheme.onInverseSurface, const Color(0xff2e3135));
    expect(scheme.inversePrimary, const Color(0xff36618e));
    expect(scheme.shadow, const Color(0xff000000));
    expect(scheme.scrim, const Color(0xff000000));
    expect(scheme.surfaceTint, const Color(0xffa0cafd));
    expect(scheme.brightness, Brightness.dark);
  });

  test('can override specific colors in a generated scheme', () {
    final ColorScheme baseScheme = ColorScheme.fromSeed(seedColor: Colors.blue);
    const Color primaryOverride = Color(0xffabcdef);
    final ColorScheme scheme = ColorScheme.fromSeed(
      seedColor: Colors.blue,
      primary: primaryOverride,
    );
    expect(scheme.primary, primaryOverride);
    // The rest should be the same.
    expect(scheme.onPrimary, baseScheme.onPrimary);
    expect(scheme.primaryContainer, baseScheme.primaryContainer);
    expect(scheme.onPrimaryContainer, baseScheme.onPrimaryContainer);
    expect(scheme.primaryFixed, baseScheme.primaryFixed);
    expect(scheme.primaryFixedDim, baseScheme.primaryFixedDim);
    expect(scheme.onPrimaryFixed, baseScheme.onPrimaryFixed);
    expect(scheme.onPrimaryFixedVariant, baseScheme.onPrimaryFixedVariant);
    expect(scheme.secondary, baseScheme.secondary);
    expect(scheme.onSecondary, baseScheme.onSecondary);
    expect(scheme.secondaryContainer, baseScheme.secondaryContainer);
    expect(scheme.onSecondaryContainer, baseScheme.onSecondaryContainer);
    expect(scheme.secondaryFixed, baseScheme.secondaryFixed);
    expect(scheme.secondaryFixedDim, baseScheme.secondaryFixedDim);
    expect(scheme.onSecondaryFixed, baseScheme.onSecondaryFixed);
    expect(scheme.onSecondaryFixedVariant, baseScheme.onSecondaryFixedVariant);
    expect(scheme.tertiary, baseScheme.tertiary);
    expect(scheme.onTertiary, baseScheme.onTertiary);
    expect(scheme.tertiaryContainer, baseScheme.tertiaryContainer);
    expect(scheme.onTertiaryContainer, baseScheme.onTertiaryContainer);
    expect(scheme.tertiaryFixed, baseScheme.tertiaryFixed);
    expect(scheme.tertiaryFixedDim, baseScheme.tertiaryFixedDim);
    expect(scheme.onTertiaryFixed, baseScheme.onTertiaryFixed);
    expect(scheme.onTertiaryFixedVariant, baseScheme.onTertiaryFixedVariant);
    expect(scheme.error, baseScheme.error);
    expect(scheme.onError, baseScheme.onError);
    expect(scheme.errorContainer, baseScheme.errorContainer);
    expect(scheme.onErrorContainer, baseScheme.onErrorContainer);
    expect(scheme.outline, baseScheme.outline);
    expect(scheme.outlineVariant, baseScheme.outlineVariant);
    expect(scheme.background, baseScheme.background);
    expect(scheme.onBackground, baseScheme.onBackground);
    expect(scheme.surface, baseScheme.surface);
    expect(scheme.surfaceBright, baseScheme.surfaceBright);
    expect(scheme.surfaceDim, baseScheme.surfaceDim);
    expect(scheme.surfaceContainerLowest, baseScheme.surfaceContainerLowest);
    expect(scheme.surfaceContainerLow, baseScheme.surfaceContainerLow);
    expect(scheme.surfaceContainer, baseScheme.surfaceContainer);
    expect(scheme.surfaceContainerHigh, baseScheme.surfaceContainerHigh);
    expect(scheme.surfaceContainerHighest, baseScheme.surfaceContainerHighest);
    expect(scheme.onSurface, baseScheme.onSurface);
    expect(scheme.surfaceVariant, baseScheme.surfaceVariant);
    expect(scheme.onSurfaceVariant, baseScheme.onSurfaceVariant);
    expect(scheme.inverseSurface, baseScheme.inverseSurface);
    expect(scheme.onInverseSurface, baseScheme.onInverseSurface);
    expect(scheme.inversePrimary, baseScheme.inversePrimary);
    expect(scheme.shadow, baseScheme.shadow);
    expect(scheme.scrim, baseScheme.shadow);
    expect(scheme.surfaceTint, baseScheme.surfaceTint);
    expect(scheme.brightness, baseScheme.brightness);
  });

  test(
    'can generate a light scheme from an imageProvider',
    () async {
      final Uint8List blueSquareBytes = Uint8List.fromList(kBlueSquarePng);
      final ImageProvider image = MemoryImage(blueSquareBytes);

      final ColorScheme scheme = await ColorScheme.fromImageProvider(provider: image);

      expect(scheme.brightness, Brightness.light);
      expect(scheme.primary, const Color(0xff575992));
      expect(scheme.onPrimary, const Color(0xffffffff));
      expect(scheme.primaryContainer, const Color(0xffe1e0ff));
      expect(scheme.onPrimaryContainer, const Color(0xff13144b));
      expect(scheme.primaryFixed, const Color(0xffe1e0ff));
      expect(scheme.primaryFixedDim, const Color(0xffc0c1ff));
      expect(scheme.onPrimaryFixed, const Color(0xff13144b));
      expect(scheme.onPrimaryFixedVariant, const Color(0xff3f4178));
      expect(scheme.secondary, const Color(0xff5d5c72));
      expect(scheme.onSecondary, const Color(0xffffffff));
      expect(scheme.secondaryContainer, const Color(0xffe2e0f9));
      expect(scheme.onSecondaryContainer, const Color(0xff191a2c));
      expect(scheme.secondaryFixed, const Color(0xffe2e0f9));
      expect(scheme.secondaryFixedDim, const Color(0xffc6c4dd));
      expect(scheme.onSecondaryFixed, const Color(0xff191a2c));
      expect(scheme.onSecondaryFixedVariant, const Color(0xff454559));
      expect(scheme.tertiary, const Color(0xff79536a));
      expect(scheme.onTertiary, const Color(0xffffffff));
      expect(scheme.tertiaryContainer, const Color(0xffffd8ec));
      expect(scheme.onTertiaryContainer, const Color(0xff2e1125));
      expect(scheme.tertiaryFixed, const Color(0xffffd8ec));
      expect(scheme.tertiaryFixedDim, const Color(0xffe9b9d3));
      expect(scheme.onTertiaryFixed, const Color(0xff2e1125));
      expect(scheme.onTertiaryFixedVariant, const Color(0xff5f3c51));
      expect(scheme.error, const Color(0xffba1a1a));
      expect(scheme.onError, const Color(0xffffffff));
      expect(scheme.errorContainer, const Color(0xffffdad6));
      expect(scheme.onErrorContainer, const Color(0xff410002));
      expect(scheme.background, const Color(0xfffcf8ff));
      expect(scheme.onBackground, const Color(0xff1b1b21));
      expect(scheme.surface, const Color(0xfffcf8ff));
      expect(scheme.surfaceDim, const Color(0xffdcd9e0));
      expect(scheme.surfaceBright, const Color(0xfffcf8ff));
      expect(scheme.surfaceContainerLowest, const Color(0xffffffff));
      expect(scheme.surfaceContainerLow, const Color(0xfff6f2fa));
      expect(scheme.surfaceContainer, const Color(0xfff0ecf4));
      expect(scheme.surfaceContainerHigh, const Color(0xffeae7ef));
      expect(scheme.surfaceContainerHighest, const Color(0xffe4e1e9));
      expect(scheme.onSurface, const Color(0xff1b1b21));
      expect(scheme.surfaceVariant, const Color(0xffe4e1ec));
      expect(scheme.onSurfaceVariant, const Color(0xff46464f));
      expect(scheme.outline, const Color(0xff777680));
      expect(scheme.outlineVariant, const Color(0xffc8c5d0));
      expect(scheme.shadow, const Color(0xff000000));
      expect(scheme.scrim, const Color(0xff000000));
      expect(scheme.inverseSurface, const Color(0xff303036));
      expect(scheme.onInverseSurface, const Color(0xfff3eff7));
      expect(scheme.inversePrimary, const Color(0xffc0c1ff));
      expect(scheme.surfaceTint, const Color(0xff575992));
    },
    skip: isBrowser, // [intended] uses dart:typed_data.
  );

  test(
    'can generate a dark scheme from an imageProvider',
    () async {
      final Uint8List blueSquareBytes = Uint8List.fromList(kBlueSquarePng);
      final ImageProvider image = MemoryImage(blueSquareBytes);

      final ColorScheme scheme = await ColorScheme.fromImageProvider(
        provider: image,
        brightness: Brightness.dark,
      );

      expect(scheme.primary, const Color(0xffc0c1ff));
      expect(scheme.onPrimary, const Color(0xff292a60));
      expect(scheme.primaryContainer, const Color(0xff3f4178));
      expect(scheme.onPrimaryContainer, const Color(0xffe1e0ff));
      expect(scheme.primaryFixed, const Color(0xffe1e0ff));
      expect(scheme.primaryFixedDim, const Color(0xffc0c1ff));
      expect(scheme.onPrimaryFixed, const Color(0xff13144b));
      expect(scheme.onPrimaryFixedVariant, const Color(0xff3f4178));
      expect(scheme.secondary, const Color(0xffc6c4dd));
      expect(scheme.onSecondary, const Color(0xff2e2f42));
      expect(scheme.secondaryContainer, const Color(0xff454559));
      expect(scheme.onSecondaryContainer, const Color(0xffe2e0f9));
      expect(scheme.secondaryFixed, const Color(0xffe2e0f9));
      expect(scheme.secondaryFixedDim, const Color(0xffc6c4dd));
      expect(scheme.onSecondaryFixed, const Color(0xff191a2c));
      expect(scheme.onSecondaryFixedVariant, const Color(0xff454559));
      expect(scheme.tertiary, const Color(0xffe9b9d3));
      expect(scheme.onTertiary, const Color(0xff46263a));
      expect(scheme.tertiaryContainer, const Color(0xff5f3c51));
      expect(scheme.onTertiaryContainer, const Color(0xffffd8ec));
      expect(scheme.tertiaryFixed, const Color(0xffffd8ec));
      expect(scheme.tertiaryFixedDim, const Color(0xffe9b9d3));
      expect(scheme.onTertiaryFixed, const Color(0xff2e1125));
      expect(scheme.onTertiaryFixedVariant, const Color(0xff5f3c51));
      expect(scheme.error, const Color(0xffffb4ab));
      expect(scheme.onError, const Color(0xff690005));
      expect(scheme.errorContainer, const Color(0xff93000a));
      expect(scheme.onErrorContainer, const Color(0xffffdad6));
      expect(scheme.background, const Color(0xff131318));
      expect(scheme.onBackground, const Color(0xffe4e1e9));
      expect(scheme.surface, const Color(0xff131318));
      expect(scheme.surfaceDim, const Color(0xff131318));
      expect(scheme.surfaceBright, const Color(0xff39383f));
      expect(scheme.surfaceContainerLowest, const Color(0xff0e0e13));
      expect(scheme.surfaceContainerLow, const Color(0xff1b1b21));
      expect(scheme.surfaceContainer, const Color(0xff1f1f25));
      expect(scheme.surfaceContainerHigh, const Color(0xff2a292f));
      expect(scheme.surfaceContainerHighest, const Color(0xff35343a));
      expect(scheme.onSurface, const Color(0xffe4e1e9));
      expect(scheme.surfaceVariant, const Color(0xff46464f));
      expect(scheme.onSurfaceVariant, const Color(0xffc8c5d0));
      expect(scheme.outline, const Color(0xff918f9a));
      expect(scheme.outlineVariant, const Color(0xff46464f));
      expect(scheme.inverseSurface, const Color(0xffe4e1e9));
      expect(scheme.onInverseSurface, const Color(0xff303036));
      expect(scheme.inversePrimary, const Color(0xff575992));
      expect(scheme.surfaceTint, const Color(0xffc0c1ff));
    },
    skip: isBrowser, // [intended] uses dart:isolate and io.
  );

  test('fromSeed() asserts on invalid contrast levels', () {
    expect(() {
      ColorScheme.fromSeed(seedColor: Colors.blue, contrastLevel: -1.5);
    }, throwsAssertionError);

    expect(() {
      ColorScheme.fromSeed(seedColor: Colors.blue, contrastLevel: 1.5);
    }, throwsAssertionError);
  });

  test('fromImageProvider() asserts on invalid contrast levels', () async {
    final Uint8List blueSquareBytes = Uint8List.fromList(kBlueSquarePng);
    final ImageProvider image = MemoryImage(blueSquareBytes);

    expect(
      () => ColorScheme.fromImageProvider(provider: image, contrastLevel: -1.5),
      throwsAssertionError,
    );

    expect(
      () => ColorScheme.fromImageProvider(provider: image, contrastLevel: 1.5),
      throwsAssertionError,
    );
  });

  test('fromImageProvider() propagates TimeoutException when image cannot be rendered', () async {
    final Uint8List blueSquareBytes = Uint8List.fromList(kBlueSquarePng);

    // Corrupt the image's bytelist so it cannot be read.
    final Uint8List corruptImage = blueSquareBytes.sublist(5);
    final ImageProvider image = MemoryImage(corruptImage);

    expect(
      () async => ColorScheme.fromImageProvider(provider: image),
      throwsA(
        isA<Exception>().having(
          (Exception e) => e.toString(),
          'Timeout occurred trying to load image',
          contains('TimeoutException'),
        ),
      ),
    );
  });

  testWidgets(
    'generated scheme "on" colors meet a11y contrast guidelines',
    (WidgetTester tester) async {
      final ColorScheme colors = ColorScheme.fromSeed(seedColor: Colors.teal);

      Widget label(String text, Color textColor, Color background) {
        return Container(
          color: background,
          padding: const EdgeInsets.all(8),
          child: Text(text, style: TextStyle(color: textColor)),
        );
      }

      await tester.pumpWidget(
        MaterialApp(
          theme: ThemeData.from(colorScheme: colors),
          home: Scaffold(
            body: Column(
              children: <Widget>[
                label('primary', colors.onPrimary, colors.primary),
                label('secondary', colors.onSecondary, colors.secondary),
                label('tertiary', colors.onTertiary, colors.tertiary),
                label('error', colors.onError, colors.error),
                label('background', colors.onBackground, colors.background),
                label('surface', colors.onSurface, colors.surface),
              ],
            ),
          ),
        ),
      );
      await expectLater(tester, meetsGuideline(textContrastGuideline));
    },
    skip: isBrowser, // https://github.com/flutter/flutter/issues/44115
  );

  testWidgets(
    'Color values in ColorScheme.fromSeed with different variants matches values in DynamicScheme',
    (WidgetTester tester) async {
      const Color seedColor = Colors.orange;
      final Hct sourceColor = Hct.fromInt(seedColor.value);
      for (final DynamicSchemeVariant schemeVariant in DynamicSchemeVariant.values) {
        final DynamicScheme dynamicScheme = switch (schemeVariant) {
          DynamicSchemeVariant.tonalSpot => SchemeTonalSpot(
            sourceColorHct: sourceColor,
            isDark: false,
            contrastLevel: 0.0,
          ),
          DynamicSchemeVariant.fidelity => SchemeFidelity(
            sourceColorHct: sourceColor,
            isDark: false,
            contrastLevel: 0.0,
          ),
          DynamicSchemeVariant.content => SchemeContent(
            sourceColorHct: sourceColor,
            isDark: false,
            contrastLevel: 0.0,
          ),
          DynamicSchemeVariant.monochrome => SchemeMonochrome(
            sourceColorHct: sourceColor,
            isDark: false,
            contrastLevel: 0.0,
          ),
          DynamicSchemeVariant.neutral => SchemeNeutral(
            sourceColorHct: sourceColor,
            isDark: false,
            contrastLevel: 0.0,
          ),
          DynamicSchemeVariant.vibrant => SchemeVibrant(
            sourceColorHct: sourceColor,
            isDark: false,
            contrastLevel: 0.0,
          ),
          DynamicSchemeVariant.expressive => SchemeExpressive(
            sourceColorHct: sourceColor,
            isDark: false,
            contrastLevel: 0.0,
          ),
          DynamicSchemeVariant.rainbow => SchemeRainbow(
            sourceColorHct: sourceColor,
            isDark: false,
            contrastLevel: 0.0,
          ),
          DynamicSchemeVariant.fruitSalad => SchemeFruitSalad(
            sourceColorHct: sourceColor,
            isDark: false,
            contrastLevel: 0.0,
          ),
        };
        final ColorScheme colorScheme = ColorScheme.fromSeed(
          seedColor: seedColor,
          dynamicSchemeVariant: schemeVariant,
        );

        expect(colorScheme.primary.value, MaterialDynamicColors.primary.getArgb(dynamicScheme));
        expect(colorScheme.onPrimary.value, MaterialDynamicColors.onPrimary.getArgb(dynamicScheme));
        expect(
          colorScheme.primaryContainer.value,
          MaterialDynamicColors.primaryContainer.getArgb(dynamicScheme),
        );
        expect(
          colorScheme.onPrimaryContainer.value,
          MaterialDynamicColors.onPrimaryContainer.getArgb(dynamicScheme),
        );
        expect(
          colorScheme.primaryFixed.value,
          MaterialDynamicColors.primaryFixed.getArgb(dynamicScheme),
        );
        expect(
          colorScheme.primaryFixedDim.value,
          MaterialDynamicColors.primaryFixedDim.getArgb(dynamicScheme),
        );
        expect(
          colorScheme.onPrimaryFixed.value,
          MaterialDynamicColors.onPrimaryFixed.getArgb(dynamicScheme),
        );
        expect(
          colorScheme.onPrimaryFixedVariant.value,
          MaterialDynamicColors.onPrimaryFixedVariant.getArgb(dynamicScheme),
        );
        expect(colorScheme.secondary.value, MaterialDynamicColors.secondary.getArgb(dynamicScheme));
        expect(
          colorScheme.onSecondary.value,
          MaterialDynamicColors.onSecondary.getArgb(dynamicScheme),
        );
        expect(
          colorScheme.secondaryContainer.value,
          MaterialDynamicColors.secondaryContainer.getArgb(dynamicScheme),
        );
        expect(
          colorScheme.onSecondaryContainer.value,
          MaterialDynamicColors.onSecondaryContainer.getArgb(dynamicScheme),
        );
        expect(
          colorScheme.secondaryFixed.value,
          MaterialDynamicColors.secondaryFixed.getArgb(dynamicScheme),
        );
        expect(
          colorScheme.secondaryFixedDim.value,
          MaterialDynamicColors.secondaryFixedDim.getArgb(dynamicScheme),
        );
        expect(
          colorScheme.onSecondaryFixed.value,
          MaterialDynamicColors.onSecondaryFixed.getArgb(dynamicScheme),
        );
        expect(
          colorScheme.onSecondaryFixedVariant.value,
          MaterialDynamicColors.onSecondaryFixedVariant.getArgb(dynamicScheme),
        );
        expect(colorScheme.tertiary.value, MaterialDynamicColors.tertiary.getArgb(dynamicScheme));
        expect(
          colorScheme.onTertiary.value,
          MaterialDynamicColors.onTertiary.getArgb(dynamicScheme),
        );
        expect(
          colorScheme.tertiaryContainer.value,
          MaterialDynamicColors.tertiaryContainer.getArgb(dynamicScheme),
        );
        expect(
          colorScheme.onTertiaryContainer.value,
          MaterialDynamicColors.onTertiaryContainer.getArgb(dynamicScheme),
        );
        expect(
          colorScheme.tertiaryFixed.value,
          MaterialDynamicColors.tertiaryFixed.getArgb(dynamicScheme),
        );
        expect(
          colorScheme.tertiaryFixedDim.value,
          MaterialDynamicColors.tertiaryFixedDim.getArgb(dynamicScheme),
        );
        expect(
          colorScheme.onTertiaryFixed.value,
          MaterialDynamicColors.onTertiaryFixed.getArgb(dynamicScheme),
        );
        expect(
          colorScheme.onTertiaryFixedVariant.value,
          MaterialDynamicColors.onTertiaryFixedVariant.getArgb(dynamicScheme),
        );
        expect(colorScheme.error.value, MaterialDynamicColors.error.getArgb(dynamicScheme));
        expect(colorScheme.onError.value, MaterialDynamicColors.onError.getArgb(dynamicScheme));
        expect(
          colorScheme.errorContainer.value,
          MaterialDynamicColors.errorContainer.getArgb(dynamicScheme),
        );
        expect(
          colorScheme.onErrorContainer.value,
          MaterialDynamicColors.onErrorContainer.getArgb(dynamicScheme),
        );
        expect(
          colorScheme.background.value,
          MaterialDynamicColors.background.getArgb(dynamicScheme),
        );
        expect(
          colorScheme.onBackground.value,
          MaterialDynamicColors.onBackground.getArgb(dynamicScheme),
        );
        expect(colorScheme.surface.value, MaterialDynamicColors.surface.getArgb(dynamicScheme));
        expect(
          colorScheme.surfaceDim.value,
          MaterialDynamicColors.surfaceDim.getArgb(dynamicScheme),
        );
        expect(
          colorScheme.surfaceBright.value,
          MaterialDynamicColors.surfaceBright.getArgb(dynamicScheme),
        );
        expect(
          colorScheme.surfaceContainerLowest.value,
          MaterialDynamicColors.surfaceContainerLowest.getArgb(dynamicScheme),
        );
        expect(
          colorScheme.surfaceContainerLow.value,
          MaterialDynamicColors.surfaceContainerLow.getArgb(dynamicScheme),
        );
        expect(
          colorScheme.surfaceContainer.value,
          MaterialDynamicColors.surfaceContainer.getArgb(dynamicScheme),
        );
        expect(
          colorScheme.surfaceContainerHigh.value,
          MaterialDynamicColors.surfaceContainerHigh.getArgb(dynamicScheme),
        );
        expect(
          colorScheme.surfaceContainerHighest.value,
          MaterialDynamicColors.surfaceContainerHighest.getArgb(dynamicScheme),
        );
        expect(colorScheme.onSurface.value, MaterialDynamicColors.onSurface.getArgb(dynamicScheme));
        expect(
          colorScheme.surfaceVariant.value,
          MaterialDynamicColors.surfaceVariant.getArgb(dynamicScheme),
        );
        expect(
          colorScheme.onSurfaceVariant.value,
          MaterialDynamicColors.onSurfaceVariant.getArgb(dynamicScheme),
        );
        expect(colorScheme.outline.value, MaterialDynamicColors.outline.getArgb(dynamicScheme));
        expect(
          colorScheme.outlineVariant.value,
          MaterialDynamicColors.outlineVariant.getArgb(dynamicScheme),
        );
        expect(colorScheme.shadow.value, MaterialDynamicColors.shadow.getArgb(dynamicScheme));
        expect(colorScheme.scrim.value, MaterialDynamicColors.scrim.getArgb(dynamicScheme));
        expect(
          colorScheme.inverseSurface.value,
          MaterialDynamicColors.inverseSurface.getArgb(dynamicScheme),
        );
        expect(
          colorScheme.onInverseSurface.value,
          MaterialDynamicColors.inverseOnSurface.getArgb(dynamicScheme),
        );
        expect(
          colorScheme.inversePrimary.value,
          MaterialDynamicColors.inversePrimary.getArgb(dynamicScheme),
        );
      }
    },
  );

  testWidgets('ColorScheme.fromSeed with different variants spot checks', (
    WidgetTester tester,
  ) async {
    // Default (Variant.tonalSpot).
    await _testFilledButtonColor(
      tester,
      ColorScheme.fromSeed(seedColor: const Color(0xFF000000)),
      const Color(0xFF8C4A60),
    );
    await _testFilledButtonColor(
      tester,
      ColorScheme.fromSeed(seedColor: const Color(0xFF00FF00)),
      const Color(0xFF406836),
    );
    await _testFilledButtonColor(
      tester,
      ColorScheme.fromSeed(seedColor: const Color(0xFF6559F5)),
      const Color(0xFF5B5891),
    );
    await _testFilledButtonColor(
      tester,
      ColorScheme.fromSeed(seedColor: const Color(0xFFFFFFFF)),
      const Color(0xFF006874),
    );

    // Variant.fidelity.
    await _testFilledButtonColor(
      tester,
      ColorScheme.fromSeed(
        seedColor: const Color(0xFF000000),
        dynamicSchemeVariant: DynamicSchemeVariant.fidelity,
      ),
      const Color(0xFF000000),
    );
    await _testFilledButtonColor(
      tester,
      ColorScheme.fromSeed(
        seedColor: const Color(0xFF00FF00),
        dynamicSchemeVariant: DynamicSchemeVariant.fidelity,
      ),
      const Color(0xFF026E00),
    );
    await _testFilledButtonColor(
      tester,
      ColorScheme.fromSeed(
        seedColor: const Color(0xFF6559F5),
        dynamicSchemeVariant: DynamicSchemeVariant.fidelity,
      ),
      const Color(0xFF4C3CDB),
    );
    await _testFilledButtonColor(
      tester,
      ColorScheme.fromSeed(
        seedColor: const Color(0xFFFFFFFF),
        dynamicSchemeVariant: DynamicSchemeVariant.fidelity,
      ),
      const Color(0xFF5D5F5F),
    );
  });

  testWidgets('Colors in high-contrast color scheme matches colors in DynamicScheme', (
    WidgetTester tester,
  ) async {
    const Color seedColor = Colors.blue;
    final Hct sourceColor = Hct.fromInt(seedColor.value);

    void colorsMatchDynamicSchemeColors(
      DynamicSchemeVariant schemeVariant,
      Brightness brightness,
      double contrastLevel,
    ) {
      final bool isDark = brightness == Brightness.dark;
      final DynamicScheme dynamicScheme = switch (schemeVariant) {
        DynamicSchemeVariant.tonalSpot => SchemeTonalSpot(
          sourceColorHct: sourceColor,
          isDark: isDark,
          contrastLevel: contrastLevel,
        ),
        DynamicSchemeVariant.fidelity => SchemeFidelity(
          sourceColorHct: sourceColor,
          isDark: isDark,
          contrastLevel: contrastLevel,
        ),
        DynamicSchemeVariant.content => SchemeContent(
          sourceColorHct: sourceColor,
          isDark: isDark,
          contrastLevel: contrastLevel,
        ),
        DynamicSchemeVariant.monochrome => SchemeMonochrome(
          sourceColorHct: sourceColor,
          isDark: isDark,
          contrastLevel: contrastLevel,
        ),
        DynamicSchemeVariant.neutral => SchemeNeutral(
          sourceColorHct: sourceColor,
          isDark: isDark,
          contrastLevel: contrastLevel,
        ),
        DynamicSchemeVariant.vibrant => SchemeVibrant(
          sourceColorHct: sourceColor,
          isDark: isDark,
          contrastLevel: contrastLevel,
        ),
        DynamicSchemeVariant.expressive => SchemeExpressive(
          sourceColorHct: sourceColor,
          isDark: isDark,
          contrastLevel: contrastLevel,
        ),
        DynamicSchemeVariant.rainbow => SchemeRainbow(
          sourceColorHct: sourceColor,
          isDark: isDark,
          contrastLevel: contrastLevel,
        ),
        DynamicSchemeVariant.fruitSalad => SchemeFruitSalad(
          sourceColorHct: sourceColor,
          isDark: isDark,
          contrastLevel: contrastLevel,
        ),
      };

      final ColorScheme colorScheme = ColorScheme.fromSeed(
        seedColor: seedColor,
        brightness: brightness,
        dynamicSchemeVariant: schemeVariant,
        contrastLevel: contrastLevel,
      );

      expect(colorScheme.primary.value, MaterialDynamicColors.primary.getArgb(dynamicScheme));
      expect(colorScheme.onPrimary.value, MaterialDynamicColors.onPrimary.getArgb(dynamicScheme));
      expect(
        colorScheme.primaryContainer.value,
        MaterialDynamicColors.primaryContainer.getArgb(dynamicScheme),
      );
      expect(
        colorScheme.onPrimaryContainer.value,
        MaterialDynamicColors.onPrimaryContainer.getArgb(dynamicScheme),
      );
      expect(
        colorScheme.primaryFixed.value,
        MaterialDynamicColors.primaryFixed.getArgb(dynamicScheme),
      );
      expect(
        colorScheme.primaryFixedDim.value,
        MaterialDynamicColors.primaryFixedDim.getArgb(dynamicScheme),
      );
      expect(
        colorScheme.onPrimaryFixed.value,
        MaterialDynamicColors.onPrimaryFixed.getArgb(dynamicScheme),
      );
      expect(
        colorScheme.onPrimaryFixedVariant.value,
        MaterialDynamicColors.onPrimaryFixedVariant.getArgb(dynamicScheme),
      );
      expect(colorScheme.secondary.value, MaterialDynamicColors.secondary.getArgb(dynamicScheme));
      expect(
        colorScheme.onSecondary.value,
        MaterialDynamicColors.onSecondary.getArgb(dynamicScheme),
      );
      expect(
        colorScheme.secondaryContainer.value,
        MaterialDynamicColors.secondaryContainer.getArgb(dynamicScheme),
      );
      expect(
        colorScheme.onSecondaryContainer.value,
        MaterialDynamicColors.onSecondaryContainer.getArgb(dynamicScheme),
      );
      expect(
        colorScheme.secondaryFixed.value,
        MaterialDynamicColors.secondaryFixed.getArgb(dynamicScheme),
      );
      expect(
        colorScheme.secondaryFixedDim.value,
        MaterialDynamicColors.secondaryFixedDim.getArgb(dynamicScheme),
      );
      expect(
        colorScheme.onSecondaryFixed.value,
        MaterialDynamicColors.onSecondaryFixed.getArgb(dynamicScheme),
      );
      expect(
        colorScheme.onSecondaryFixedVariant.value,
        MaterialDynamicColors.onSecondaryFixedVariant.getArgb(dynamicScheme),
      );
      expect(colorScheme.tertiary.value, MaterialDynamicColors.tertiary.getArgb(dynamicScheme));
      expect(colorScheme.onTertiary.value, MaterialDynamicColors.onTertiary.getArgb(dynamicScheme));
      expect(
        colorScheme.tertiaryContainer.value,
        MaterialDynamicColors.tertiaryContainer.getArgb(dynamicScheme),
      );
      expect(
        colorScheme.onTertiaryContainer.value,
        MaterialDynamicColors.onTertiaryContainer.getArgb(dynamicScheme),
      );
      expect(
        colorScheme.tertiaryFixed.value,
        MaterialDynamicColors.tertiaryFixed.getArgb(dynamicScheme),
      );
      expect(
        colorScheme.tertiaryFixedDim.value,
        MaterialDynamicColors.tertiaryFixedDim.getArgb(dynamicScheme),
      );
      expect(
        colorScheme.onTertiaryFixed.value,
        MaterialDynamicColors.onTertiaryFixed.getArgb(dynamicScheme),
      );
      expect(
        colorScheme.onTertiaryFixedVariant.value,
        MaterialDynamicColors.onTertiaryFixedVariant.getArgb(dynamicScheme),
      );
      expect(colorScheme.error.value, MaterialDynamicColors.error.getArgb(dynamicScheme));
      expect(colorScheme.onError.value, MaterialDynamicColors.onError.getArgb(dynamicScheme));
      expect(
        colorScheme.errorContainer.value,
        MaterialDynamicColors.errorContainer.getArgb(dynamicScheme),
      );
      expect(
        colorScheme.onErrorContainer.value,
        MaterialDynamicColors.onErrorContainer.getArgb(dynamicScheme),
      );
      expect(colorScheme.background.value, MaterialDynamicColors.background.getArgb(dynamicScheme));
      expect(
        colorScheme.onBackground.value,
        MaterialDynamicColors.onBackground.getArgb(dynamicScheme),
      );
      expect(colorScheme.surface.value, MaterialDynamicColors.surface.getArgb(dynamicScheme));
      expect(colorScheme.surfaceDim.value, MaterialDynamicColors.surfaceDim.getArgb(dynamicScheme));
      expect(
        colorScheme.surfaceBright.value,
        MaterialDynamicColors.surfaceBright.getArgb(dynamicScheme),
      );
      expect(
        colorScheme.surfaceContainerLowest.value,
        MaterialDynamicColors.surfaceContainerLowest.getArgb(dynamicScheme),
      );
      expect(
        colorScheme.surfaceContainerLow.value,
        MaterialDynamicColors.surfaceContainerLow.getArgb(dynamicScheme),
      );
      expect(
        colorScheme.surfaceContainer.value,
        MaterialDynamicColors.surfaceContainer.getArgb(dynamicScheme),
      );
      expect(
        colorScheme.surfaceContainerHigh.value,
        MaterialDynamicColors.surfaceContainerHigh.getArgb(dynamicScheme),
      );
      expect(
        colorScheme.surfaceContainerHighest.value,
        MaterialDynamicColors.surfaceContainerHighest.getArgb(dynamicScheme),
      );
      expect(colorScheme.onSurface.value, MaterialDynamicColors.onSurface.getArgb(dynamicScheme));
      expect(
        colorScheme.surfaceVariant.value,
        MaterialDynamicColors.surfaceVariant.getArgb(dynamicScheme),
      );
      expect(
        colorScheme.onSurfaceVariant.value,
        MaterialDynamicColors.onSurfaceVariant.getArgb(dynamicScheme),
      );
      expect(colorScheme.outline.value, MaterialDynamicColors.outline.getArgb(dynamicScheme));
      expect(
        colorScheme.outlineVariant.value,
        MaterialDynamicColors.outlineVariant.getArgb(dynamicScheme),
      );
      expect(colorScheme.shadow.value, MaterialDynamicColors.shadow.getArgb(dynamicScheme));
      expect(colorScheme.scrim.value, MaterialDynamicColors.scrim.getArgb(dynamicScheme));
      expect(
        colorScheme.inverseSurface.value,
        MaterialDynamicColors.inverseSurface.getArgb(dynamicScheme),
      );
      expect(
        colorScheme.onInverseSurface.value,
        MaterialDynamicColors.inverseOnSurface.getArgb(dynamicScheme),
      );
      expect(
        colorScheme.inversePrimary.value,
        MaterialDynamicColors.inversePrimary.getArgb(dynamicScheme),
      );
    }

    for (final DynamicSchemeVariant schemeVariant in DynamicSchemeVariant.values) {
      colorsMatchDynamicSchemeColors(schemeVariant, Brightness.light, 1.0); // High contrast
      colorsMatchDynamicSchemeColors(schemeVariant, Brightness.dark, 1.0);

      colorsMatchDynamicSchemeColors(schemeVariant, Brightness.light, 0.5); // Medium contrast
      colorsMatchDynamicSchemeColors(schemeVariant, Brightness.dark, 0.5);
    }
  });

  testWidgets('ColorScheme.of(context) is equivalent to Theme.of(context).colorScheme', (
    WidgetTester tester,
  ) async {
    const Key sizedBoxKey = Key('sizedBox');
    final ColorScheme colorScheme = ColorScheme.fromSeed(seedColor: Colors.red);

    await tester.pumpWidget(
      MaterialApp(
        theme: ThemeData.from(colorScheme: colorScheme),
        home: const SizedBox(key: sizedBoxKey),
      ),
    );

    final BuildContext context = tester.element(find.byKey(sizedBoxKey));
    final ColorScheme colorSchemeOfTheme = Theme.of(context).colorScheme;
    final ColorScheme colorSchemeFromContext = ColorScheme.of(context);

    expect(colorSchemeOfTheme, colorScheme);
    expect(colorSchemeFromContext, colorScheme);
  });
}

Future<void> _testFilledButtonColor(
  WidgetTester tester,
  ColorScheme scheme,
  Color expectation,
) async {
  final GlobalKey key = GlobalKey();
  await tester.pumpWidget(Container()); // reset
  await tester.pumpWidget(
    MaterialApp(
      theme: ThemeData(colorScheme: scheme),
      home: FilledButton(key: key, onPressed: () {}, child: const SizedBox.square(dimension: 200)),
    ),
  );

  final Finder buttonMaterial = find.descendant(
    of: find.byType(FilledButton),
    matching: find.byType(Material),
  );
  final Material material = tester.widget<Material>(buttonMaterial);

  expect(material.color, expectation);
}
