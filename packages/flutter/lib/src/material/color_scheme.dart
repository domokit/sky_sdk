// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'input_decorator.dart';
/// @docImport 'scaffold.dart';
library;

import 'dart:async';
import 'dart:ui' as ui;

import 'package:flutter/foundation.dart';
import 'package:flutter/widgets.dart';
import 'package:material_color_utilities/material_color_utilities.dart';

import 'colors.dart';
import 'theme.dart';

/// The algorithm used to construct a [ColorScheme] in [ColorScheme.fromSeed].
///
/// The `tonalSpot` variant builds default Material scheme colors. These colors are
/// mapped to light or dark tones to achieve visually accessible color
/// pairings with sufficient contrast between foreground and background elements.
///
/// In some cases, the tones can prevent colors from appearing as intended,
/// such as when a color is too light to offer enough contrast for accessibility.
/// Color fidelity (`DynamicSchemeVariant.fidelity`) is a feature that adjusts
/// tones in these cases to produce the intended visual results without harming
/// visual contrast.
enum DynamicSchemeVariant {
  /// Default for Material theme colors. Builds pastel palettes with a low chroma.
  tonalSpot,

  /// The resulting color palettes match seed color, even if the seed color
  /// is very bright (high chroma).
  fidelity,

  /// All colors are grayscale, no chroma.
  monochrome,

  /// Close to grayscale, a hint of chroma.
  neutral,

  /// Pastel colors, high chroma palettes. The primary palette's chroma is at
  /// maximum. Use `fidelity` instead if tokens should alter their tone to match
  /// the palette vibrancy.
  vibrant,

  /// Pastel colors, medium chroma palettes. The primary palette's hue is
  /// different from the seed color, for variety.
  expressive,

  /// Almost identical to `fidelity`. Tokens and palettes match the seed color.
  /// [ColorScheme.primaryContainer] is the seed color, adjusted to ensure
  /// contrast with surfaces. The tertiary palette is analogue of the seed color.
  content,

  /// A playful theme - the seed color's hue does not appear in the theme.
  rainbow,

  /// A playful theme - the seed color's hue does not appear in the theme.
  fruitSalad,
}

/// {@template flutter.material.color_scheme.ColorScheme}
/// A set of 45 colors based on the
/// [Material spec](https://m3.material.io/styles/color/the-color-system/color-roles)
/// that can be used to configure the color properties of most components.
/// {@endtemplate}
///
/// ### Colors in Material 3
///
/// {@macro flutter.material.colors.colorRoles}
///
/// The main accent color groups in the scheme are [primary], [secondary],
/// and [tertiary].
///
/// * Primary colors are used for key components across the UI, such as the FAB,
///   prominent buttons, and active states.
///
/// * Secondary colors are used for less prominent components in the UI, such as
///   filter chips, while expanding the opportunity for color expression.
///
/// * Tertiary colors are used for contrasting accents that can be used to
///   balance primary and secondary colors or bring heightened attention to
///   an element, such as an input field. The tertiary colors are left
///   for makers to use at their discretion and are intended to support
///   broader color expression in products.
///
/// Each accent color group (primary, secondary and tertiary) includes '-Fixed'
/// '-Dim' color roles, such as [primaryFixed] and [primaryFixedDim]. Fixed roles
/// are appropriate to use in places where Container roles are normally used,
/// but they stay the same color between light and dark themes. The '-Dim' roles
/// provide a stronger, more emphasized color with the same fixed behavior.
///
/// The remaining colors of the scheme are composed of neutral colors used for
/// backgrounds and surfaces, as well as specific colors for errors, dividers
/// and shadows. Surface colors are used for backgrounds and large, low-emphasis
/// areas of the screen.
///
/// Material 3 also introduces tone-based surfaces and surface containers.
/// They replace the old opacity-based model which applied a tinted overlay on
/// top of surfaces based on their elevation. These colors include: [surfaceBright],
/// [surfaceDim], [surfaceContainerLowest], [surfaceContainerLow], [surfaceContainer],
/// [surfaceContainerHigh], and [surfaceContainerHighest].
///
/// Many of the colors have matching 'on' colors, which are used for drawing
/// content on top of the matching color. For example, if something is using
/// [primary] for a background color, [onPrimary] would be used to paint text
/// and icons on top of it. For this reason, the 'on' colors should have a
/// contrast ratio with their matching colors of at least 4.5:1 in order to
/// be readable. On '-FixedVariant' roles, such as [onPrimaryFixedVariant],
/// also have the same color between light and dark themes, but compared
/// with on '-Fixed' roles, such as [onPrimaryFixed], they provide a
/// lower-emphasis option for text and icons.
///
/// {@tool dartpad}
/// This example shows all Material [ColorScheme] roles in light and dark
/// brightnesses.
///
/// ** See code in examples/api/lib/material/color_scheme/color_scheme.0.dart **
/// {@end-tool}
///
/// ### Setting Colors in Flutter
///
///{@macro flutter.material.colors.settingColors}
@immutable
class ColorScheme with Diagnosticable {
  /// Create a ColorScheme instance from the given colors.
  ///
  /// [ColorScheme.fromSeed] can be used as a simpler way to create a full
  /// color scheme derived from a single seed color.
  ///
  /// For the color parameters that are nullable, it is still recommended
  /// that applications provide values for them. They are only nullable due
  /// to backwards compatibility concerns.
  ///
  /// If a color is not provided, the closest fallback color from the given
  /// colors will be used for it (e.g. [primaryContainer] will default
  /// to [primary]). Material Design 3 makes use of these colors for many
  /// component defaults, so for the best results the application should
  /// supply colors for all the parameters. An easy way to ensure this is to
  /// use [ColorScheme.fromSeed] to generate a full set of colors.
  ///
  /// During the migration to Material Design 3, if an app's
  /// [ThemeData.useMaterial3] is false, then components will only
  /// use the following colors for defaults:
  ///
  /// * [primary]
  /// * [onPrimary]
  /// * [secondary]
  /// * [onSecondary]
  /// * [error]
  /// * [onError]
  /// * [surface]
  /// * [onSurface]
  /// DEPRECATED:
  /// * [background]
  /// * [onBackground]
  const ColorScheme({
    required this.brightness,
    required this.primary,
    required this.onPrimary,
    Color? primaryContainer,
    Color? onPrimaryContainer,
    Color? primaryFixed,
    Color? primaryFixedDim,
    Color? onPrimaryFixed,
    Color? onPrimaryFixedVariant,
    required this.secondary,
    required this.onSecondary,
    Color? secondaryContainer,
    Color? onSecondaryContainer,
    Color? secondaryFixed,
    Color? secondaryFixedDim,
    Color? onSecondaryFixed,
    Color? onSecondaryFixedVariant,
    Color? tertiary,
    Color? onTertiary,
    Color? tertiaryContainer,
    Color? onTertiaryContainer,
    Color? tertiaryFixed,
    Color? tertiaryFixedDim,
    Color? onTertiaryFixed,
    Color? onTertiaryFixedVariant,
    required this.error,
    required this.onError,
    Color? errorContainer,
    Color? onErrorContainer,
    required this.surface,
    required this.onSurface,
    Color? surfaceDim,
    Color? surfaceBright,
    Color? surfaceContainerLowest,
    Color? surfaceContainerLow,
    Color? surfaceContainer,
    Color? surfaceContainerHigh,
    Color? surfaceContainerHighest,
    Color? onSurfaceVariant,
    Color? outline,
    Color? outlineVariant,
    Color? shadow,
    Color? scrim,
    Color? inverseSurface,
    Color? onInverseSurface,
    Color? inversePrimary,
    Color? surfaceTint,
    @Deprecated(
      'Use surface instead. '
      'This feature was deprecated after v3.18.0-0.1.pre.',
    )
    Color? background,
    @Deprecated(
      'Use onSurface instead. '
      'This feature was deprecated after v3.18.0-0.1.pre.',
    )
    Color? onBackground,
    @Deprecated(
      'Use surfaceContainerHighest instead. '
      'This feature was deprecated after v3.18.0-0.1.pre.',
    )
    Color? surfaceVariant,
  }) : _primaryContainer = primaryContainer,
       _onPrimaryContainer = onPrimaryContainer,
       _primaryFixed = primaryFixed,
       _primaryFixedDim = primaryFixedDim,
       _onPrimaryFixed = onPrimaryFixed,
       _onPrimaryFixedVariant = onPrimaryFixedVariant,
       _secondaryContainer = secondaryContainer,
       _onSecondaryContainer = onSecondaryContainer,
       _secondaryFixed = secondaryFixed,
       _secondaryFixedDim = secondaryFixedDim,
       _onSecondaryFixed = onSecondaryFixed,
       _onSecondaryFixedVariant = onSecondaryFixedVariant,
       _tertiary = tertiary,
       _onTertiary = onTertiary,
       _tertiaryContainer = tertiaryContainer,
       _onTertiaryContainer = onTertiaryContainer,
       _tertiaryFixed = tertiaryFixed,
       _tertiaryFixedDim = tertiaryFixedDim,
       _onTertiaryFixed = onTertiaryFixed,
       _onTertiaryFixedVariant = onTertiaryFixedVariant,
       _errorContainer = errorContainer,
       _onErrorContainer = onErrorContainer,
       _surfaceDim = surfaceDim,
       _surfaceBright = surfaceBright,
       _surfaceContainerLowest = surfaceContainerLowest,
       _surfaceContainerLow = surfaceContainerLow,
       _surfaceContainer = surfaceContainer,
       _surfaceContainerHigh = surfaceContainerHigh,
       _surfaceContainerHighest = surfaceContainerHighest,
       _onSurfaceVariant = onSurfaceVariant,
       _outline = outline,
       _outlineVariant = outlineVariant,
       _shadow = shadow,
       _scrim = scrim,
       _inverseSurface = inverseSurface,
       _onInverseSurface = onInverseSurface,
       _inversePrimary = inversePrimary,
       _surfaceTint = surfaceTint,
       // DEPRECATED (newest deprecations at the bottom)
       _background = background,
       _onBackground = onBackground,
       _surfaceVariant = surfaceVariant;

  /// Generate a [ColorScheme] derived from the given `seedColor`.
  ///
  /// Using the `seedColor` as a starting point, a set of tonal palettes are
  /// constructed. By default, the tonal palettes are based on the Material 3
  /// Color system and provide all of the [ColorScheme] colors. These colors are
  /// designed to work well together and meet contrast requirements for
  /// accessibility.
  ///
  /// If any of the optional color parameters are non-null they will be
  /// used in place of the generated colors for that field in the resulting
  /// color scheme. This allows apps to override specific colors for their
  /// needs.
  ///
  /// Given the nature of the algorithm, the `seedColor` may not wind up as
  /// one of the ColorScheme colors.
  ///
  /// The `dynamicSchemeVariant` parameter creates different types of
  /// [DynamicScheme]s, which are used to generate different styles of [ColorScheme]s.
  /// By default, `dynamicSchemeVariant` is set to `tonalSpot`. A [ColorScheme]
  /// constructed by `dynamicSchemeVariant.tonalSpot` has pastel palettes and
  /// won't be too "colorful" even if the `seedColor` has a high chroma value.
  /// If the resulting color scheme is too dark, consider setting `dynamicSchemeVariant`
  /// to [DynamicSchemeVariant.fidelity], whose palettes match the seed color.
  ///
  /// The `contrastLevel` parameter indicates the contrast level between color
  /// pairs, such as [primary] and [onPrimary]. 0.0 is the default (normal);
  /// -1.0 is the lowest; 1.0 is the highest. From Material Design guideline, the
  /// medium and high contrast correspond to 0.5 and 1.0 respectively.
  ///
  /// {@tool dartpad}
  /// This sample shows how to use [ColorScheme.fromSeed] to create dynamic
  /// color schemes with different [DynamicSchemeVariant]s and different
  /// contrast level.
  ///
  /// ** See code in examples/api/lib/material/color_scheme/color_scheme.0.dart **
  /// {@end-tool}
  ///
  /// See also:
  ///
  ///  * <https://m3.material.io/styles/color/the-color-system/color-roles>, the
  ///    Material 3 Color system specification.
  ///  * <https://pub.dev/packages/material_color_utilities>, the package
  ///    used to generate the tonal palettes needed for the scheme.
  factory ColorScheme.fromSeed({
    required Color seedColor,
    Brightness brightness = Brightness.light,
    DynamicSchemeVariant dynamicSchemeVariant = DynamicSchemeVariant.tonalSpot,
    double contrastLevel = 0.0,
    Color? primary,
    Color? onPrimary,
    Color? primaryContainer,
    Color? onPrimaryContainer,
    Color? primaryFixed,
    Color? primaryFixedDim,
    Color? onPrimaryFixed,
    Color? onPrimaryFixedVariant,
    Color? secondary,
    Color? onSecondary,
    Color? secondaryContainer,
    Color? onSecondaryContainer,
    Color? secondaryFixed,
    Color? secondaryFixedDim,
    Color? onSecondaryFixed,
    Color? onSecondaryFixedVariant,
    Color? tertiary,
    Color? onTertiary,
    Color? tertiaryContainer,
    Color? onTertiaryContainer,
    Color? tertiaryFixed,
    Color? tertiaryFixedDim,
    Color? onTertiaryFixed,
    Color? onTertiaryFixedVariant,
    Color? error,
    Color? onError,
    Color? errorContainer,
    Color? onErrorContainer,
    Color? outline,
    Color? outlineVariant,
    Color? surface,
    Color? onSurface,
    Color? surfaceDim,
    Color? surfaceBright,
    Color? surfaceContainerLowest,
    Color? surfaceContainerLow,
    Color? surfaceContainer,
    Color? surfaceContainerHigh,
    Color? surfaceContainerHighest,
    Color? onSurfaceVariant,
    Color? inverseSurface,
    Color? onInverseSurface,
    Color? inversePrimary,
    Color? shadow,
    Color? scrim,
    Color? surfaceTint,
    @Deprecated(
      'Use surface instead. '
      'This feature was deprecated after v3.18.0-0.1.pre.',
    )
    Color? background,
    @Deprecated(
      'Use onSurface instead. '
      'This feature was deprecated after v3.18.0-0.1.pre.',
    )
    Color? onBackground,
    @Deprecated(
      'Use surfaceContainerHighest instead. '
      'This feature was deprecated after v3.18.0-0.1.pre.',
    )
    Color? surfaceVariant,
  }) {
    final DynamicScheme scheme = _buildDynamicScheme(
      brightness,
      seedColor,
      dynamicSchemeVariant,
      contrastLevel,
    );

    return ColorScheme(
      primary: primary ?? Color(MaterialDynamicColors.primary.getArgb(scheme)),
      onPrimary: onPrimary ?? Color(MaterialDynamicColors.onPrimary.getArgb(scheme)),
      primaryContainer:
          primaryContainer ?? Color(MaterialDynamicColors.primaryContainer.getArgb(scheme)),
      onPrimaryContainer:
          onPrimaryContainer ?? Color(MaterialDynamicColors.onPrimaryContainer.getArgb(scheme)),
      primaryFixed: primaryFixed ?? Color(MaterialDynamicColors.primaryFixed.getArgb(scheme)),
      primaryFixedDim:
          primaryFixedDim ?? Color(MaterialDynamicColors.primaryFixedDim.getArgb(scheme)),
      onPrimaryFixed: onPrimaryFixed ?? Color(MaterialDynamicColors.onPrimaryFixed.getArgb(scheme)),
      onPrimaryFixedVariant:
          onPrimaryFixedVariant ??
          Color(MaterialDynamicColors.onPrimaryFixedVariant.getArgb(scheme)),
      secondary: secondary ?? Color(MaterialDynamicColors.secondary.getArgb(scheme)),
      onSecondary: onSecondary ?? Color(MaterialDynamicColors.onSecondary.getArgb(scheme)),
      secondaryContainer:
          secondaryContainer ?? Color(MaterialDynamicColors.secondaryContainer.getArgb(scheme)),
      onSecondaryContainer:
          onSecondaryContainer ?? Color(MaterialDynamicColors.onSecondaryContainer.getArgb(scheme)),
      secondaryFixed: secondaryFixed ?? Color(MaterialDynamicColors.secondaryFixed.getArgb(scheme)),
      secondaryFixedDim:
          secondaryFixedDim ?? Color(MaterialDynamicColors.secondaryFixedDim.getArgb(scheme)),
      onSecondaryFixed:
          onSecondaryFixed ?? Color(MaterialDynamicColors.onSecondaryFixed.getArgb(scheme)),
      onSecondaryFixedVariant:
          onSecondaryFixedVariant ??
          Color(MaterialDynamicColors.onSecondaryFixedVariant.getArgb(scheme)),
      tertiary: tertiary ?? Color(MaterialDynamicColors.tertiary.getArgb(scheme)),
      onTertiary: onTertiary ?? Color(MaterialDynamicColors.onTertiary.getArgb(scheme)),
      tertiaryContainer:
          tertiaryContainer ?? Color(MaterialDynamicColors.tertiaryContainer.getArgb(scheme)),
      onTertiaryContainer:
          onTertiaryContainer ?? Color(MaterialDynamicColors.onTertiaryContainer.getArgb(scheme)),
      tertiaryFixed: tertiaryFixed ?? Color(MaterialDynamicColors.tertiaryFixed.getArgb(scheme)),
      tertiaryFixedDim:
          tertiaryFixedDim ?? Color(MaterialDynamicColors.tertiaryFixedDim.getArgb(scheme)),
      onTertiaryFixed:
          onTertiaryFixed ?? Color(MaterialDynamicColors.onTertiaryFixed.getArgb(scheme)),
      onTertiaryFixedVariant:
          onTertiaryFixedVariant ??
          Color(MaterialDynamicColors.onTertiaryFixedVariant.getArgb(scheme)),
      error: error ?? Color(MaterialDynamicColors.error.getArgb(scheme)),
      onError: onError ?? Color(MaterialDynamicColors.onError.getArgb(scheme)),
      errorContainer: errorContainer ?? Color(MaterialDynamicColors.errorContainer.getArgb(scheme)),
      onErrorContainer:
          onErrorContainer ?? Color(MaterialDynamicColors.onErrorContainer.getArgb(scheme)),
      outline: outline ?? Color(MaterialDynamicColors.outline.getArgb(scheme)),
      outlineVariant: outlineVariant ?? Color(MaterialDynamicColors.outlineVariant.getArgb(scheme)),
      surface: surface ?? Color(MaterialDynamicColors.surface.getArgb(scheme)),
      surfaceDim: surfaceDim ?? Color(MaterialDynamicColors.surfaceDim.getArgb(scheme)),
      surfaceBright: surfaceBright ?? Color(MaterialDynamicColors.surfaceBright.getArgb(scheme)),
      surfaceContainerLowest:
          surfaceContainerLowest ??
          Color(MaterialDynamicColors.surfaceContainerLowest.getArgb(scheme)),
      surfaceContainerLow:
          surfaceContainerLow ?? Color(MaterialDynamicColors.surfaceContainerLow.getArgb(scheme)),
      surfaceContainer:
          surfaceContainer ?? Color(MaterialDynamicColors.surfaceContainer.getArgb(scheme)),
      surfaceContainerHigh:
          surfaceContainerHigh ?? Color(MaterialDynamicColors.surfaceContainerHigh.getArgb(scheme)),
      surfaceContainerHighest:
          surfaceContainerHighest ??
          Color(MaterialDynamicColors.surfaceContainerHighest.getArgb(scheme)),
      onSurface: onSurface ?? Color(MaterialDynamicColors.onSurface.getArgb(scheme)),
      onSurfaceVariant:
          onSurfaceVariant ?? Color(MaterialDynamicColors.onSurfaceVariant.getArgb(scheme)),
      inverseSurface: inverseSurface ?? Color(MaterialDynamicColors.inverseSurface.getArgb(scheme)),
      onInverseSurface:
          onInverseSurface ?? Color(MaterialDynamicColors.inverseOnSurface.getArgb(scheme)),
      inversePrimary: inversePrimary ?? Color(MaterialDynamicColors.inversePrimary.getArgb(scheme)),
      shadow: shadow ?? Color(MaterialDynamicColors.shadow.getArgb(scheme)),
      scrim: scrim ?? Color(MaterialDynamicColors.scrim.getArgb(scheme)),
      surfaceTint: surfaceTint ?? Color(MaterialDynamicColors.primary.getArgb(scheme)),
      brightness: brightness,
      // DEPRECATED (newest deprecations at the bottom)
      background: background ?? Color(MaterialDynamicColors.background.getArgb(scheme)),
      onBackground: onBackground ?? Color(MaterialDynamicColors.onBackground.getArgb(scheme)),
      surfaceVariant: surfaceVariant ?? Color(MaterialDynamicColors.surfaceVariant.getArgb(scheme)),
    );
  }

  /// Create a light ColorScheme based on a purple primary color that matches the
  /// [baseline Material 2 color scheme](https://material.io/design/color/the-color-system.html#color-theme-creation).
  ///
  /// This constructor shouldn't be used to update the Material 3 color scheme.
  ///
  /// For Material 3, use [ColorScheme.fromSeed] to create a color scheme
  /// from a single seed color based on the Material 3 color system.
  ///
  /// {@tool snippet}
  /// This example demonstrates how to create a color scheme similar to [ColorScheme.light]
  /// using the [ColorScheme.fromSeed] constructor:
  ///
  /// ```dart
  /// colorScheme: ColorScheme.fromSeed(seedColor: const Color(0xff6200ee)).copyWith(
  ///   primaryContainer: const Color(0xff6200ee),
  ///   onPrimaryContainer: Colors.white,
  ///   secondaryContainer: const Color(0xff03dac6),
  ///   onSecondaryContainer: Colors.black,
  ///   error: const Color(0xffb00020),
  ///   onError: Colors.white,
  /// ),
  /// ```
  /// {@end-tool}
  const ColorScheme.light({
    this.brightness = Brightness.light,
    this.primary = const Color(0xff6200ee),
    this.onPrimary = Colors.white,
    Color? primaryContainer,
    Color? onPrimaryContainer,
    Color? primaryFixed,
    Color? primaryFixedDim,
    Color? onPrimaryFixed,
    Color? onPrimaryFixedVariant,
    this.secondary = const Color(0xff03dac6),
    this.onSecondary = Colors.black,
    Color? secondaryContainer,
    Color? onSecondaryContainer,
    Color? secondaryFixed,
    Color? secondaryFixedDim,
    Color? onSecondaryFixed,
    Color? onSecondaryFixedVariant,
    Color? tertiary,
    Color? onTertiary,
    Color? tertiaryContainer,
    Color? onTertiaryContainer,
    Color? tertiaryFixed,
    Color? tertiaryFixedDim,
    Color? onTertiaryFixed,
    Color? onTertiaryFixedVariant,
    this.error = const Color(0xffb00020),
    this.onError = Colors.white,
    Color? errorContainer,
    Color? onErrorContainer,
    this.surface = Colors.white,
    this.onSurface = Colors.black,
    Color? surfaceDim,
    Color? surfaceBright,
    Color? surfaceContainerLowest,
    Color? surfaceContainerLow,
    Color? surfaceContainer,
    Color? surfaceContainerHigh,
    Color? surfaceContainerHighest,
    Color? onSurfaceVariant,
    Color? outline,
    Color? outlineVariant,
    Color? shadow,
    Color? scrim,
    Color? inverseSurface,
    Color? onInverseSurface,
    Color? inversePrimary,
    Color? surfaceTint,
    @Deprecated(
      'Use surface instead. '
      'This feature was deprecated after v3.18.0-0.1.pre.',
    )
    Color? background = Colors.white,
    @Deprecated(
      'Use onSurface instead. '
      'This feature was deprecated after v3.18.0-0.1.pre.',
    )
    Color? onBackground = Colors.black,
    @Deprecated(
      'Use surfaceContainerHighest instead. '
      'This feature was deprecated after v3.18.0-0.1.pre.',
    )
    Color? surfaceVariant,
  }) : _primaryContainer = primaryContainer,
       _onPrimaryContainer = onPrimaryContainer,
       _primaryFixed = primaryFixed,
       _primaryFixedDim = primaryFixedDim,
       _onPrimaryFixed = onPrimaryFixed,
       _onPrimaryFixedVariant = onPrimaryFixedVariant,
       _secondaryContainer = secondaryContainer,
       _onSecondaryContainer = onSecondaryContainer,
       _secondaryFixed = secondaryFixed,
       _secondaryFixedDim = secondaryFixedDim,
       _onSecondaryFixed = onSecondaryFixed,
       _onSecondaryFixedVariant = onSecondaryFixedVariant,
       _tertiary = tertiary,
       _onTertiary = onTertiary,
       _tertiaryContainer = tertiaryContainer,
       _onTertiaryContainer = onTertiaryContainer,
       _tertiaryFixed = tertiaryFixed,
       _tertiaryFixedDim = tertiaryFixedDim,
       _onTertiaryFixed = onTertiaryFixed,
       _onTertiaryFixedVariant = onTertiaryFixedVariant,
       _errorContainer = errorContainer,
       _onErrorContainer = onErrorContainer,
       _surfaceDim = surfaceDim,
       _surfaceBright = surfaceBright,
       _surfaceContainerLowest = surfaceContainerLowest,
       _surfaceContainerLow = surfaceContainerLow,
       _surfaceContainer = surfaceContainer,
       _surfaceContainerHigh = surfaceContainerHigh,
       _surfaceContainerHighest = surfaceContainerHighest,
       _onSurfaceVariant = onSurfaceVariant,
       _outline = outline,
       _outlineVariant = outlineVariant,
       _shadow = shadow,
       _scrim = scrim,
       _inverseSurface = inverseSurface,
       _onInverseSurface = onInverseSurface,
       _inversePrimary = inversePrimary,
       _surfaceTint = surfaceTint,
       // DEPRECATED (newest deprecations at the bottom)
       _background = background,
       _onBackground = onBackground,
       _surfaceVariant = surfaceVariant;

  /// Create the dark color scheme that matches the
  /// [baseline Material 2 color scheme](https://material.io/design/color/dark-theme.html#ui-application).
  ///
  /// This constructor shouldn't be used to update the Material 3 color scheme.
  ///
  /// For Material 3, use [ColorScheme.fromSeed] to create a color scheme
  /// from a single seed color based on the Material 3 color system.
  /// Override the `brightness` property of [ColorScheme.fromSeed] to create a
  /// dark color scheme.
  ///
  /// {@tool snippet}
  /// This example demonstrates how to create a color scheme similar to [ColorScheme.dark]
  /// using the [ColorScheme.fromSeed] constructor:
  ///
  /// ```dart
  /// colorScheme: ColorScheme.fromSeed(
  ///   seedColor: const Color(0xffbb86fc),
  ///   brightness: Brightness.dark,
  /// ).copyWith(
  ///   primaryContainer: const Color(0xffbb86fc),
  ///   onPrimaryContainer: Colors.black,
  ///   secondaryContainer: const Color(0xff03dac6),
  ///   onSecondaryContainer: Colors.black,
  ///   error: const Color(0xffcf6679),
  ///   onError: Colors.black,
  /// ),
  /// ```
  /// {@end-tool}
  const ColorScheme.dark({
    this.brightness = Brightness.dark,
    this.primary = const Color(0xffbb86fc),
    this.onPrimary = Colors.black,
    Color? primaryContainer,
    Color? onPrimaryContainer,
    Color? primaryFixed,
    Color? primaryFixedDim,
    Color? onPrimaryFixed,
    Color? onPrimaryFixedVariant,
    this.secondary = const Color(0xff03dac6),
    this.onSecondary = Colors.black,
    Color? secondaryContainer,
    Color? onSecondaryContainer,
    Color? secondaryFixed,
    Color? secondaryFixedDim,
    Color? onSecondaryFixed,
    Color? onSecondaryFixedVariant,
    Color? tertiary,
    Color? onTertiary,
    Color? tertiaryContainer,
    Color? onTertiaryContainer,
    Color? tertiaryFixed,
    Color? tertiaryFixedDim,
    Color? onTertiaryFixed,
    Color? onTertiaryFixedVariant,
    this.error = const Color(0xffcf6679),
    this.onError = Colors.black,
    Color? errorContainer,
    Color? onErrorContainer,
    this.surface = const Color(0xff121212),
    this.onSurface = Colors.white,
    Color? surfaceDim,
    Color? surfaceBright,
    Color? surfaceContainerLowest,
    Color? surfaceContainerLow,
    Color? surfaceContainer,
    Color? surfaceContainerHigh,
    Color? surfaceContainerHighest,
    Color? onSurfaceVariant,
    Color? outline,
    Color? outlineVariant,
    Color? shadow,
    Color? scrim,
    Color? inverseSurface,
    Color? onInverseSurface,
    Color? inversePrimary,
    Color? surfaceTint,
    @Deprecated(
      'Use surface instead. '
      'This feature was deprecated after v3.18.0-0.1.pre.',
    )
    Color? background = const Color(0xff121212),
    @Deprecated(
      'Use onSurface instead. '
      'This feature was deprecated after v3.18.0-0.1.pre.',
    )
    Color? onBackground = Colors.white,
    @Deprecated(
      'Use surfaceContainerHighest instead. '
      'This feature was deprecated after v3.18.0-0.1.pre.',
    )
    Color? surfaceVariant,
  }) : _primaryContainer = primaryContainer,
       _onPrimaryContainer = onPrimaryContainer,
       _primaryFixed = primaryFixed,
       _primaryFixedDim = primaryFixedDim,
       _onPrimaryFixed = onPrimaryFixed,
       _onPrimaryFixedVariant = onPrimaryFixedVariant,
       _secondaryContainer = secondaryContainer,
       _onSecondaryContainer = onSecondaryContainer,
       _secondaryFixed = secondaryFixed,
       _secondaryFixedDim = secondaryFixedDim,
       _onSecondaryFixed = onSecondaryFixed,
       _onSecondaryFixedVariant = onSecondaryFixedVariant,
       _tertiary = tertiary,
       _onTertiary = onTertiary,
       _tertiaryContainer = tertiaryContainer,
       _onTertiaryContainer = onTertiaryContainer,
       _tertiaryFixed = tertiaryFixed,
       _tertiaryFixedDim = tertiaryFixedDim,
       _onTertiaryFixed = onTertiaryFixed,
       _onTertiaryFixedVariant = onTertiaryFixedVariant,
       _errorContainer = errorContainer,
       _onErrorContainer = onErrorContainer,
       _surfaceDim = surfaceDim,
       _surfaceBright = surfaceBright,
       _surfaceContainerLowest = surfaceContainerLowest,
       _surfaceContainerLow = surfaceContainerLow,
       _surfaceContainer = surfaceContainer,
       _surfaceContainerHigh = surfaceContainerHigh,
       _surfaceContainerHighest = surfaceContainerHighest,
       _onSurfaceVariant = onSurfaceVariant,
       _outline = outline,
       _outlineVariant = outlineVariant,
       _shadow = shadow,
       _scrim = scrim,
       _inverseSurface = inverseSurface,
       _onInverseSurface = onInverseSurface,
       _inversePrimary = inversePrimary,
       _surfaceTint = surfaceTint,
       // DEPRECATED (newest deprecations at the bottom)
       _background = background,
       _onBackground = onBackground,
       _surfaceVariant = surfaceVariant;

  /// Create a high contrast ColorScheme based on a purple primary color that
  /// matches the [baseline Material 2 color scheme](https://material.io/design/color/the-color-system.html#color-theme-creation).
  ///
  /// This constructor shouldn't be used to update the Material 3 color scheme.
  ///
  /// For Material 3, use [ColorScheme.fromSeed] to create a color scheme
  /// from a single seed color based on the Material 3 color system. To create a
  /// high-contrast color scheme, set `contrastLevel` to 1.0.
  ///
  /// {@tool snippet}
  /// This example demonstrates how to create a color scheme similar to [ColorScheme.highContrastLight]
  /// using the [ColorScheme.fromSeed] constructor:
  ///
  /// ```dart
  /// colorScheme: ColorScheme.fromSeed(seedColor: const Color(0xff0000ba)).copyWith(
  ///   primaryContainer: const Color(0xff0000ba),
  ///   onPrimaryContainer: Colors.white,
  ///   secondaryContainer: const Color(0xff66fff9),
  ///   onSecondaryContainer: Colors.black,
  ///   error: const Color(0xff790000),
  ///   onError: Colors.white,
  /// ),
  /// ```
  /// {@end-tool}
  const ColorScheme.highContrastLight({
    this.brightness = Brightness.light,
    this.primary = const Color(0xff0000ba),
    this.onPrimary = Colors.white,
    Color? primaryContainer,
    Color? onPrimaryContainer,
    Color? primaryFixed,
    Color? primaryFixedDim,
    Color? onPrimaryFixed,
    Color? onPrimaryFixedVariant,
    this.secondary = const Color(0xff66fff9),
    this.onSecondary = Colors.black,
    Color? secondaryContainer,
    Color? onSecondaryContainer,
    Color? secondaryFixed,
    Color? secondaryFixedDim,
    Color? onSecondaryFixed,
    Color? onSecondaryFixedVariant,
    Color? tertiary,
    Color? onTertiary,
    Color? tertiaryContainer,
    Color? onTertiaryContainer,
    Color? tertiaryFixed,
    Color? tertiaryFixedDim,
    Color? onTertiaryFixed,
    Color? onTertiaryFixedVariant,
    this.error = const Color(0xff790000),
    this.onError = Colors.white,
    Color? errorContainer,
    Color? onErrorContainer,
    this.surface = Colors.white,
    this.onSurface = Colors.black,
    Color? surfaceDim,
    Color? surfaceBright,
    Color? surfaceContainerLowest,
    Color? surfaceContainerLow,
    Color? surfaceContainer,
    Color? surfaceContainerHigh,
    Color? surfaceContainerHighest,
    Color? onSurfaceVariant,
    Color? outline,
    Color? outlineVariant,
    Color? shadow,
    Color? scrim,
    Color? inverseSurface,
    Color? onInverseSurface,
    Color? inversePrimary,
    Color? surfaceTint,
    @Deprecated(
      'Use surface instead. '
      'This feature was deprecated after v3.18.0-0.1.pre.',
    )
    Color? background = Colors.white,
    @Deprecated(
      'Use onSurface instead. '
      'This feature was deprecated after v3.18.0-0.1.pre.',
    )
    Color? onBackground = Colors.black,
    @Deprecated(
      'Use surfaceContainerHighest instead. '
      'This feature was deprecated after v3.18.0-0.1.pre.',
    )
    Color? surfaceVariant,
  }) : _primaryContainer = primaryContainer,
       _onPrimaryContainer = onPrimaryContainer,
       _primaryFixed = primaryFixed,
       _primaryFixedDim = primaryFixedDim,
       _onPrimaryFixed = onPrimaryFixed,
       _onPrimaryFixedVariant = onPrimaryFixedVariant,
       _secondaryContainer = secondaryContainer,
       _onSecondaryContainer = onSecondaryContainer,
       _secondaryFixed = secondaryFixed,
       _secondaryFixedDim = secondaryFixedDim,
       _onSecondaryFixed = onSecondaryFixed,
       _onSecondaryFixedVariant = onSecondaryFixedVariant,
       _tertiary = tertiary,
       _onTertiary = onTertiary,
       _tertiaryContainer = tertiaryContainer,
       _onTertiaryContainer = onTertiaryContainer,
       _tertiaryFixed = tertiaryFixed,
       _tertiaryFixedDim = tertiaryFixedDim,
       _onTertiaryFixed = onTertiaryFixed,
       _onTertiaryFixedVariant = onTertiaryFixedVariant,
       _errorContainer = errorContainer,
       _onErrorContainer = onErrorContainer,
       _surfaceDim = surfaceDim,
       _surfaceBright = surfaceBright,
       _surfaceContainerLowest = surfaceContainerLowest,
       _surfaceContainerLow = surfaceContainerLow,
       _surfaceContainer = surfaceContainer,
       _surfaceContainerHigh = surfaceContainerHigh,
       _surfaceContainerHighest = surfaceContainerHighest,
       _onSurfaceVariant = onSurfaceVariant,
       _outline = outline,
       _outlineVariant = outlineVariant,
       _shadow = shadow,
       _scrim = scrim,
       _inverseSurface = inverseSurface,
       _onInverseSurface = onInverseSurface,
       _inversePrimary = inversePrimary,
       _surfaceTint = surfaceTint,
       // DEPRECATED (newest deprecations at the bottom)
       _background = background,
       _onBackground = onBackground,
       _surfaceVariant = surfaceVariant;

  /// Create a high contrast ColorScheme based on the dark
  /// [baseline Material 2 color scheme](https://material.io/design/color/dark-theme.html#ui-application).
  ///
  /// This constructor shouldn't be used to update the Material 3 color scheme.
  ///
  /// For Material 3, use [ColorScheme.fromSeed] to create a color scheme
  /// from a single seed color based on the Material 3 color system.
  /// Override the `brightness` property of [ColorScheme.fromSeed] to create a
  /// dark color scheme. To create a high-contrast color scheme, set
  /// `contrastLevel` to 1.0.
  ///
  /// {@tool snippet}
  /// This example demonstrates how to create a color scheme similar to [ColorScheme.highContrastDark]
  /// using the [ColorScheme.fromSeed] constructor:
  ///
  /// ```dart
  /// colorScheme: ColorScheme.fromSeed(
  ///   seedColor: const Color(0xffefb7ff),
  ///   brightness: Brightness.dark,
  /// ).copyWith(
  ///   primaryContainer: const Color(0xffefb7ff),
  ///   onPrimaryContainer: Colors.black,
  ///   secondaryContainer: const Color(0xff66fff9),
  ///   onSecondaryContainer: Colors.black,
  ///   error: const Color(0xff9b374d),
  ///   onError: Colors.white,
  /// ),
  /// ```
  /// {@end-tool}
  const ColorScheme.highContrastDark({
    this.brightness = Brightness.dark,
    this.primary = const Color(0xffefb7ff),
    this.onPrimary = Colors.black,
    Color? primaryContainer,
    Color? onPrimaryContainer,
    Color? primaryFixed,
    Color? primaryFixedDim,
    Color? onPrimaryFixed,
    Color? onPrimaryFixedVariant,
    this.secondary = const Color(0xff66fff9),
    this.onSecondary = Colors.black,
    Color? secondaryContainer,
    Color? onSecondaryContainer,
    Color? secondaryFixed,
    Color? secondaryFixedDim,
    Color? onSecondaryFixed,
    Color? onSecondaryFixedVariant,
    Color? tertiary,
    Color? onTertiary,
    Color? tertiaryContainer,
    Color? onTertiaryContainer,
    Color? tertiaryFixed,
    Color? tertiaryFixedDim,
    Color? onTertiaryFixed,
    Color? onTertiaryFixedVariant,
    this.error = const Color(0xff9b374d),
    this.onError = Colors.black,
    Color? errorContainer,
    Color? onErrorContainer,
    this.surface = const Color(0xff121212),
    this.onSurface = Colors.white,
    Color? surfaceDim,
    Color? surfaceBright,
    Color? surfaceContainerLowest,
    Color? surfaceContainerLow,
    Color? surfaceContainer,
    Color? surfaceContainerHigh,
    Color? surfaceContainerHighest,
    Color? onSurfaceVariant,
    Color? outline,
    Color? outlineVariant,
    Color? shadow,
    Color? scrim,
    Color? inverseSurface,
    Color? onInverseSurface,
    Color? inversePrimary,
    Color? surfaceTint,
    @Deprecated(
      'Use surface instead. '
      'This feature was deprecated after v3.18.0-0.1.pre.',
    )
    Color? background = const Color(0xff121212),
    @Deprecated(
      'Use onSurface instead. '
      'This feature was deprecated after v3.18.0-0.1.pre.',
    )
    Color? onBackground = Colors.white,
    @Deprecated(
      'Use surfaceContainerHighest instead. '
      'This feature was deprecated after v3.18.0-0.1.pre.',
    )
    Color? surfaceVariant,
  }) : _primaryContainer = primaryContainer,
       _onPrimaryContainer = onPrimaryContainer,
       _primaryFixed = primaryFixed,
       _primaryFixedDim = primaryFixedDim,
       _onPrimaryFixed = onPrimaryFixed,
       _onPrimaryFixedVariant = onPrimaryFixedVariant,
       _secondaryContainer = secondaryContainer,
       _onSecondaryContainer = onSecondaryContainer,
       _secondaryFixed = secondaryFixed,
       _secondaryFixedDim = secondaryFixedDim,
       _onSecondaryFixed = onSecondaryFixed,
       _onSecondaryFixedVariant = onSecondaryFixedVariant,
       _tertiary = tertiary,
       _onTertiary = onTertiary,
       _tertiaryContainer = tertiaryContainer,
       _onTertiaryContainer = onTertiaryContainer,
       _tertiaryFixed = tertiaryFixed,
       _tertiaryFixedDim = tertiaryFixedDim,
       _onTertiaryFixed = onTertiaryFixed,
       _onTertiaryFixedVariant = onTertiaryFixedVariant,
       _errorContainer = errorContainer,
       _onErrorContainer = onErrorContainer,
       _surfaceDim = surfaceDim,
       _surfaceBright = surfaceBright,
       _surfaceContainerLowest = surfaceContainerLowest,
       _surfaceContainerLow = surfaceContainerLow,
       _surfaceContainer = surfaceContainer,
       _surfaceContainerHigh = surfaceContainerHigh,
       _surfaceContainerHighest = surfaceContainerHighest,
       _onSurfaceVariant = onSurfaceVariant,
       _outline = outline,
       _outlineVariant = outlineVariant,
       _shadow = shadow,
       _scrim = scrim,
       _inverseSurface = inverseSurface,
       _onInverseSurface = onInverseSurface,
       _inversePrimary = inversePrimary,
       _surfaceTint = surfaceTint,
       // DEPRECATED (newest deprecations at the bottom)
       _background = background,
       _onBackground = onBackground,
       _surfaceVariant = surfaceVariant;

  /// Creates a color scheme from a [MaterialColor] swatch.
  ///
  /// In Material 3, this constructor is ignored by [ThemeData] when creating
  /// its default color scheme. Instead, [ThemeData] uses [ColorScheme.fromSeed]
  /// to create its default color scheme. This constructor shouldn't be used
  /// to update the Material 3 color scheme. It will be phased out gradually;
  /// see https://github.com/flutter/flutter/issues/120064 for more details.
  ///
  /// If [ThemeData.useMaterial3] is false, then this constructor is used by
  /// [ThemeData] to create its default color scheme.
  factory ColorScheme.fromSwatch({
    MaterialColor primarySwatch = Colors.blue,
    Color? accentColor,
    Color? cardColor,
    Color? backgroundColor,
    Color? errorColor,
    Brightness brightness = Brightness.light,
  }) {
    final bool isDark = brightness == Brightness.dark;
    final bool primaryIsDark = _brightnessFor(primarySwatch) == Brightness.dark;
    final Color secondary = accentColor ?? (isDark ? Colors.tealAccent[200]! : primarySwatch);
    final bool secondaryIsDark = _brightnessFor(secondary) == Brightness.dark;

    return ColorScheme(
      primary: primarySwatch,
      secondary: secondary,
      surface: cardColor ?? (isDark ? Colors.grey[800]! : Colors.white),
      error: errorColor ?? Colors.red[700]!,
      onPrimary: primaryIsDark ? Colors.white : Colors.black,
      onSecondary: secondaryIsDark ? Colors.white : Colors.black,
      onSurface: isDark ? Colors.white : Colors.black,
      onError: isDark ? Colors.black : Colors.white,
      brightness: brightness,
      // DEPRECATED (newest deprecations at the bottom)
      background: backgroundColor ?? (isDark ? Colors.grey[700]! : primarySwatch[200]!),
      onBackground: primaryIsDark ? Colors.white : Colors.black,
    );
  }

  static Brightness _brightnessFor(Color color) => ThemeData.estimateBrightnessForColor(color);

  /// The overall brightness of this color scheme.
  final Brightness brightness;

  /// The color displayed most frequently across your app’s screens and components.
  final Color primary;

  /// A color that's clearly legible when drawn on [primary].
  ///
  /// To ensure that an app is accessible, a contrast ratio between
  /// [primary] and [onPrimary] of at least 4.5:1 is recommended. See
  /// <https://www.w3.org/TR/UNDERSTANDING-WCAG20/visual-audio-contrast-contrast.html>.
  final Color onPrimary;

  final Color? _primaryContainer;

  /// A color used for elements needing less emphasis than [primary].
  Color get primaryContainer => _primaryContainer ?? primary;

  final Color? _onPrimaryContainer;

  /// A color that's clearly legible when drawn on [primaryContainer].
  ///
  /// To ensure that an app is accessible, a contrast ratio between
  /// [primaryContainer] and [onPrimaryContainer] of at least 4.5:1
  /// is recommended. See
  /// <https://www.w3.org/TR/UNDERSTANDING-WCAG20/visual-audio-contrast-contrast.html>.
  Color get onPrimaryContainer => _onPrimaryContainer ?? onPrimary;

  final Color? _primaryFixed;

  /// A substitute for [primaryContainer] that's the same color for the dark
  /// and light themes.
  Color get primaryFixed => _primaryFixed ?? primary;

  final Color? _primaryFixedDim;

  /// A color used for elements needing more emphasis than [primaryFixed].
  Color get primaryFixedDim => _primaryFixedDim ?? primary;

  final Color? _onPrimaryFixed;

  /// A color that is used for text and icons that exist on top of elements having
  /// [primaryFixed] color.
  Color get onPrimaryFixed => _onPrimaryFixed ?? onPrimary;

  final Color? _onPrimaryFixedVariant;

  /// A color that provides a lower-emphasis option for text and icons than
  /// [onPrimaryFixed].
  Color get onPrimaryFixedVariant => _onPrimaryFixedVariant ?? onPrimary;

  /// An accent color used for less prominent components in the UI, such as
  /// filter chips, while expanding the opportunity for color expression.
  final Color secondary;

  /// A color that's clearly legible when drawn on [secondary].
  ///
  /// To ensure that an app is accessible, a contrast ratio between
  /// [secondary] and [onSecondary] of at least 4.5:1 is recommended. See
  /// <https://www.w3.org/TR/UNDERSTANDING-WCAG20/visual-audio-contrast-contrast.html>.
  final Color onSecondary;

  final Color? _secondaryContainer;

  /// A color used for elements needing less emphasis than [secondary].
  Color get secondaryContainer => _secondaryContainer ?? secondary;

  final Color? _onSecondaryContainer;

  /// A color that's clearly legible when drawn on [secondaryContainer].
  ///
  /// To ensure that an app is accessible, a contrast ratio between
  /// [secondaryContainer] and [onSecondaryContainer] of at least 4.5:1 is
  /// recommended. See
  /// <https://www.w3.org/TR/UNDERSTANDING-WCAG20/visual-audio-contrast-contrast.html>.
  Color get onSecondaryContainer => _onSecondaryContainer ?? onSecondary;

  final Color? _secondaryFixed;

  /// A substitute for [secondaryContainer] that's the same color for the dark
  /// and light themes.
  Color get secondaryFixed => _secondaryFixed ?? secondary;

  final Color? _secondaryFixedDim;

  /// A color used for elements needing more emphasis than [secondaryFixed].
  Color get secondaryFixedDim => _secondaryFixedDim ?? secondary;

  final Color? _onSecondaryFixed;

  /// A color that is used for text and icons that exist on top of elements having
  /// [secondaryFixed] color.
  Color get onSecondaryFixed => _onSecondaryFixed ?? onSecondary;

  final Color? _onSecondaryFixedVariant;

  /// A color that provides a lower-emphasis option for text and icons than
  /// [onSecondaryFixed].
  Color get onSecondaryFixedVariant => _onSecondaryFixedVariant ?? onSecondary;

  final Color? _tertiary;

  /// A color used as a contrasting accent that can balance [primary]
  /// and [secondary] colors or bring heightened attention to an element,
  /// such as an input field.
  Color get tertiary => _tertiary ?? secondary;

  final Color? _onTertiary;

  /// A color that's clearly legible when drawn on [tertiary].
  ///
  /// To ensure that an app is accessible, a contrast ratio between
  /// [tertiary] and [onTertiary] of at least 4.5:1 is recommended. See
  /// <https://www.w3.org/TR/UNDERSTANDING-WCAG20/visual-audio-contrast-contrast.html>.
  Color get onTertiary => _onTertiary ?? onSecondary;

  final Color? _tertiaryContainer;

  /// A color used for elements needing less emphasis than [tertiary].
  Color get tertiaryContainer => _tertiaryContainer ?? tertiary;

  final Color? _onTertiaryContainer;

  /// A color that's clearly legible when drawn on [tertiaryContainer].
  ///
  /// To ensure that an app is accessible, a contrast ratio between
  /// [tertiaryContainer] and [onTertiaryContainer] of at least 4.5:1 is
  /// recommended. See
  /// <https://www.w3.org/TR/UNDERSTANDING-WCAG20/visual-audio-contrast-contrast.html>.
  Color get onTertiaryContainer => _onTertiaryContainer ?? onTertiary;

  final Color? _tertiaryFixed;

  /// A substitute for [tertiaryContainer] that's the same color for dark
  /// and light themes.
  Color get tertiaryFixed => _tertiaryFixed ?? tertiary;

  final Color? _tertiaryFixedDim;

  /// A color used for elements needing more emphasis than [tertiaryFixed].
  Color get tertiaryFixedDim => _tertiaryFixedDim ?? tertiary;

  final Color? _onTertiaryFixed;

  /// A color that is used for text and icons that exist on top of elements having
  /// [tertiaryFixed] color.
  Color get onTertiaryFixed => _onTertiaryFixed ?? onTertiary;

  final Color? _onTertiaryFixedVariant;

  /// A color that provides a lower-emphasis option for text and icons than
  /// [onTertiaryFixed].
  Color get onTertiaryFixedVariant => _onTertiaryFixedVariant ?? onTertiary;

  /// The color to use for input validation errors, e.g. for
  /// [InputDecoration.errorText].
  final Color error;

  /// A color that's clearly legible when drawn on [error].
  ///
  /// To ensure that an app is accessible, a contrast ratio between
  /// [error] and [onError] of at least 4.5:1 is recommended. See
  /// <https://www.w3.org/TR/UNDERSTANDING-WCAG20/visual-audio-contrast-contrast.html>.
  final Color onError;

  final Color? _errorContainer;

  /// A color used for error elements needing less emphasis than [error].
  Color get errorContainer => _errorContainer ?? error;

  final Color? _onErrorContainer;

  /// A color that's clearly legible when drawn on [errorContainer].
  ///
  /// To ensure that an app is accessible, a contrast ratio between
  /// [errorContainer] and [onErrorContainer] of at least 4.5:1 is
  /// recommended. See
  /// <https://www.w3.org/TR/UNDERSTANDING-WCAG20/visual-audio-contrast-contrast.html>.
  Color get onErrorContainer => _onErrorContainer ?? onError;

  /// The background color for widgets like [Scaffold].
  final Color surface;

  /// A color that's clearly legible when drawn on [surface].
  ///
  /// To ensure that an app is accessible, a contrast ratio between
  /// [surface] and [onSurface] of at least 4.5:1 is recommended. See
  /// <https://www.w3.org/TR/UNDERSTANDING-WCAG20/visual-audio-contrast-contrast.html>.
  final Color onSurface;

  final Color? _surfaceVariant;

  /// A color variant of [surface] that can be used for differentiation against
  /// a component using [surface].
  @Deprecated(
    'Use surfaceContainerHighest instead. '
    'This feature was deprecated after v3.18.0-0.1.pre.',
  )
  Color get surfaceVariant => _surfaceVariant ?? surface;

  final Color? _surfaceDim;

  /// A color that's always darkest in the dark or light theme.
  Color get surfaceDim => _surfaceDim ?? surface;

  final Color? _surfaceBright;

  /// A color that's always the lightest in the dark or light theme.
  Color get surfaceBright => _surfaceBright ?? surface;

  final Color? _surfaceContainerLowest;

  /// A surface container color with the lightest tone and the least emphasis
  /// relative to the surface.
  Color get surfaceContainerLowest => _surfaceContainerLowest ?? surface;

  final Color? _surfaceContainerLow;

  /// A surface container color with a lighter tone that creates less emphasis
  /// than [surfaceContainer] but more emphasis than [surfaceContainerLowest].
  Color get surfaceContainerLow => _surfaceContainerLow ?? surface;

  final Color? _surfaceContainer;

  /// A recommended color role for a distinct area within the surface.
  ///
  /// Surface container color roles are independent of elevation. They replace the old
  /// opacity-based model which applied a tinted overlay on top of
  /// surfaces based on their elevation.
  ///
  /// Surface container colors include [surfaceContainerLowest], [surfaceContainerLow],
  /// [surfaceContainer], [surfaceContainerHigh] and [surfaceContainerHighest].
  Color get surfaceContainer => _surfaceContainer ?? surface;

  final Color? _surfaceContainerHigh;

  /// A surface container color with a darker tone. It is used to create more
  /// emphasis than [surfaceContainer] but less emphasis than [surfaceContainerHighest].
  Color get surfaceContainerHigh => _surfaceContainerHigh ?? surface;

  final Color? _surfaceContainerHighest;

  /// A surface container color with the darkest tone. It is used to create the
  /// most emphasis against the surface.
  Color get surfaceContainerHighest => _surfaceContainerHighest ?? surface;

  final Color? _onSurfaceVariant;

  /// A color that's clearly legible when drawn on [surfaceVariant].
  ///
  /// To ensure that an app is accessible, a contrast ratio between
  /// [surfaceVariant] and [onSurfaceVariant] of at least 4.5:1 is
  /// recommended. See
  /// <https://www.w3.org/TR/UNDERSTANDING-WCAG20/visual-audio-contrast-contrast.html>.
  Color get onSurfaceVariant => _onSurfaceVariant ?? onSurface;

  final Color? _outline;

  /// A utility color that creates boundaries and emphasis to improve usability.
  Color get outline => _outline ?? onBackground;

  final Color? _outlineVariant;

  /// A utility color that creates boundaries for decorative elements when a
  /// 3:1 contrast isn’t required, such as for dividers or decorative elements.
  Color get outlineVariant => _outlineVariant ?? onBackground;

  final Color? _shadow;

  /// A color use to paint the drop shadows of elevated components.
  Color get shadow => _shadow ?? const Color(0xff000000);

  final Color? _scrim;

  /// A color use to paint the scrim around of modal components.
  Color get scrim => _scrim ?? const Color(0xff000000);

  final Color? _inverseSurface;

  /// A surface color used for displaying the reverse of what’s seen in the
  /// surrounding UI, for example in a SnackBar to bring attention to
  /// an alert.
  Color get inverseSurface => _inverseSurface ?? onSurface;

  final Color? _onInverseSurface;

  /// A color that's clearly legible when drawn on [inverseSurface].
  ///
  /// To ensure that an app is accessible, a contrast ratio between
  /// [inverseSurface] and [onInverseSurface] of at least 4.5:1 is
  /// recommended. See
  /// <https://www.w3.org/TR/UNDERSTANDING-WCAG20/visual-audio-contrast-contrast.html>.
  Color get onInverseSurface => _onInverseSurface ?? surface;

  final Color? _inversePrimary;

  /// An accent color used for displaying a highlight color on [inverseSurface]
  /// backgrounds, like button text in a SnackBar.
  Color get inversePrimary => _inversePrimary ?? onPrimary;

  final Color? _surfaceTint;

  /// A color used as an overlay on a surface color to indicate a component's
  /// elevation.
  Color get surfaceTint => _surfaceTint ?? primary;

  final Color? _background;

  /// A color that typically appears behind scrollable content.
  @Deprecated(
    'Use surface instead. '
    'This feature was deprecated after v3.18.0-0.1.pre.',
  )
  Color get background => _background ?? surface;

  final Color? _onBackground;

  /// A color that's clearly legible when drawn on [background].
  ///
  /// To ensure that an app is accessible, a contrast ratio between
  /// [background] and [onBackground] of at least 4.5:1 is recommended. See
  /// <https://www.w3.org/TR/UNDERSTANDING-WCAG20/visual-audio-contrast-contrast.html>.
  @Deprecated(
    'Use onSurface instead. '
    'This feature was deprecated after v3.18.0-0.1.pre.',
  )
  Color get onBackground => _onBackground ?? onSurface;

  /// Creates a copy of this color scheme with the given fields
  /// replaced by the non-null parameter values.
  ColorScheme copyWith({
    Brightness? brightness,
    Color? primary,
    Color? onPrimary,
    Color? primaryContainer,
    Color? onPrimaryContainer,
    Color? primaryFixed,
    Color? primaryFixedDim,
    Color? onPrimaryFixed,
    Color? onPrimaryFixedVariant,
    Color? secondary,
    Color? onSecondary,
    Color? secondaryContainer,
    Color? onSecondaryContainer,
    Color? secondaryFixed,
    Color? secondaryFixedDim,
    Color? onSecondaryFixed,
    Color? onSecondaryFixedVariant,
    Color? tertiary,
    Color? onTertiary,
    Color? tertiaryContainer,
    Color? onTertiaryContainer,
    Color? tertiaryFixed,
    Color? tertiaryFixedDim,
    Color? onTertiaryFixed,
    Color? onTertiaryFixedVariant,
    Color? error,
    Color? onError,
    Color? errorContainer,
    Color? onErrorContainer,
    Color? surface,
    Color? onSurface,
    Color? surfaceDim,
    Color? surfaceBright,
    Color? surfaceContainerLowest,
    Color? surfaceContainerLow,
    Color? surfaceContainer,
    Color? surfaceContainerHigh,
    Color? surfaceContainerHighest,
    Color? onSurfaceVariant,
    Color? outline,
    Color? outlineVariant,
    Color? shadow,
    Color? scrim,
    Color? inverseSurface,
    Color? onInverseSurface,
    Color? inversePrimary,
    Color? surfaceTint,
    @Deprecated(
      'Use surface instead. '
      'This feature was deprecated after v3.18.0-0.1.pre.',
    )
    Color? background,
    @Deprecated(
      'Use onSurface instead. '
      'This feature was deprecated after v3.18.0-0.1.pre.',
    )
    Color? onBackground,
    @Deprecated(
      'Use surfaceContainerHighest instead. '
      'This feature was deprecated after v3.18.0-0.1.pre.',
    )
    Color? surfaceVariant,
  }) {
    return ColorScheme(
      brightness: brightness ?? this.brightness,
      primary: primary ?? this.primary,
      onPrimary: onPrimary ?? this.onPrimary,
      primaryContainer: primaryContainer ?? this.primaryContainer,
      onPrimaryContainer: onPrimaryContainer ?? this.onPrimaryContainer,
      primaryFixed: primaryFixed ?? this.primaryFixed,
      primaryFixedDim: primaryFixedDim ?? this.primaryFixedDim,
      onPrimaryFixed: onPrimaryFixed ?? this.onPrimaryFixed,
      onPrimaryFixedVariant: onPrimaryFixedVariant ?? this.onPrimaryFixedVariant,
      secondary: secondary ?? this.secondary,
      onSecondary: onSecondary ?? this.onSecondary,
      secondaryContainer: secondaryContainer ?? this.secondaryContainer,
      onSecondaryContainer: onSecondaryContainer ?? this.onSecondaryContainer,
      secondaryFixed: secondaryFixed ?? this.secondaryFixed,
      secondaryFixedDim: secondaryFixedDim ?? this.secondaryFixedDim,
      onSecondaryFixed: onSecondaryFixed ?? this.onSecondaryFixed,
      onSecondaryFixedVariant: onSecondaryFixedVariant ?? this.onSecondaryFixedVariant,
      tertiary: tertiary ?? this.tertiary,
      onTertiary: onTertiary ?? this.onTertiary,
      tertiaryContainer: tertiaryContainer ?? this.tertiaryContainer,
      onTertiaryContainer: onTertiaryContainer ?? this.onTertiaryContainer,
      tertiaryFixed: tertiaryFixed ?? this.tertiaryFixed,
      tertiaryFixedDim: tertiaryFixedDim ?? this.tertiaryFixedDim,
      onTertiaryFixed: onTertiaryFixed ?? this.onTertiaryFixed,
      onTertiaryFixedVariant: onTertiaryFixedVariant ?? this.onTertiaryFixedVariant,
      error: error ?? this.error,
      onError: onError ?? this.onError,
      errorContainer: errorContainer ?? this.errorContainer,
      onErrorContainer: onErrorContainer ?? this.onErrorContainer,
      surface: surface ?? this.surface,
      onSurface: onSurface ?? this.onSurface,
      surfaceDim: surfaceDim ?? this.surfaceDim,
      surfaceBright: surfaceBright ?? this.surfaceBright,
      surfaceContainerLowest: surfaceContainerLowest ?? this.surfaceContainerLowest,
      surfaceContainerLow: surfaceContainerLow ?? this.surfaceContainerLow,
      surfaceContainer: surfaceContainer ?? this.surfaceContainer,
      surfaceContainerHigh: surfaceContainerHigh ?? this.surfaceContainerHigh,
      surfaceContainerHighest: surfaceContainerHighest ?? this.surfaceContainerHighest,
      onSurfaceVariant: onSurfaceVariant ?? this.onSurfaceVariant,
      outline: outline ?? this.outline,
      outlineVariant: outlineVariant ?? this.outlineVariant,
      shadow: shadow ?? this.shadow,
      scrim: scrim ?? this.scrim,
      inverseSurface: inverseSurface ?? this.inverseSurface,
      onInverseSurface: onInverseSurface ?? this.onInverseSurface,
      inversePrimary: inversePrimary ?? this.inversePrimary,
      surfaceTint: surfaceTint ?? this.surfaceTint,
      // DEPRECATED (newest deprecations at the bottom)
      background: background ?? this.background,
      onBackground: onBackground ?? this.onBackground,
      surfaceVariant: surfaceVariant ?? this.surfaceVariant,
    );
  }

  /// Linearly interpolate between two [ColorScheme] objects.
  ///
  /// {@macro dart.ui.shadow.lerp}
  static ColorScheme lerp(ColorScheme a, ColorScheme b, double t) {
    if (identical(a, b)) {
      return a;
    }
    return ColorScheme(
      brightness: t < 0.5 ? a.brightness : b.brightness,
      primary: Color.lerp(a.primary, b.primary, t)!,
      onPrimary: Color.lerp(a.onPrimary, b.onPrimary, t)!,
      primaryContainer: Color.lerp(a.primaryContainer, b.primaryContainer, t),
      onPrimaryContainer: Color.lerp(a.onPrimaryContainer, b.onPrimaryContainer, t),
      primaryFixed: Color.lerp(a.primaryFixed, b.primaryFixed, t),
      primaryFixedDim: Color.lerp(a.primaryFixedDim, b.primaryFixedDim, t),
      onPrimaryFixed: Color.lerp(a.onPrimaryFixed, b.onPrimaryFixed, t),
      onPrimaryFixedVariant: Color.lerp(a.onPrimaryFixedVariant, b.onPrimaryFixedVariant, t),
      secondary: Color.lerp(a.secondary, b.secondary, t)!,
      onSecondary: Color.lerp(a.onSecondary, b.onSecondary, t)!,
      secondaryContainer: Color.lerp(a.secondaryContainer, b.secondaryContainer, t),
      onSecondaryContainer: Color.lerp(a.onSecondaryContainer, b.onSecondaryContainer, t),
      secondaryFixed: Color.lerp(a.secondaryFixed, b.secondaryFixed, t),
      secondaryFixedDim: Color.lerp(a.secondaryFixedDim, b.secondaryFixedDim, t),
      onSecondaryFixed: Color.lerp(a.onSecondaryFixed, b.onSecondaryFixed, t),
      onSecondaryFixedVariant: Color.lerp(a.onSecondaryFixedVariant, b.onSecondaryFixedVariant, t),
      tertiary: Color.lerp(a.tertiary, b.tertiary, t),
      onTertiary: Color.lerp(a.onTertiary, b.onTertiary, t),
      tertiaryContainer: Color.lerp(a.tertiaryContainer, b.tertiaryContainer, t),
      onTertiaryContainer: Color.lerp(a.onTertiaryContainer, b.onTertiaryContainer, t),
      tertiaryFixed: Color.lerp(a.tertiaryFixed, b.tertiaryFixed, t),
      tertiaryFixedDim: Color.lerp(a.tertiaryFixedDim, b.tertiaryFixedDim, t),
      onTertiaryFixed: Color.lerp(a.onTertiaryFixed, b.onTertiaryFixed, t),
      onTertiaryFixedVariant: Color.lerp(a.onTertiaryFixedVariant, b.onTertiaryFixedVariant, t),
      error: Color.lerp(a.error, b.error, t)!,
      onError: Color.lerp(a.onError, b.onError, t)!,
      errorContainer: Color.lerp(a.errorContainer, b.errorContainer, t),
      onErrorContainer: Color.lerp(a.onErrorContainer, b.onErrorContainer, t),
      surface: Color.lerp(a.surface, b.surface, t)!,
      onSurface: Color.lerp(a.onSurface, b.onSurface, t)!,
      surfaceDim: Color.lerp(a.surfaceDim, b.surfaceDim, t),
      surfaceBright: Color.lerp(a.surfaceBright, b.surfaceBright, t),
      surfaceContainerLowest: Color.lerp(a.surfaceContainerLowest, b.surfaceContainerLowest, t),
      surfaceContainerLow: Color.lerp(a.surfaceContainerLow, b.surfaceContainerLow, t),
      surfaceContainer: Color.lerp(a.surfaceContainer, b.surfaceContainer, t),
      surfaceContainerHigh: Color.lerp(a.surfaceContainerHigh, b.surfaceContainerHigh, t),
      surfaceContainerHighest: Color.lerp(a.surfaceContainerHighest, b.surfaceContainerHighest, t),
      onSurfaceVariant: Color.lerp(a.onSurfaceVariant, b.onSurfaceVariant, t),
      outline: Color.lerp(a.outline, b.outline, t),
      outlineVariant: Color.lerp(a.outlineVariant, b.outlineVariant, t),
      shadow: Color.lerp(a.shadow, b.shadow, t),
      scrim: Color.lerp(a.scrim, b.scrim, t),
      inverseSurface: Color.lerp(a.inverseSurface, b.inverseSurface, t),
      onInverseSurface: Color.lerp(a.onInverseSurface, b.onInverseSurface, t),
      inversePrimary: Color.lerp(a.inversePrimary, b.inversePrimary, t),
      surfaceTint: Color.lerp(a.surfaceTint, b.surfaceTint, t),
      // DEPRECATED (newest deprecations at the bottom)
      background: Color.lerp(a.background, b.background, t),
      onBackground: Color.lerp(a.onBackground, b.onBackground, t),
      surfaceVariant: Color.lerp(a.surfaceVariant, b.surfaceVariant, t),
    );
  }

  @override
  bool operator ==(Object other) {
    if (identical(this, other)) {
      return true;
    }
    if (other.runtimeType != runtimeType) {
      return false;
    }
    return other is ColorScheme &&
        other.brightness == brightness &&
        other.primary == primary &&
        other.onPrimary == onPrimary &&
        other.primaryContainer == primaryContainer &&
        other.onPrimaryContainer == onPrimaryContainer &&
        other.primaryFixed == primaryFixed &&
        other.primaryFixedDim == primaryFixedDim &&
        other.onPrimaryFixed == onPrimaryFixed &&
        other.onPrimaryFixedVariant == onPrimaryFixedVariant &&
        other.secondary == secondary &&
        other.onSecondary == onSecondary &&
        other.secondaryContainer == secondaryContainer &&
        other.onSecondaryContainer == onSecondaryContainer &&
        other.secondaryFixed == secondaryFixed &&
        other.secondaryFixedDim == secondaryFixedDim &&
        other.onSecondaryFixed == onSecondaryFixed &&
        other.onSecondaryFixedVariant == onSecondaryFixedVariant &&
        other.tertiary == tertiary &&
        other.onTertiary == onTertiary &&
        other.tertiaryContainer == tertiaryContainer &&
        other.onTertiaryContainer == onTertiaryContainer &&
        other.tertiaryFixed == tertiaryFixed &&
        other.tertiaryFixedDim == tertiaryFixedDim &&
        other.onTertiaryFixed == onTertiaryFixed &&
        other.onTertiaryFixedVariant == onTertiaryFixedVariant &&
        other.error == error &&
        other.onError == onError &&
        other.errorContainer == errorContainer &&
        other.onErrorContainer == onErrorContainer &&
        other.surface == surface &&
        other.onSurface == onSurface &&
        other.surfaceDim == surfaceDim &&
        other.surfaceBright == surfaceBright &&
        other.surfaceContainerLowest == surfaceContainerLowest &&
        other.surfaceContainerLow == surfaceContainerLow &&
        other.surfaceContainer == surfaceContainer &&
        other.surfaceContainerHigh == surfaceContainerHigh &&
        other.surfaceContainerHighest == surfaceContainerHighest &&
        other.onSurfaceVariant == onSurfaceVariant &&
        other.outline == outline &&
        other.outlineVariant == outlineVariant &&
        other.shadow == shadow &&
        other.scrim == scrim &&
        other.inverseSurface == inverseSurface &&
        other.onInverseSurface == onInverseSurface &&
        other.inversePrimary == inversePrimary &&
        other.surfaceTint == surfaceTint
        // DEPRECATED (newest deprecations at the bottom)
        &&
        other.background == background &&
        other.onBackground == onBackground &&
        other.surfaceVariant == surfaceVariant;
  }

  @override
  int get hashCode => Object.hash(
    brightness,
    primary,
    onPrimary,
    primaryContainer,
    onPrimaryContainer,
    secondary,
    onSecondary,
    secondaryContainer,
    onSecondaryContainer,
    tertiary,
    onTertiary,
    tertiaryContainer,
    onTertiaryContainer,
    error,
    onError,
    errorContainer,
    onErrorContainer,
    Object.hash(
      surface,
      onSurface,
      surfaceDim,
      surfaceBright,
      surfaceContainerLowest,
      surfaceContainerLow,
      surfaceContainer,
      surfaceContainerHigh,
      surfaceContainerHighest,
      onSurfaceVariant,
      outline,
      outlineVariant,
      shadow,
      scrim,
      inverseSurface,
      onInverseSurface,
      inversePrimary,
      surfaceTint,
      Object.hash(
        primaryFixed,
        primaryFixedDim,
        onPrimaryFixed,
        onPrimaryFixedVariant,
        secondaryFixed,
        secondaryFixedDim,
        onSecondaryFixed,
        onSecondaryFixedVariant,
        tertiaryFixed,
        tertiaryFixedDim,
        onTertiaryFixed,
        onTertiaryFixedVariant,
        // DEPRECATED (newest deprecations at the bottom)
        background,
        onBackground,
        surfaceVariant,
      ),
    ),
  );

  @override
  void debugFillProperties(DiagnosticPropertiesBuilder properties) {
    super.debugFillProperties(properties);
    const ColorScheme defaultScheme = ColorScheme.light();
    properties.add(
      DiagnosticsProperty<Brightness>(
        'brightness',
        brightness,
        defaultValue: defaultScheme.brightness,
      ),
    );
    properties.add(ColorProperty('primary', primary, defaultValue: defaultScheme.primary));
    properties.add(ColorProperty('onPrimary', onPrimary, defaultValue: defaultScheme.onPrimary));
    properties.add(
      ColorProperty(
        'primaryContainer',
        primaryContainer,
        defaultValue: defaultScheme.primaryContainer,
      ),
    );
    properties.add(
      ColorProperty(
        'onPrimaryContainer',
        onPrimaryContainer,
        defaultValue: defaultScheme.onPrimaryContainer,
      ),
    );
    properties.add(
      ColorProperty('primaryFixed', primaryFixed, defaultValue: defaultScheme.primaryFixed),
    );
    properties.add(
      ColorProperty(
        'primaryFixedDim',
        primaryFixedDim,
        defaultValue: defaultScheme.primaryFixedDim,
      ),
    );
    properties.add(
      ColorProperty('onPrimaryFixed', onPrimaryFixed, defaultValue: defaultScheme.onPrimaryFixed),
    );
    properties.add(
      ColorProperty(
        'onPrimaryFixedVariant',
        onPrimaryFixedVariant,
        defaultValue: defaultScheme.onPrimaryFixedVariant,
      ),
    );
    properties.add(ColorProperty('secondary', secondary, defaultValue: defaultScheme.secondary));
    properties.add(
      ColorProperty('onSecondary', onSecondary, defaultValue: defaultScheme.onSecondary),
    );
    properties.add(
      ColorProperty(
        'secondaryContainer',
        secondaryContainer,
        defaultValue: defaultScheme.secondaryContainer,
      ),
    );
    properties.add(
      ColorProperty(
        'onSecondaryContainer',
        onSecondaryContainer,
        defaultValue: defaultScheme.onSecondaryContainer,
      ),
    );
    properties.add(
      ColorProperty('secondaryFixed', secondaryFixed, defaultValue: defaultScheme.secondaryFixed),
    );
    properties.add(
      ColorProperty(
        'secondaryFixedDim',
        secondaryFixedDim,
        defaultValue: defaultScheme.secondaryFixedDim,
      ),
    );
    properties.add(
      ColorProperty(
        'onSecondaryFixed',
        onSecondaryFixed,
        defaultValue: defaultScheme.onSecondaryFixed,
      ),
    );
    properties.add(
      ColorProperty(
        'onSecondaryFixedVariant',
        onSecondaryFixedVariant,
        defaultValue: defaultScheme.onSecondaryFixedVariant,
      ),
    );
    properties.add(ColorProperty('tertiary', tertiary, defaultValue: defaultScheme.tertiary));
    properties.add(ColorProperty('onTertiary', onTertiary, defaultValue: defaultScheme.onTertiary));
    properties.add(
      ColorProperty(
        'tertiaryContainer',
        tertiaryContainer,
        defaultValue: defaultScheme.tertiaryContainer,
      ),
    );
    properties.add(
      ColorProperty(
        'onTertiaryContainer',
        onTertiaryContainer,
        defaultValue: defaultScheme.onTertiaryContainer,
      ),
    );
    properties.add(
      ColorProperty('tertiaryFixed', tertiaryFixed, defaultValue: defaultScheme.tertiaryFixed),
    );
    properties.add(
      ColorProperty(
        'tertiaryFixedDim',
        tertiaryFixedDim,
        defaultValue: defaultScheme.tertiaryFixedDim,
      ),
    );
    properties.add(
      ColorProperty(
        'onTertiaryFixed',
        onTertiaryFixed,
        defaultValue: defaultScheme.onTertiaryFixed,
      ),
    );
    properties.add(
      ColorProperty(
        'onTertiaryFixedVariant',
        onTertiaryFixedVariant,
        defaultValue: defaultScheme.onTertiaryFixedVariant,
      ),
    );
    properties.add(ColorProperty('error', error, defaultValue: defaultScheme.error));
    properties.add(ColorProperty('onError', onError, defaultValue: defaultScheme.onError));
    properties.add(
      ColorProperty('errorContainer', errorContainer, defaultValue: defaultScheme.errorContainer),
    );
    properties.add(
      ColorProperty(
        'onErrorContainer',
        onErrorContainer,
        defaultValue: defaultScheme.onErrorContainer,
      ),
    );
    properties.add(ColorProperty('surface', surface, defaultValue: defaultScheme.surface));
    properties.add(ColorProperty('onSurface', onSurface, defaultValue: defaultScheme.onSurface));
    properties.add(ColorProperty('surfaceDim', surfaceDim, defaultValue: defaultScheme.surfaceDim));
    properties.add(
      ColorProperty('surfaceBright', surfaceBright, defaultValue: defaultScheme.surfaceBright),
    );
    properties.add(
      ColorProperty(
        'surfaceContainerLowest',
        surfaceContainerLowest,
        defaultValue: defaultScheme.surfaceContainerLowest,
      ),
    );
    properties.add(
      ColorProperty(
        'surfaceContainerLow',
        surfaceContainerLow,
        defaultValue: defaultScheme.surfaceContainerLow,
      ),
    );
    properties.add(
      ColorProperty(
        'surfaceContainer',
        surfaceContainer,
        defaultValue: defaultScheme.surfaceContainer,
      ),
    );
    properties.add(
      ColorProperty(
        'surfaceContainerHigh',
        surfaceContainerHigh,
        defaultValue: defaultScheme.surfaceContainerHigh,
      ),
    );
    properties.add(
      ColorProperty(
        'surfaceContainerHighest',
        surfaceContainerHighest,
        defaultValue: defaultScheme.surfaceContainerHighest,
      ),
    );
    properties.add(
      ColorProperty(
        'onSurfaceVariant',
        onSurfaceVariant,
        defaultValue: defaultScheme.onSurfaceVariant,
      ),
    );
    properties.add(ColorProperty('outline', outline, defaultValue: defaultScheme.outline));
    properties.add(
      ColorProperty('outlineVariant', outlineVariant, defaultValue: defaultScheme.outlineVariant),
    );
    properties.add(ColorProperty('shadow', shadow, defaultValue: defaultScheme.shadow));
    properties.add(ColorProperty('scrim', scrim, defaultValue: defaultScheme.scrim));
    properties.add(
      ColorProperty('inverseSurface', inverseSurface, defaultValue: defaultScheme.inverseSurface),
    );
    properties.add(
      ColorProperty(
        'onInverseSurface',
        onInverseSurface,
        defaultValue: defaultScheme.onInverseSurface,
      ),
    );
    properties.add(
      ColorProperty('inversePrimary', inversePrimary, defaultValue: defaultScheme.inversePrimary),
    );
    properties.add(
      ColorProperty('surfaceTint', surfaceTint, defaultValue: defaultScheme.surfaceTint),
    );
    // DEPRECATED (newest deprecations at the bottom)
    properties.add(ColorProperty('background', background, defaultValue: defaultScheme.background));
    properties.add(
      ColorProperty('onBackground', onBackground, defaultValue: defaultScheme.onBackground),
    );
    properties.add(
      ColorProperty('surfaceVariant', surfaceVariant, defaultValue: defaultScheme.surfaceVariant),
    );
  }

  /// Generate a [ColorScheme] derived from the given `imageProvider`.
  ///
  /// Material Color Utilities extracts the dominant color from the
  /// supplied [ImageProvider]. Using this color, a [ColorScheme] is generated
  /// with harmonious colors that meet contrast requirements for accessibility.
  ///
  /// If any of the optional color parameters are non-null, they will be
  /// used in place of the generated colors for that field in the resulting
  /// [ColorScheme]. This allows apps to override specific colors for their
  /// needs.
  ///
  /// Given the nature of the algorithm, the most dominant color of the
  /// `imageProvider` may not wind up as one of the [ColorScheme] colors.
  ///
  /// The provided image will be scaled down to a maximum size of 112x112 pixels
  /// during color extraction.
  ///
  /// {@tool dartpad}
  /// This sample shows how to use [ColorScheme.fromImageProvider] to create
  /// content-based dynamic color schemes.
  ///
  /// ** See code in examples/api/lib/material/color_scheme/dynamic_content_color.0.dart **
  /// {@end-tool}
  ///
  /// See also:
  ///
  ///  * [M3 Guidelines: Dynamic color from content](https://m3.material.io/styles/color/dynamic-color/user-generated-color#8af550b9-a19e-4e9f-bb0a-7f611fed5d0f)
  ///  * <https://pub.dev/packages/dynamic_color>, a package to create
  ///    [ColorScheme]s based on a platform's implementation of dynamic color.
  ///  * <https://m3.material.io/styles/color/the-color-system/color-roles>, the
  ///    Material 3 Color system specification.
  ///  * <https://pub.dev/packages/material_color_utilities>, the package
  ///    used to algorithmically determine the dominant color and to generate
  ///    the [ColorScheme].
  static Future<ColorScheme> fromImageProvider({
    required ImageProvider provider,
    Brightness brightness = Brightness.light,
    DynamicSchemeVariant dynamicSchemeVariant = DynamicSchemeVariant.tonalSpot,
    double contrastLevel = 0.0,
    Color? primary,
    Color? onPrimary,
    Color? primaryContainer,
    Color? onPrimaryContainer,
    Color? primaryFixed,
    Color? primaryFixedDim,
    Color? onPrimaryFixed,
    Color? onPrimaryFixedVariant,
    Color? secondary,
    Color? onSecondary,
    Color? secondaryContainer,
    Color? onSecondaryContainer,
    Color? secondaryFixed,
    Color? secondaryFixedDim,
    Color? onSecondaryFixed,
    Color? onSecondaryFixedVariant,
    Color? tertiary,
    Color? onTertiary,
    Color? tertiaryContainer,
    Color? onTertiaryContainer,
    Color? tertiaryFixed,
    Color? tertiaryFixedDim,
    Color? onTertiaryFixed,
    Color? onTertiaryFixedVariant,
    Color? error,
    Color? onError,
    Color? errorContainer,
    Color? onErrorContainer,
    Color? outline,
    Color? outlineVariant,
    Color? surface,
    Color? onSurface,
    Color? surfaceDim,
    Color? surfaceBright,
    Color? surfaceContainerLowest,
    Color? surfaceContainerLow,
    Color? surfaceContainer,
    Color? surfaceContainerHigh,
    Color? surfaceContainerHighest,
    Color? onSurfaceVariant,
    Color? inverseSurface,
    Color? onInverseSurface,
    Color? inversePrimary,
    Color? shadow,
    Color? scrim,
    Color? surfaceTint,
    @Deprecated(
      'Use surface instead. '
      'This feature was deprecated after v3.18.0-0.1.pre.',
    )
    Color? background,
    @Deprecated(
      'Use onSurface instead. '
      'This feature was deprecated after v3.18.0-0.1.pre.',
    )
    Color? onBackground,
    @Deprecated(
      'Use surfaceContainerHighest instead. '
      'This feature was deprecated after v3.18.0-0.1.pre.',
    )
    Color? surfaceVariant,
  }) async {
    // Extract dominant colors from image.
    final QuantizerResult quantizerResult = await _extractColorsFromImageProvider(provider);
    final Map<int, int> colorToCount = quantizerResult.colorToCount.map(
      (int key, int value) => MapEntry<int, int>(_getArgbFromAbgr(key), value),
    );

    // Score colors for color scheme suitability.
    final List<int> scoredResults = Score.score(colorToCount, desired: 1);
    final ui.Color baseColor = Color(scoredResults.first);

    final DynamicScheme scheme = _buildDynamicScheme(
      brightness,
      baseColor,
      dynamicSchemeVariant,
      contrastLevel,
    );

    return ColorScheme(
      primary: primary ?? Color(MaterialDynamicColors.primary.getArgb(scheme)),
      onPrimary: onPrimary ?? Color(MaterialDynamicColors.onPrimary.getArgb(scheme)),
      primaryContainer:
          primaryContainer ?? Color(MaterialDynamicColors.primaryContainer.getArgb(scheme)),
      onPrimaryContainer:
          onPrimaryContainer ?? Color(MaterialDynamicColors.onPrimaryContainer.getArgb(scheme)),
      primaryFixed: primaryFixed ?? Color(MaterialDynamicColors.primaryFixed.getArgb(scheme)),
      primaryFixedDim:
          primaryFixedDim ?? Color(MaterialDynamicColors.primaryFixedDim.getArgb(scheme)),
      onPrimaryFixed: onPrimaryFixed ?? Color(MaterialDynamicColors.onPrimaryFixed.getArgb(scheme)),
      onPrimaryFixedVariant:
          onPrimaryFixedVariant ??
          Color(MaterialDynamicColors.onPrimaryFixedVariant.getArgb(scheme)),
      secondary: secondary ?? Color(MaterialDynamicColors.secondary.getArgb(scheme)),
      onSecondary: onSecondary ?? Color(MaterialDynamicColors.onSecondary.getArgb(scheme)),
      secondaryContainer:
          secondaryContainer ?? Color(MaterialDynamicColors.secondaryContainer.getArgb(scheme)),
      onSecondaryContainer:
          onSecondaryContainer ?? Color(MaterialDynamicColors.onSecondaryContainer.getArgb(scheme)),
      secondaryFixed: secondaryFixed ?? Color(MaterialDynamicColors.secondaryFixed.getArgb(scheme)),
      secondaryFixedDim:
          secondaryFixedDim ?? Color(MaterialDynamicColors.secondaryFixedDim.getArgb(scheme)),
      onSecondaryFixed:
          onSecondaryFixed ?? Color(MaterialDynamicColors.onSecondaryFixed.getArgb(scheme)),
      onSecondaryFixedVariant:
          onSecondaryFixedVariant ??
          Color(MaterialDynamicColors.onSecondaryFixedVariant.getArgb(scheme)),
      tertiary: tertiary ?? Color(MaterialDynamicColors.tertiary.getArgb(scheme)),
      onTertiary: onTertiary ?? Color(MaterialDynamicColors.onTertiary.getArgb(scheme)),
      tertiaryContainer:
          tertiaryContainer ?? Color(MaterialDynamicColors.tertiaryContainer.getArgb(scheme)),
      onTertiaryContainer:
          onTertiaryContainer ?? Color(MaterialDynamicColors.onTertiaryContainer.getArgb(scheme)),
      tertiaryFixed: tertiaryFixed ?? Color(MaterialDynamicColors.tertiaryFixed.getArgb(scheme)),
      tertiaryFixedDim:
          tertiaryFixedDim ?? Color(MaterialDynamicColors.tertiaryFixedDim.getArgb(scheme)),
      onTertiaryFixed:
          onTertiaryFixed ?? Color(MaterialDynamicColors.onTertiaryFixed.getArgb(scheme)),
      onTertiaryFixedVariant:
          onTertiaryFixedVariant ??
          Color(MaterialDynamicColors.onTertiaryFixedVariant.getArgb(scheme)),
      error: error ?? Color(MaterialDynamicColors.error.getArgb(scheme)),
      onError: onError ?? Color(MaterialDynamicColors.onError.getArgb(scheme)),
      errorContainer: errorContainer ?? Color(MaterialDynamicColors.errorContainer.getArgb(scheme)),
      onErrorContainer:
          onErrorContainer ?? Color(MaterialDynamicColors.onErrorContainer.getArgb(scheme)),
      outline: outline ?? Color(MaterialDynamicColors.outline.getArgb(scheme)),
      outlineVariant: outlineVariant ?? Color(MaterialDynamicColors.outlineVariant.getArgb(scheme)),
      surface: surface ?? Color(MaterialDynamicColors.surface.getArgb(scheme)),
      surfaceDim: surfaceDim ?? Color(MaterialDynamicColors.surfaceDim.getArgb(scheme)),
      surfaceBright: surfaceBright ?? Color(MaterialDynamicColors.surfaceBright.getArgb(scheme)),
      surfaceContainerLowest:
          surfaceContainerLowest ??
          Color(MaterialDynamicColors.surfaceContainerLowest.getArgb(scheme)),
      surfaceContainerLow:
          surfaceContainerLow ?? Color(MaterialDynamicColors.surfaceContainerLow.getArgb(scheme)),
      surfaceContainer:
          surfaceContainer ?? Color(MaterialDynamicColors.surfaceContainer.getArgb(scheme)),
      surfaceContainerHigh:
          surfaceContainerHigh ?? Color(MaterialDynamicColors.surfaceContainerHigh.getArgb(scheme)),
      surfaceContainerHighest:
          surfaceContainerHighest ??
          Color(MaterialDynamicColors.surfaceContainerHighest.getArgb(scheme)),
      onSurface: onSurface ?? Color(MaterialDynamicColors.onSurface.getArgb(scheme)),
      onSurfaceVariant:
          onSurfaceVariant ?? Color(MaterialDynamicColors.onSurfaceVariant.getArgb(scheme)),
      inverseSurface: inverseSurface ?? Color(MaterialDynamicColors.inverseSurface.getArgb(scheme)),
      onInverseSurface:
          onInverseSurface ?? Color(MaterialDynamicColors.inverseOnSurface.getArgb(scheme)),
      inversePrimary: inversePrimary ?? Color(MaterialDynamicColors.inversePrimary.getArgb(scheme)),
      shadow: shadow ?? Color(MaterialDynamicColors.shadow.getArgb(scheme)),
      scrim: scrim ?? Color(MaterialDynamicColors.scrim.getArgb(scheme)),
      surfaceTint: surfaceTint ?? Color(MaterialDynamicColors.primary.getArgb(scheme)),
      brightness: brightness,
      // DEPRECATED (newest deprecations at the bottom)
      background: background ?? Color(MaterialDynamicColors.background.getArgb(scheme)),
      onBackground: onBackground ?? Color(MaterialDynamicColors.onBackground.getArgb(scheme)),
      surfaceVariant: surfaceVariant ?? Color(MaterialDynamicColors.surfaceVariant.getArgb(scheme)),
    );
  }

  // ColorScheme.fromImageProvider() utilities.

  // Extracts bytes from an [ImageProvider] and returns a [QuantizerResult]
  // containing the most dominant colors.
  static Future<QuantizerResult> _extractColorsFromImageProvider(
    ImageProvider imageProvider,
  ) async {
    final ui.Image scaledImage = await _imageProviderToScaled(imageProvider);
    final ByteData? imageBytes = await scaledImage.toByteData();

    final QuantizerResult quantizerResult = await QuantizerCelebi().quantize(
      imageBytes!.buffer.asUint32List(),
      128,
      returnInputPixelToClusterPixel: true,
    );
    return quantizerResult;
  }

  // Scale image size down to reduce computation time of color extraction.
  static Future<ui.Image> _imageProviderToScaled(ImageProvider imageProvider) async {
    const double maxDimension = 112.0;
    final ImageStream stream = imageProvider.resolve(
      const ImageConfiguration(size: Size(maxDimension, maxDimension)),
    );
    final Completer<ui.Image> imageCompleter = Completer<ui.Image>();
    late ImageStreamListener listener;
    late ui.Image scaledImage;
    Timer? loadFailureTimeout;

    listener = ImageStreamListener(
      (ImageInfo info, bool sync) async {
        loadFailureTimeout?.cancel();
        stream.removeListener(listener);
        final ui.Image image = info.image;
        final int width = image.width;
        final int height = image.height;
        double paintWidth = width.toDouble();
        double paintHeight = height.toDouble();
        assert(width > 0 && height > 0);

        final bool rescale = width > maxDimension || height > maxDimension;
        if (rescale) {
          paintWidth = (width > height) ? maxDimension : (maxDimension / height) * width;
          paintHeight = (height > width) ? maxDimension : (maxDimension / width) * height;
        }
        final ui.PictureRecorder pictureRecorder = ui.PictureRecorder();
        final Canvas canvas = Canvas(pictureRecorder);
        paintImage(
          canvas: canvas,
          rect: Rect.fromLTRB(0, 0, paintWidth, paintHeight),
          image: image,
          filterQuality: FilterQuality.none,
        );

        final ui.Picture picture = pictureRecorder.endRecording();
        scaledImage = await picture.toImage(paintWidth.toInt(), paintHeight.toInt());
        imageCompleter.complete(info.image);
      },
      onError: (Object exception, StackTrace? stackTrace) {
        stream.removeListener(listener);
        throw Exception('Failed to render image: $exception');
      },
    );

    loadFailureTimeout = Timer(const Duration(seconds: 5), () {
      stream.removeListener(listener);
      imageCompleter.completeError(TimeoutException('Timeout occurred trying to load image'));
    });

    stream.addListener(listener);
    await imageCompleter.future;
    return scaledImage;
  }

  // Converts AABBGGRR color int to AARRGGBB format.
  static int _getArgbFromAbgr(int abgr) {
    const int exceptRMask = 0xFF00FFFF;
    const int onlyRMask = ~exceptRMask;
    const int exceptBMask = 0xFFFFFF00;
    const int onlyBMask = ~exceptBMask;
    final int r = (abgr & onlyRMask) >> 16;
    final int b = abgr & onlyBMask;
    return (abgr & exceptRMask & exceptBMask) | (b << 16) | r;
  }

  static DynamicScheme _buildDynamicScheme(
    Brightness brightness,
    Color seedColor,
    DynamicSchemeVariant schemeVariant,
    double contrastLevel,
  ) {
    assert(
      contrastLevel >= -1.0 && contrastLevel <= 1.0,
      'contrastLevel must be between -1.0 and 1.0 inclusive.',
    );
    final bool isDark = brightness == Brightness.dark;
    final Hct sourceColor = Hct.fromInt(seedColor.value);
    return switch (schemeVariant) {
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
  }

  /// The [ThemeData.colorScheme] of the ambient [Theme].
  ///
  /// Equivalent to `Theme.of(context).colorScheme`.
  static ColorScheme of(BuildContext context) => Theme.of(context).colorScheme;
}
