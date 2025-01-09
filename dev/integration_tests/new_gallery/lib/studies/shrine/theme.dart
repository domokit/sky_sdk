// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:google_fonts/google_fonts.dart';

import '../../layout/letter_spacing.dart';
import 'colors.dart';
import 'supplemental/cut_corners_border.dart';

const double defaultLetterSpacing = 0.03;
const double mediumLetterSpacing = 0.04;
const double largeLetterSpacing = 1.0;

final ThemeData shrineTheme = _buildShrineTheme();

IconThemeData _customIconTheme(IconThemeData original) {
  return original.copyWith(color: shrineBrown900);
}

ThemeData _buildShrineTheme() {
  final ThemeData base = ThemeData.light();
  return base.copyWith(
    appBarTheme: const AppBarTheme(systemOverlayStyle: SystemUiOverlayStyle.dark, elevation: 0),
    scaffoldBackgroundColor: shrineBackgroundWhite,
    cardColor: shrineBackgroundWhite,
    primaryIconTheme: _customIconTheme(base.iconTheme),
    inputDecorationTheme: const InputDecorationTheme(
      border: CutCornersBorder(borderSide: BorderSide(color: shrineBrown900, width: 0.5)),
      contentPadding: EdgeInsets.symmetric(vertical: 20, horizontal: 16),
    ),
    textTheme: _buildShrineTextTheme(base.textTheme),
    textSelectionTheme: const TextSelectionThemeData(selectionColor: shrinePink100),
    primaryTextTheme: _buildShrineTextTheme(base.primaryTextTheme),
    iconTheme: _customIconTheme(base.iconTheme),
    colorScheme: _shrineColorScheme.copyWith(error: shrineErrorRed, primary: shrinePink100),
  );
}

TextTheme _buildShrineTextTheme(TextTheme base) {
  return GoogleFonts.rubikTextTheme(
    base
        .copyWith(
          headlineSmall: base.headlineSmall!.copyWith(
            fontWeight: FontWeight.w500,
            letterSpacing: letterSpacingOrNone(defaultLetterSpacing),
          ),
          titleLarge: base.titleLarge!.copyWith(
            fontSize: 18,
            letterSpacing: letterSpacingOrNone(defaultLetterSpacing),
          ),
          bodySmall: base.bodySmall!.copyWith(
            fontWeight: FontWeight.w400,
            fontSize: 14,
            letterSpacing: letterSpacingOrNone(defaultLetterSpacing),
          ),
          bodyLarge: base.bodyLarge!.copyWith(
            fontWeight: FontWeight.w500,
            fontSize: 16,
            letterSpacing: letterSpacingOrNone(defaultLetterSpacing),
          ),
          bodyMedium: base.bodyMedium!.copyWith(
            letterSpacing: letterSpacingOrNone(defaultLetterSpacing),
          ),
          titleMedium: base.titleMedium!.copyWith(
            letterSpacing: letterSpacingOrNone(defaultLetterSpacing),
          ),
          headlineMedium: base.headlineMedium!.copyWith(
            letterSpacing: letterSpacingOrNone(defaultLetterSpacing),
          ),
          labelLarge: base.labelLarge!.copyWith(
            fontWeight: FontWeight.w500,
            fontSize: 14,
            letterSpacing: letterSpacingOrNone(defaultLetterSpacing),
          ),
        )
        .apply(displayColor: shrineBrown900, bodyColor: shrineBrown900),
  );
}

const ColorScheme _shrineColorScheme = ColorScheme(
  primary: shrinePink100,
  primaryContainer: shrineBrown900,
  secondary: shrinePink50,
  secondaryContainer: shrineBrown900,
  surface: shrineSurfaceWhite,
  background: shrineBackgroundWhite,
  error: shrineErrorRed,
  onPrimary: shrineBrown900,
  onSecondary: shrineBrown900,
  onSurface: shrineBrown900,
  onBackground: shrineBrown900,
  onError: shrineSurfaceWhite,
  brightness: Brightness.light,
);
