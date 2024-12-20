// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:google_fonts/google_fonts.dart';

import '../../layout/letter_spacing.dart';
import 'colors.dart';

final ThemeData craneTheme = _buildCraneTheme();

IconThemeData _customIconTheme(IconThemeData original, Color color) {
  return original.copyWith(color: color);
}

ThemeData _buildCraneTheme() {
  final ThemeData base = ThemeData.light();

  return base.copyWith(
    colorScheme: const ColorScheme.light().copyWith(
      primary: cranePurple800,
      secondary: craneRed700,
      error: craneErrorOrange,
    ),
    hintColor: craneWhite60,
    indicatorColor: cranePrimaryWhite,
    scaffoldBackgroundColor: cranePrimaryWhite,
    cardColor: cranePrimaryWhite,
    highlightColor: Colors.transparent,
    textTheme: _buildCraneTextTheme(base.textTheme),
    textSelectionTheme: const TextSelectionThemeData(selectionColor: cranePurple700),
    primaryTextTheme: _buildCraneTextTheme(base.primaryTextTheme),
    iconTheme: _customIconTheme(base.iconTheme, craneWhite60),
    primaryIconTheme: _customIconTheme(base.iconTheme, cranePrimaryWhite),
  );
}

TextTheme _buildCraneTextTheme(TextTheme base) {
  return GoogleFonts.ralewayTextTheme(
    base.copyWith(
      displayLarge: base.displayLarge!.copyWith(fontWeight: FontWeight.w300, fontSize: 96),
      displayMedium: base.displayMedium!.copyWith(fontWeight: FontWeight.w400, fontSize: 60),
      displaySmall: base.displaySmall!.copyWith(fontWeight: FontWeight.w600, fontSize: 48),
      headlineMedium: base.headlineMedium!.copyWith(fontWeight: FontWeight.w600, fontSize: 34),
      headlineSmall: base.headlineSmall!.copyWith(fontWeight: FontWeight.w600, fontSize: 24),
      titleLarge: base.titleLarge!.copyWith(fontWeight: FontWeight.w600, fontSize: 20),
      titleMedium: base.titleMedium!.copyWith(
        fontWeight: FontWeight.w500,
        fontSize: 16,
        letterSpacing: letterSpacingOrNone(0.5),
      ),
      titleSmall: base.titleSmall!.copyWith(
        fontWeight: FontWeight.w600,
        fontSize: 12,
        color: craneGrey,
      ),
      bodyLarge: base.bodyLarge!.copyWith(fontWeight: FontWeight.w500, fontSize: 16),
      bodyMedium: base.bodyMedium!.copyWith(fontWeight: FontWeight.w400, fontSize: 14),
      labelLarge: base.labelLarge!.copyWith(
        fontWeight: FontWeight.w600,
        fontSize: 13,
        letterSpacing: letterSpacingOrNone(0.8),
      ),
      bodySmall: base.bodySmall!.copyWith(
        fontWeight: FontWeight.w500,
        fontSize: 12,
        color: craneGrey,
      ),
      labelSmall: base.labelSmall!.copyWith(fontWeight: FontWeight.w600, fontSize: 12),
    ),
  );
}
