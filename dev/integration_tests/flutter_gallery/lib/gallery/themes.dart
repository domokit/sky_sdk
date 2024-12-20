// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

final ThemeData kLightGalleryTheme = _buildLightTheme();
final ThemeData kDarkGalleryTheme = _buildDarkTheme();

TextTheme _buildTextTheme(TextTheme base) {
  return base.copyWith(titleLarge: base.titleLarge!.copyWith(fontFamily: 'GoogleSans'));
}

ThemeData _buildDarkTheme() {
  const Color primaryColor = Color(0xFF0175c2);
  const Color secondaryColor = Color(0xFF13B9FD);
  final ColorScheme colorScheme = const ColorScheme.dark().copyWith(
    primary: primaryColor,
    secondary: secondaryColor,
    onPrimary: Colors.white,
    error: const Color(0xFFB00020),
    surface: const Color(0xFF202124),
  );
  final ThemeData base = ThemeData(
    useMaterial3: false,
    brightness: Brightness.dark,
    colorScheme: colorScheme,
    primaryColor: primaryColor,
    primaryColorDark: const Color(0xFF0050a0),
    primaryColorLight: secondaryColor,
    indicatorColor: Colors.white,
    canvasColor: const Color(0xFF202124),
    scaffoldBackgroundColor: const Color(0xFF202124),
  );
  return base.copyWith(
    textTheme: _buildTextTheme(base.textTheme),
    primaryTextTheme: _buildTextTheme(base.primaryTextTheme),
  );
}

ThemeData _buildLightTheme() {
  const Color primaryColor = Color(0xFF0175c2);
  const Color secondaryColor = Color(0xFF13B9FD);
  final ColorScheme colorScheme = const ColorScheme.light().copyWith(
    primary: primaryColor,
    secondary: secondaryColor,
    error: const Color(0xFFB00020),
  );
  final ThemeData base = ThemeData(
    useMaterial3: false,
    brightness: Brightness.light,
    colorScheme: colorScheme,
    primaryColor: primaryColor,
    indicatorColor: Colors.white,
    splashColor: Colors.white24,
    splashFactory: InkRipple.splashFactory,
    canvasColor: Colors.white,
    scaffoldBackgroundColor: Colors.white,
  );
  return base.copyWith(
    textTheme: _buildTextTheme(base.textTheme),
    primaryTextTheme: _buildTextTheme(base.primaryTextTheme),
  );
}
