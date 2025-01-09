// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';

void main() {
  // Changes made in https://github.com/flutter/flutter/pull/93427
  ColorScheme colorScheme = ColorScheme();
  colorScheme = ColorScheme(
    primaryVariant: Colors.black,
    secondaryVariant: Colors.white,
  );
  colorScheme = ColorScheme.light(
    primaryVariant: Colors.black,
    secondaryVariant: Colors.white,
  );
  colorScheme = ColorScheme.dark(
    primaryVariant: Colors.black,
    secondaryVariant: Colors.white,
  );
  colorScheme = ColorScheme.highContrastLight(
    primaryVariant: Colors.black,
    secondaryVariant: Colors.white,
  );
  colorScheme = ColorScheme.highContrastDark(
    primaryVariant: Colors.black,
    secondaryVariant: Colors.white,
  );
  colorScheme = colorScheme.copyWith(
    primaryVariant: Colors.black,
    secondaryVariant: Colors.white,
  );
  colorScheme.primaryVariant; // Removing field reference not supported.
  colorScheme.secondaryVariant;

  // Changes made in https://github.com/flutter/flutter/pull/138521
  ColorScheme colorScheme = ColorScheme();
  colorScheme = ColorScheme(
    background: Colors.black,
    onBackground: Colors.white,
    surfaceVariant: Colors.red,
  );
  colorScheme = ColorScheme(
    background: Colors.black,
    surface: Colors.orange,
    onBackground: Colors.white,
    onSurface: Colors.yellow,
    surfaceVariant: Colors.red,
    surfaceContainerHighest: Colors.blue,
  );
  colorScheme = ColorScheme.light(
    background: Colors.black,
    onBackground: Colors.white,
    surfaceVariant: Colors.red,
  );
  colorScheme = ColorScheme.light(
    background: Colors.black,
    surface: Colors.orange,
    onBackground: Colors.white,
    onSurface: Colors.yellow,
    surfaceVariant: Colors.red,
    surfaceContainerHighest: Colors.blue,
  );
  colorScheme = ColorScheme.dark(
    background: Colors.black,
    onBackground: Colors.white,
    surfaceVariant: Colors.red,
  );
  colorScheme = ColorScheme.dark(
    background: Colors.black,
    surface: Colors.orange,
    onBackground: Colors.white,
    onSurface: Colors.yellow,
    surfaceVariant: Colors.red,
    surfaceContainerHighest: Colors.blue,
  );
  colorScheme = ColorScheme.highContrastLight(
    background: Colors.black,
    onBackground: Colors.white,
    surfaceVariant: Colors.red,
  );
  colorScheme = ColorScheme.highContrastLight(
    background: Colors.black,
    surface: Colors.orange,
    onBackground: Colors.white,
    onSurface: Colors.yellow,
    surfaceVariant: Colors.red,
    surfaceContainerHighest: Colors.blue,
  );
  colorScheme = ColorScheme.highContrastDark(
    background: Colors.black,
    onBackground: Colors.white,
    surfaceVariant: Colors.red,
  );
  colorScheme = ColorScheme.highContrastDark(
    background: Colors.black,
    surface: Colors.orange,
    onBackground: Colors.white,
    onSurface: Colors.yellow,
    surfaceVariant: Colors.red,
    surfaceContainerHighest: Colors.blue,
  );
  colorScheme = colorScheme.copyWith(
    background: Colors.black,
    onBackground: Colors.white,
    surfaceVariant: Colors.red,
  );
  colorScheme = colorScheme.copyWith(
    background: Colors.black,
    surface: Colors.orange,
    onBackground: Colors.white,
    onSurface: Colors.yellow,
    surfaceVariant: Colors.red,
    surfaceContainerHighest: Colors.blue,
  );
  colorScheme.background;
  colorScheme.onBackground;
  colorScheme.surfaceVariant;
}
