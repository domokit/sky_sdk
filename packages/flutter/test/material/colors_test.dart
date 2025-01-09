// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';

const List<int> primaryKeys = <int>[50, 100, 200, 300, 400, 500, 600, 700, 800, 900];
const List<int> accentKeys = <int>[100, 200, 400, 700];

void main() {
  test('MaterialColor basic functionality', () {
    const MaterialColor color = MaterialColor(500, <int, Color>{
      50: Color(0x00000050),
      100: Color(0x00000100),
      200: Color(0x00000200),
      300: Color(0x00000300),
      400: Color(0x00000400),
      500: Color(0x00000500),
      600: Color(0x00000600),
      700: Color(0x00000700),
      800: Color(0x00000800),
      900: Color(0x00000900),
    });

    expect(color.value, 500);

    expect(color[50]!.value, 0x00000050);
    expect(color[100]!.value, 0x00000100);
    expect(color[200]!.value, 0x00000200);
    expect(color[300]!.value, 0x00000300);
    expect(color[400]!.value, 0x00000400);
    expect(color[500]!.value, 0x00000500);
    expect(color[600]!.value, 0x00000600);
    expect(color[700]!.value, 0x00000700);
    expect(color[800]!.value, 0x00000800);
    expect(color[900]!.value, 0x00000900);

    expect(color.shade50.value, 0x00000050);
    expect(color.shade100.value, 0x00000100);
    expect(color.shade200.value, 0x00000200);
    expect(color.shade300.value, 0x00000300);
    expect(color.shade400.value, 0x00000400);
    expect(color.shade500.value, 0x00000500);
    expect(color.shade600.value, 0x00000600);
    expect(color.shade700.value, 0x00000700);
    expect(color.shade800.value, 0x00000800);
    expect(color.shade900.value, 0x00000900);
  });

  test('Colors swatches do not contain duplicates', () {
    for (final MaterialColor color in Colors.primaries) {
      expect(primaryKeys.map<Color>((int key) => color[key]!).toSet().length, primaryKeys.length);
    }

    expect(
      primaryKeys.map<Color>((int key) => Colors.grey[key]!).toSet().length,
      primaryKeys.length,
    );

    for (final MaterialAccentColor color in Colors.accents) {
      expect(accentKeys.map<Color>((int key) => color[key]!).toSet().length, accentKeys.length);
    }
  });

  test('All color swatch colors are opaque and equal their primary color', () {
    for (final MaterialColor color in Colors.primaries) {
      expect(color.value, color.shade500.value);
      for (final int key in primaryKeys) {
        expect(color[key]!.alpha, 0xFF);
      }
    }

    expect(Colors.grey.value, Colors.grey.shade500.value);
    for (final int key in primaryKeys) {
      expect(Colors.grey[key]!.alpha, 0xFF);
    }

    for (final MaterialAccentColor color in Colors.accents) {
      expect(color.value, color.shade200.value);
      for (final int key in accentKeys) {
        expect(color[key]!.alpha, 0xFF);
      }
    }
  });
}
