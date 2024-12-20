// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

void main() {
  test('IconThemeData control test', () {
    const IconThemeData data = IconThemeData(
      size: 16.0,
      fill: 0.5,
      weight: 600,
      grade: 25,
      opticalSize: 45,
      color: Color(0xAAAAAAAA),
      opacity: 0.5,
      shadows: <Shadow>[
        Shadow(color: Color(0xAAAAAAAA), blurRadius: 1.0, offset: Offset(1.0, 1.0)),
      ],
    );
    expect(data, hasOneLineDescription);
    expect(data, equals(data.copyWith()));
    expect(data.hashCode, equals(data.copyWith().hashCode));

    final IconThemeData lerped = IconThemeData.lerp(data, const IconThemeData.fallback(), 0.25);
    expect(lerped.size, 18.0);
    expect(lerped.fill, 0.375);
    expect(lerped.weight, 550.0);
    expect(lerped.grade, 18.75);
    expect(lerped.opticalSize, 45.75);
    expect(lerped.color, isSameColorAs(const Color(0xBF7F7F7F)));
    expect(lerped.opacity, 0.625);
    expect(lerped.shadows, const <Shadow>[
      Shadow(color: Color(0xAAAAAAAA), blurRadius: 0.75, offset: Offset(0.75, 0.75)),
    ]);
  });

  group('IconThemeData lerp', () {
    const IconThemeData data = IconThemeData(
      size: 16.0,
      fill: 0.5,
      weight: 600,
      grade: 25,
      opticalSize: 45,
      color: Color(0xFFFFFFFF),
      opacity: 1.0,
      shadows: <Shadow>[
        Shadow(color: Color(0xFFFFFFFF), blurRadius: 1.0, offset: Offset(1.0, 1.0)),
      ],
    );

    test('with first null', () {
      final IconThemeData lerped = IconThemeData.lerp(null, data, 0.25);

      expect(lerped.size, 4.0);
      expect(lerped.fill, 0.125);
      expect(lerped.weight, 150.0);
      expect(lerped.grade, 6.25);
      expect(lerped.opticalSize, 11.25);
      expect(lerped.color, isSameColorAs(const Color(0x40FFFFFF)));
      expect(lerped.opacity, 0.25);
      expect(lerped.shadows, const <Shadow>[
        Shadow(color: Color(0xFFFFFFFF), blurRadius: 0.25, offset: Offset(0.25, 0.25)),
      ]);
    });

    test('IconThemeData lerp special cases', () {
      expect(IconThemeData.lerp(null, null, 0), const IconThemeData());
      const IconThemeData data = IconThemeData();
      expect(identical(IconThemeData.lerp(data, data, 0.5), data), true);
    });

    test('with second null', () {
      final IconThemeData lerped = IconThemeData.lerp(data, null, 0.25);

      expect(lerped.size, 12.0);
      expect(lerped.fill, 0.375);
      expect(lerped.weight, 450.0);
      expect(lerped.grade, 18.75);
      expect(lerped.opticalSize, 33.75);
      expect(lerped.color, isSameColorAs(const Color(0xBFFFFFFF)));
      expect(lerped.opacity, 0.75);
      expect(lerped.shadows, const <Shadow>[
        Shadow(color: Color(0xFFFFFFFF), blurRadius: 0.75, offset: Offset(0.75, 0.75)),
      ]);
    });

    test('with both null', () {
      final IconThemeData lerped = IconThemeData.lerp(null, null, 0.25);

      expect(lerped.size, null);
      expect(lerped.fill, null);
      expect(lerped.weight, null);
      expect(lerped.grade, null);
      expect(lerped.opticalSize, null);
      expect(lerped.color, null);
      expect(lerped.opacity, null);
      expect(lerped.shadows, null);
    });
  });

  test('Throws if given invalid values', () {
    expect(() => IconThemeData(fill: -0.1), throwsAssertionError);
    expect(() => IconThemeData(fill: 1.1), throwsAssertionError);
    expect(() => IconThemeData(weight: -0.1), throwsAssertionError);
    expect(() => IconThemeData(weight: 0.0), throwsAssertionError);
    expect(() => IconThemeData(opticalSize: -0.1), throwsAssertionError);
    expect(() => IconThemeData(opticalSize: 0), throwsAssertionError);
  });
}
