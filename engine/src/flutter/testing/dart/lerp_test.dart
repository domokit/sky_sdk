// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui';

import 'package:test/test.dart';

const precisionErrorTolerance = 1e-4;

// These tests should be kept in sync with the web tests in
// lib/web_ui/test/lerp_test.dart.
void main() {
  test('lerpDouble should return null if and only if both inputs are null', () {
    expect(lerpDouble(null, null, 1.0), isNull);
    expect(lerpDouble(5.0, null, 0.25), isNotNull);
    expect(lerpDouble(null, 5.0, 0.25), isNotNull);

    expect(lerpDouble(5, null, 0.25), isNotNull);
    expect(lerpDouble(null, 5, 0.25), isNotNull);
  });

  test('lerpDouble should treat a null input as 0 if the other input is non-null', () {
    expect(lerpDouble(null, 10.0, 0.25), closeTo(2.5, precisionErrorTolerance));
    expect(lerpDouble(10.0, null, 0.25), closeTo(7.5, precisionErrorTolerance));

    expect(lerpDouble(null, 10, 0.25), closeTo(2.5, precisionErrorTolerance));
    expect(lerpDouble(10, null, 0.25), closeTo(7.5, precisionErrorTolerance));
  });

  test('lerpDouble should handle interpolation values < 0.0', () {
    expect(lerpDouble(0.0, 10.0, -5.0), closeTo(-50.0, precisionErrorTolerance));
    expect(lerpDouble(10.0, 0.0, -5.0), closeTo(60.0, precisionErrorTolerance));

    expect(lerpDouble(0, 10, -5), closeTo(-50, precisionErrorTolerance));
    expect(lerpDouble(10, 0, -5), closeTo(60, precisionErrorTolerance));
  });

  test('lerpDouble should return the start value at 0.0', () {
    expect(lerpDouble(2.0, 10.0, 0.0), 2.0);
    expect(lerpDouble(10.0, 2.0, 0.0), 10.0);

    expect(lerpDouble(2, 10, 0), 2);
    expect(lerpDouble(10, 2, 0), 10);
  });

  test('lerpDouble should interpolate between two values', () {
    expect(lerpDouble(0.0, 10.0, 0.25), closeTo(2.5, precisionErrorTolerance));
    expect(lerpDouble(10.0, 0.0, 0.25), closeTo(7.5, precisionErrorTolerance));

    expect(lerpDouble(0, 10, 0.25), closeTo(2.5, precisionErrorTolerance));
    expect(lerpDouble(10, 0, 0.25), closeTo(7.5, precisionErrorTolerance));

    // Exact answer: 20.0 - 1.0e-29
    expect(lerpDouble(10.0, 1.0e30, 1.0e-29), closeTo(20.0, precisionErrorTolerance));

    // Exact answer: 5.0 + 5.0e29
    expect(lerpDouble(10.0, 1.0e30, 0.5), closeTo(5.0e29, precisionErrorTolerance));
  });

  test('lerpDouble should return the end value at 1.0', () {
    expect(lerpDouble(2.0, 10.0, 1.0), 10.0);
    expect(lerpDouble(10.0, 2.0, 1.0), 2.0);

    expect(lerpDouble(0, 10, 5), 50);
    expect(lerpDouble(10, 0, 5), -40);

    expect(lerpDouble(1.0e30, 10.0, 1.0), 10.0);
    expect(lerpDouble(10.0, 1.0e30, 0.0), 10.0);
  });

  test('lerpDouble should handle interpolation values > 1.0', () {
    expect(lerpDouble(0.0, 10.0, 5.0), closeTo(50.0, precisionErrorTolerance));
    expect(lerpDouble(10.0, 0.0, 5.0), closeTo(-40.0, precisionErrorTolerance));

    expect(lerpDouble(0, 10, 5), closeTo(50, precisionErrorTolerance));
    expect(lerpDouble(10, 0, 5), closeTo(-40, precisionErrorTolerance));
  });

  test('lerpDouble should return input value in all cases if begin/end are equal', () {
    expect(lerpDouble(10.0, 10.0, 5.0), 10.0);
    expect(lerpDouble(10.0, 10.0, double.nan), 10.0);
    expect(lerpDouble(10.0, 10.0, double.infinity), 10.0);
    expect(lerpDouble(10.0, 10.0, -double.infinity), 10.0);

    expect(lerpDouble(10, 10, 5.0), 10.0);
    expect(lerpDouble(10, 10, double.nan), 10.0);
    expect(lerpDouble(10, 10, double.infinity), 10.0);
    expect(lerpDouble(10, 10, -double.infinity), 10.0);

    expect(lerpDouble(double.nan, double.nan, 5.0), isNaN);
    expect(lerpDouble(double.nan, double.nan, double.nan), isNaN);
    expect(lerpDouble(double.nan, double.nan, double.infinity), isNaN);
    expect(lerpDouble(double.nan, double.nan, -double.infinity), isNaN);

    expect(lerpDouble(double.infinity, double.infinity, 5.0), double.infinity);
    expect(lerpDouble(double.infinity, double.infinity, double.nan), double.infinity);
    expect(lerpDouble(double.infinity, double.infinity, double.infinity), double.infinity);
    expect(lerpDouble(double.infinity, double.infinity, -double.infinity), double.infinity);

    expect(lerpDouble(-double.infinity, -double.infinity, 5.0), -double.infinity);
    expect(lerpDouble(-double.infinity, -double.infinity, double.nan), -double.infinity);
    expect(lerpDouble(-double.infinity, -double.infinity, double.infinity), -double.infinity);
    expect(lerpDouble(-double.infinity, -double.infinity, -double.infinity), -double.infinity);
  });

  test('lerpDouble should throw AssertionError if interpolation value is NaN and a != b', () {
    expect(() => lerpDouble(0.0, 10.0, double.nan), throwsA(isA<AssertionError>()));
  });

  test(
    'lerpDouble should throw AssertionError if interpolation value is +/- infinity and a != b',
    () {
      expect(() => lerpDouble(0.0, 10.0, double.infinity), throwsA(isA<AssertionError>()));
      expect(() => lerpDouble(0.0, 10.0, -double.infinity), throwsA(isA<AssertionError>()));
    },
  );

  test('lerpDouble should throw AssertionError if either start or end are NaN', () {
    expect(() => lerpDouble(double.nan, 10.0, 5.0), throwsA(isA<AssertionError>()));
    expect(() => lerpDouble(0.0, double.nan, 5.0), throwsA(isA<AssertionError>()));
  });

  test('lerpDouble should throw AssertionError if either start or end are +/- infinity', () {
    expect(() => lerpDouble(double.infinity, 10.0, 5.0), throwsA(isA<AssertionError>()));
    expect(() => lerpDouble(-double.infinity, 10.0, 5.0), throwsA(isA<AssertionError>()));
    expect(() => lerpDouble(0.0, double.infinity, 5.0), throwsA(isA<AssertionError>()));
    expect(() => lerpDouble(0.0, -double.infinity, 5.0), throwsA(isA<AssertionError>()));
  });
}
