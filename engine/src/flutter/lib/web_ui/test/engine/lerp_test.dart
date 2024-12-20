// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';

import 'package:ui/ui.dart';

import '../common/matchers.dart';

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

// These tests should be kept in sync with the VM tests in
// testing/dart/lerp_test.dart.
void testMain() {
  test('lerpDouble should return null if and only if both inputs are null', () {
    expect(lerpDouble(null, null, 1.0), isNull);
    expect(lerpDouble(5.0, null, 0.25), isNotNull);
    expect(lerpDouble(null, 5.0, 0.25), isNotNull);

    expect(lerpDouble(5, null, 0.25), isNotNull);
    expect(lerpDouble(null, 5, 0.25), isNotNull);
  });

  test('lerpDouble should treat a null input as 0 if the other input is non-null', () {
    expect(lerpDouble(null, 10.0, 0.25), within(from: 2.5));
    expect(lerpDouble(10.0, null, 0.25), within(from: 7.5));

    expect(lerpDouble(null, 10, 0.25), within(from: 2.5));
    expect(lerpDouble(10, null, 0.25), within(from: 7.5));
  });

  test('lerpDouble should handle interpolation values < 0.0', () {
    expect(lerpDouble(0.0, 10.0, -5.0), within(from: -50.0));
    expect(lerpDouble(10.0, 0.0, -5.0), within(from: 60.0));

    expect(lerpDouble(0, 10, -5), within(from: -50.0));
    expect(lerpDouble(10, 0, -5), within(from: 60.0));
  });

  test('lerpDouble should return the start value at 0.0', () {
    expect(lerpDouble(2.0, 10.0, 0.0), 2.0);
    expect(lerpDouble(10.0, 2.0, 0.0), 10.0);

    expect(lerpDouble(2, 10, 0), 2);
    expect(lerpDouble(10, 2, 0), 10);
  });

  test('lerpDouble should interpolate between two values', () {
    expect(lerpDouble(0.0, 10.0, 0.25), within(from: 2.5));
    expect(lerpDouble(10.0, 0.0, 0.25), within(from: 7.5));

    expect(lerpDouble(0, 10, 0.25), within(from: 2.5));
    expect(lerpDouble(10, 0, 0.25), within(from: 7.5));

    // Exact answer: 20.0 - 1.0e-29
    expect(lerpDouble(10.0, 1.0e30, 1.0e-29), within(from: 20.0));

    // Exact answer: 5.0 + 5.0e29
    expect(lerpDouble(10.0, 1.0e30, 0.5), within(from: 5.0e29));
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
    expect(lerpDouble(0.0, 10.0, 5.0), within(from: 50.0));
    expect(lerpDouble(10.0, 0.0, 5.0), within(from: -40.0));

    expect(lerpDouble(0, 10, 5), within(from: 50.0));
    expect(lerpDouble(10, 0, 5), within(from: -40.0));
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
    expectAssertion(() => lerpDouble(0.0, 10.0, double.nan));
  });

  test(
    'lerpDouble should throw AssertionError if interpolation value is +/- infinity and a != b',
    () {
      expectAssertion(() => lerpDouble(0.0, 10.0, double.infinity));
      expectAssertion(() => lerpDouble(0.0, 10.0, -double.infinity));
    },
  );

  test('lerpDouble should throw AssertionError if either start or end are NaN', () {
    expectAssertion(() => lerpDouble(double.nan, 10.0, 5.0));
    expectAssertion(() => lerpDouble(0.0, double.nan, 5.0));
  });

  test('lerpDouble should throw AssertionError if either start or end are +/- infinity', () {
    expectAssertion(() => lerpDouble(double.infinity, 10.0, 5.0));
    expectAssertion(() => lerpDouble(-double.infinity, 10.0, 5.0));
    expectAssertion(() => lerpDouble(0.0, double.infinity, 5.0));
    expectAssertion(() => lerpDouble(0.0, -double.infinity, 5.0));
  });
}

typedef DoubleFunction = double? Function();

/// Asserts that `callback` throws an [AssertionError].
///
/// Verifies that the specified callback throws an [AssertionError] when
/// running in with assertions enabled. When asserts are not enabled, such as
/// when running using a release-mode VM with default settings, this acts as a
/// no-op.
void expectAssertion(DoubleFunction callback) {
  bool assertsEnabled = false;
  assert(() {
    assertsEnabled = true;
    return true;
  }());
  if (assertsEnabled) {
    bool threw = false;
    try {
      callback();
    } catch (e) {
      expect(e is AssertionError, isTrue);
      threw = true;
    }
    expect(threw, isTrue);
  }
}
