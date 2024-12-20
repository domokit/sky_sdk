// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/ui.dart';

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

void testMain() {
  test('GestureSettings has a reasonable toString', () {
    const GestureSettings gestureSettings = GestureSettings(
      physicalDoubleTapSlop: 2.5,
      physicalTouchSlop: 1.5,
    );

    expect(
      gestureSettings.toString(),
      'GestureSettings(physicalTouchSlop: 1.5, physicalDoubleTapSlop: 2.5)',
    );
  });

  test('GestureSettings has a correct equality', () {
    // don't refactor these to be const, that defeats the point!
    final double value = nonconst(2.0);
    final GestureSettings settingsA = GestureSettings(
      physicalDoubleTapSlop: value,
      physicalTouchSlop: 1.0,
    );
    final GestureSettings settingsB = GestureSettings(
      physicalDoubleTapSlop: value,
      physicalTouchSlop: 3.0,
    );
    final GestureSettings settingsC = GestureSettings(
      physicalDoubleTapSlop: value,
      physicalTouchSlop: 1.0,
    );

    expect(settingsA, equals(settingsC));
    expect(settingsC, equals(settingsA));

    expect(settingsA, isNot(equals(settingsB)));
    expect(settingsC, isNot(equals(settingsB)));

    expect(settingsB, isNot(equals(settingsA)));
    expect(settingsB, isNot(equals(settingsC)));
  });

  test('GestureSettings copyWith preserves already set values', () {
    const GestureSettings initial = GestureSettings(
      physicalDoubleTapSlop: 1.0,
      physicalTouchSlop: 1.0,
    );

    final GestureSettings copyA = initial.copyWith();

    expect(copyA.physicalDoubleTapSlop, 1.0);
    expect(copyA.physicalTouchSlop, 1.0);

    final GestureSettings copyB = copyA.copyWith(
      physicalDoubleTapSlop: 2.0,
      physicalTouchSlop: 2.0,
    );

    expect(copyB.physicalDoubleTapSlop, 2.0);
    expect(copyB.physicalTouchSlop, 2.0);
  });

  test('GestureSettings constructor defaults to null', () {
    const GestureSettings settings = GestureSettings();

    expect(settings.physicalDoubleTapSlop, null);
    expect(settings.physicalTouchSlop, null);
  });
}

// Prevent the linter from complaining about a const value so that
// non-identical equality can be tested.
T nonconst<T>(T value) => value;
