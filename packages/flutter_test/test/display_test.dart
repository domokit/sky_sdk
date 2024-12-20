// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui';

import 'package:flutter/material.dart';
import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

import 'utils/fake_and_mock_utils.dart';

void main() {
  group('TestDisplay', () {
    Display trueDisplay() => PlatformDispatcher.instance.displays.single;
    TestDisplay boundDisplay() =>
        WidgetsBinding.instance.platformDispatcher.displays.single as TestDisplay;

    tearDown(() {
      boundDisplay().reset();
    });

    testWidgets('can handle new methods without breaking', (WidgetTester tester) async {
      final dynamic testDisplay = tester.view.display;
      //ignore: avoid_dynamic_calls
      expect(testDisplay.someNewProperty, null);
    });

    testWidgets('can fake devicePixelRatio', (WidgetTester tester) async {
      verifyPropertyFaked<double>(
        tester: tester,
        realValue: trueDisplay().devicePixelRatio,
        fakeValue: trueDisplay().devicePixelRatio + 1,
        propertyRetriever: () => boundDisplay().devicePixelRatio,
        propertyFaker: (_, double fake) {
          boundDisplay().devicePixelRatio = fake;
        },
      );
    });

    testWidgets('can reset devicePixelRatio', (WidgetTester tester) async {
      verifyPropertyReset<double>(
        tester: tester,
        fakeValue: trueDisplay().devicePixelRatio + 1,
        propertyRetriever: () => boundDisplay().devicePixelRatio,
        propertyResetter: () => boundDisplay().resetDevicePixelRatio(),
        propertyFaker: (double fake) {
          boundDisplay().devicePixelRatio = fake;
        },
      );
    });

    testWidgets('resetting devicePixelRatio also resets view.devicePixelRatio', (
      WidgetTester tester,
    ) async {
      verifyPropertyReset(
        tester: tester,
        fakeValue: trueDisplay().devicePixelRatio + 1,
        propertyRetriever: () => tester.view.devicePixelRatio,
        propertyResetter: () => boundDisplay().resetDevicePixelRatio(),
        propertyFaker: (double dpr) => boundDisplay().devicePixelRatio = dpr,
      );
    });

    testWidgets('updating devicePixelRatio also updates view.devicePixelRatio', (
      WidgetTester tester,
    ) async {
      tester.view.display.devicePixelRatio = tester.view.devicePixelRatio + 1;

      expect(tester.view.devicePixelRatio, tester.view.display.devicePixelRatio);
    });

    testWidgets('can fake refreshRate', (WidgetTester tester) async {
      verifyPropertyFaked<double>(
        tester: tester,
        realValue: trueDisplay().refreshRate,
        fakeValue: trueDisplay().refreshRate + 1,
        propertyRetriever: () => boundDisplay().refreshRate,
        propertyFaker: (_, double fake) {
          boundDisplay().refreshRate = fake;
        },
      );
    });

    testWidgets('can reset refreshRate', (WidgetTester tester) async {
      verifyPropertyReset<double>(
        tester: tester,
        fakeValue: trueDisplay().refreshRate + 1,
        propertyRetriever: () => boundDisplay().refreshRate,
        propertyResetter: () => boundDisplay().resetRefreshRate(),
        propertyFaker: (double fake) {
          boundDisplay().refreshRate = fake;
        },
      );
    });

    testWidgets('can fake size', (WidgetTester tester) async {
      verifyPropertyFaked<Size>(
        tester: tester,
        realValue: trueDisplay().size,
        fakeValue: const Size(354, 856),
        propertyRetriever: () => boundDisplay().size,
        propertyFaker: (_, Size fake) {
          boundDisplay().size = fake;
        },
      );
    });

    testWidgets('can reset size', (WidgetTester tester) async {
      verifyPropertyReset<Size>(
        tester: tester,
        fakeValue: const Size(465, 980),
        propertyRetriever: () => boundDisplay().size,
        propertyResetter: () => boundDisplay().resetSize(),
        propertyFaker: (Size fake) {
          boundDisplay().size = fake;
        },
      );
    });

    testWidgets('can reset all values', (WidgetTester tester) async {
      final DisplaySnapshot initial = DisplaySnapshot(tester.view.display);

      tester.view.display.devicePixelRatio = 7;
      tester.view.display.refreshRate = 40;
      tester.view.display.size = const Size(476, 823);

      final DisplaySnapshot faked = DisplaySnapshot(tester.view.display);

      tester.view.display.reset();

      final DisplaySnapshot reset = DisplaySnapshot(tester.view.display);

      expect(initial, isNot(matchesSnapshot(faked)));
      expect(initial, matchesSnapshot(reset));
    });
  });
}

class DisplaySnapshot {
  DisplaySnapshot(Display display)
    : devicePixelRatio = display.devicePixelRatio,
      refreshRate = display.refreshRate,
      id = display.id,
      size = display.size;

  final double devicePixelRatio;
  final double refreshRate;
  final int id;
  final Size size;
}

Matcher matchesSnapshot(DisplaySnapshot expected) => _DisplaySnapshotMatcher(expected);

class _DisplaySnapshotMatcher extends Matcher {
  _DisplaySnapshotMatcher(this.expected);

  final DisplaySnapshot expected;

  @override
  Description describe(Description description) {
    description.add('snapshot of a Display matches');
    return description;
  }

  @override
  Description describeMismatch(
    dynamic item,
    Description mismatchDescription,
    Map<dynamic, dynamic> matchState,
    bool verbose,
  ) {
    assert(item is DisplaySnapshot, 'Can only match against snapshots of Display.');
    final DisplaySnapshot actual = item as DisplaySnapshot;

    if (actual.devicePixelRatio != expected.devicePixelRatio) {
      mismatchDescription.add(
        'actual.devicePixelRatio (${actual.devicePixelRatio}) did not match expected.devicePixelRatio (${expected.devicePixelRatio})',
      );
    }
    if (actual.refreshRate != expected.refreshRate) {
      mismatchDescription.add(
        'actual.refreshRate (${actual.refreshRate}) did not match expected.refreshRate (${expected.refreshRate})',
      );
    }
    if (actual.size != expected.size) {
      mismatchDescription.add(
        'actual.size (${actual.size}) did not match expected.size (${expected.size})',
      );
    }
    if (actual.id != expected.id) {
      mismatchDescription.add(
        'actual.id (${actual.id}) did not match expected.id (${expected.id})',
      );
    }

    return mismatchDescription;
  }

  @override
  bool matches(dynamic item, Map<dynamic, dynamic> matchState) {
    assert(item is DisplaySnapshot, 'Can only match against snapshots of Display.');
    final DisplaySnapshot actual = item as DisplaySnapshot;

    return actual.devicePixelRatio == expected.devicePixelRatio &&
        actual.refreshRate == expected.refreshRate &&
        actual.size == expected.size &&
        actual.id == expected.id;
  }
}
