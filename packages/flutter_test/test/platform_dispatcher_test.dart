// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui'
    show
        AccessibilityFeatures,
        Brightness,
        Display,
        FlutterView,
        Locale,
        PlatformDispatcher,
        ViewFocusChangeCallback,
        VoidCallback;

import 'package:flutter/widgets.dart' show WidgetsBinding, WidgetsBindingObserver;
import 'package:flutter_test/flutter_test.dart';

import 'utils/fake_and_mock_utils.dart';

void main() {
  test('TestPlatformDispatcher can handle new methods without breaking', () {
    final dynamic testPlatformDispatcher = TestPlatformDispatcher(
      platformDispatcher: PlatformDispatcher.instance,
    );
    // ignore: avoid_dynamic_calls
    expect(testPlatformDispatcher.someNewProperty, null);
  });

  testWidgets('TestPlatformDispatcher can fake locale', (WidgetTester tester) async {
    verifyPropertyFaked<Locale>(
      tester: tester,
      realValue: PlatformDispatcher.instance.locale,
      fakeValue: const Locale('fake_language_code'),
      propertyRetriever: () {
        return WidgetsBinding.instance.platformDispatcher.locale;
      },
      propertyFaker: (TestWidgetsFlutterBinding binding, Locale fakeValue) {
        binding.platformDispatcher.localeTestValue = fakeValue;
      },
    );
  });

  testWidgets('TestPlatformDispatcher can fake locales', (WidgetTester tester) async {
    verifyPropertyFaked<List<Locale>>(
      tester: tester,
      realValue: PlatformDispatcher.instance.locales,
      fakeValue: <Locale>[const Locale('fake_language_code')],
      propertyRetriever: () {
        return WidgetsBinding.instance.platformDispatcher.locales;
      },
      propertyFaker: (TestWidgetsFlutterBinding binding, List<Locale> fakeValue) {
        binding.platformDispatcher.localesTestValue = fakeValue;
      },
    );
  });

  testWidgets('TestPlatformDispatcher can fake text scale factor', (WidgetTester tester) async {
    verifyPropertyFaked<double>(
      tester: tester,
      realValue: PlatformDispatcher.instance.textScaleFactor,
      fakeValue: 2.5,
      propertyRetriever: () {
        return WidgetsBinding.instance.platformDispatcher.textScaleFactor;
      },
      propertyFaker: (TestWidgetsFlutterBinding binding, double fakeValue) {
        binding.platformDispatcher.textScaleFactorTestValue = fakeValue;
      },
    );
  });

  testWidgets('TestPlatformDispatcher can fake clock format', (WidgetTester tester) async {
    verifyPropertyFaked<bool>(
      tester: tester,
      realValue: PlatformDispatcher.instance.alwaysUse24HourFormat,
      fakeValue: !PlatformDispatcher.instance.alwaysUse24HourFormat,
      propertyRetriever: () {
        return WidgetsBinding.instance.platformDispatcher.alwaysUse24HourFormat;
      },
      propertyFaker: (TestWidgetsFlutterBinding binding, bool fakeValue) {
        binding.platformDispatcher.alwaysUse24HourFormatTestValue = fakeValue;
      },
    );
  });

  testWidgets('TestPlatformDispatcher can fake brieflyShowPassword', (WidgetTester tester) async {
    verifyPropertyFaked<bool>(
      tester: tester,
      realValue: PlatformDispatcher.instance.brieflyShowPassword,
      fakeValue: !PlatformDispatcher.instance.brieflyShowPassword,
      propertyRetriever: () => WidgetsBinding.instance.platformDispatcher.brieflyShowPassword,
      propertyFaker: (TestWidgetsFlutterBinding binding, bool fakeValue) {
        binding.platformDispatcher.brieflyShowPasswordTestValue = fakeValue;
      },
    );
  });

  testWidgets('TestPlatformDispatcher can fake default route name', (WidgetTester tester) async {
    verifyPropertyFaked<String>(
      tester: tester,
      realValue: PlatformDispatcher.instance.defaultRouteName,
      fakeValue: 'fake_route',
      propertyRetriever: () {
        return WidgetsBinding.instance.platformDispatcher.defaultRouteName;
      },
      propertyFaker: (TestWidgetsFlutterBinding binding, String fakeValue) {
        binding.platformDispatcher.defaultRouteNameTestValue = fakeValue;
      },
    );
  });

  testWidgets('TestPlatformDispatcher can fake accessibility features', (
    WidgetTester tester,
  ) async {
    verifyPropertyFaked<AccessibilityFeatures>(
      tester: tester,
      realValue: PlatformDispatcher.instance.accessibilityFeatures,
      fakeValue: const FakeAccessibilityFeatures(),
      propertyRetriever: () {
        return WidgetsBinding.instance.platformDispatcher.accessibilityFeatures;
      },
      propertyFaker: (TestWidgetsFlutterBinding binding, AccessibilityFeatures fakeValue) {
        binding.platformDispatcher.accessibilityFeaturesTestValue = fakeValue;
      },
    );
  });

  testWidgets('TestPlatformDispatcher can fake platform brightness', (WidgetTester tester) async {
    verifyPropertyFaked<Brightness>(
      tester: tester,
      realValue: Brightness.light,
      fakeValue: Brightness.dark,
      propertyRetriever: () {
        return WidgetsBinding.instance.platformDispatcher.platformBrightness;
      },
      propertyFaker: (TestWidgetsFlutterBinding binding, Brightness fakeValue) {
        binding.platformDispatcher.platformBrightnessTestValue = fakeValue;
      },
    );
  });

  testWidgets('TestPlatformDispatcher can clear out fake properties all at once', (
    WidgetTester tester,
  ) async {
    final Locale originalLocale = PlatformDispatcher.instance.locale;
    final double originalTextScaleFactor = PlatformDispatcher.instance.textScaleFactor;
    final TestPlatformDispatcher testPlatformDispatcher =
        retrieveTestBinding(tester).platformDispatcher;

    // Set fake values for window properties.
    testPlatformDispatcher.localeTestValue = const Locale('foobar');
    testPlatformDispatcher.textScaleFactorTestValue = 3.0;

    // Erase fake window property values.
    testPlatformDispatcher.clearAllTestValues();

    // Verify that the window once again reports real property values.
    expect(WidgetsBinding.instance.platformDispatcher.locale, originalLocale);
    expect(WidgetsBinding.instance.platformDispatcher.textScaleFactor, originalTextScaleFactor);
  });

  testWidgets(
    'TestPlatformDispatcher sends fake locales when WidgetsBindingObserver notifiers are called',
    (WidgetTester tester) async {
      final List<Locale> defaultLocales = WidgetsBinding.instance.platformDispatcher.locales;
      final TestObserver observer = TestObserver();
      retrieveTestBinding(tester).addObserver(observer);
      final List<Locale> expectedValue = <Locale>[const Locale('fake_language_code')];
      retrieveTestBinding(tester).platformDispatcher.localesTestValue = expectedValue;
      expect(observer.locales, equals(expectedValue));
      retrieveTestBinding(tester).platformDispatcher.localesTestValue = defaultLocales;
    },
  );

  testWidgets('TestPlatformDispatcher.view getter returns the implicit view', (
    WidgetTester tester,
  ) async {
    expect(
      WidgetsBinding.instance.platformDispatcher.view(id: tester.view.viewId),
      same(tester.view),
    );
  });

  // TODO(pdblasi-google): Removed this group of tests when the Display API is stable and supported on all platforms.
  group('TestPlatformDispatcher with unsupported Display API', () {
    testWidgets('can initialize with empty displays', (WidgetTester tester) async {
      expect(() {
        TestPlatformDispatcher(
          platformDispatcher: _FakePlatformDispatcher(
            displays: <Display>[],
            views: <FlutterView>[_FakeFlutterView()],
          ),
        );
      }, isNot(throwsA(anything)));
    });

    testWidgets('can initialize with mismatched displays', (WidgetTester tester) async {
      expect(() {
        TestPlatformDispatcher(
          platformDispatcher: _FakePlatformDispatcher(
            displays: <Display>[_FakeDisplay(id: 2)],
            views: <FlutterView>[_FakeFlutterView(display: _FakeDisplay(id: 1))],
          ),
        );
      }, isNot(throwsA(anything)));
    });

    testWidgets('creates test views for all views', (WidgetTester tester) async {
      final PlatformDispatcher backingDispatcher = _FakePlatformDispatcher(
        displays: <Display>[],
        views: <FlutterView>[_FakeFlutterView()],
      );
      final TestPlatformDispatcher testDispatcher = TestPlatformDispatcher(
        platformDispatcher: backingDispatcher,
      );

      expect(testDispatcher.views.length, backingDispatcher.views.length);
    });

    group('creates TestFlutterViews', () {
      testWidgets('that defaults to the correct devicePixelRatio', (WidgetTester tester) async {
        const double expectedDpr = 2.5;
        final TestPlatformDispatcher testDispatcher = TestPlatformDispatcher(
          platformDispatcher: _FakePlatformDispatcher(
            displays: <Display>[],
            views: <FlutterView>[_FakeFlutterView(devicePixelRatio: expectedDpr)],
          ),
        );

        expect(testDispatcher.views.single.devicePixelRatio, expectedDpr);
      });

      testWidgets('with working devicePixelRatio setter', (WidgetTester tester) async {
        const double expectedDpr = 2.5;
        const double defaultDpr = 4;
        final TestPlatformDispatcher testDispatcher = TestPlatformDispatcher(
          platformDispatcher: _FakePlatformDispatcher(
            displays: <Display>[],
            views: <FlutterView>[_FakeFlutterView(devicePixelRatio: defaultDpr)],
          ),
        );

        testDispatcher.views.single.devicePixelRatio = expectedDpr;

        expect(testDispatcher.views.single.devicePixelRatio, expectedDpr);
      });

      testWidgets('with working resetDevicePixelRatio', (WidgetTester tester) async {
        const double changedDpr = 2.5;
        const double defaultDpr = 4;
        final TestPlatformDispatcher testDispatcher = TestPlatformDispatcher(
          platformDispatcher: _FakePlatformDispatcher(
            displays: <Display>[],
            views: <FlutterView>[_FakeFlutterView(devicePixelRatio: defaultDpr)],
          ),
        );

        testDispatcher.views.single.devicePixelRatio = changedDpr;
        testDispatcher.views.single.resetDevicePixelRatio();

        expect(testDispatcher.views.single.devicePixelRatio, defaultDpr);
      });
    });
  });
}

class TestObserver with WidgetsBindingObserver {
  List<Locale>? locales;

  @override
  void didChangeLocales(List<Locale>? locales) {
    this.locales = locales;
  }
}

class _FakeDisplay extends Fake implements Display {
  _FakeDisplay({this.id = 0});

  @override
  final int id;
}

class _FakeFlutterView extends Fake implements FlutterView {
  _FakeFlutterView({this.devicePixelRatio = 1, Display? display}) : _display = display;

  @override
  final double devicePixelRatio;

  // This emulates the PlatformDispatcher not having a display on the engine
  // side. We don't have access to the `_displayId` used in the engine to try
  // to find it and can't directly extend `FlutterView` to emulate it closer.
  @override
  Display get display {
    assert(_display != null);
    return _display!;
  }

  final Display? _display;

  @override
  final int viewId = 1;
}

class _FakePlatformDispatcher extends Fake implements PlatformDispatcher {
  _FakePlatformDispatcher({required this.displays, required this.views});
  @override
  final Iterable<Display> displays;

  @override
  final Iterable<FlutterView> views;

  @override
  VoidCallback? onMetricsChanged;

  @override
  ViewFocusChangeCallback? onViewFocusChange;
}
