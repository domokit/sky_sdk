// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart' hide TypeMatcher, isInstanceOf;
import 'package:ui/src/engine.dart';
import 'package:ui/ui_web/src/ui_web.dart' as ui_web;

import '../common/matchers.dart';
import '../common/test_initialization.dart';

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

Future<void> testMain() async {
  setUpUnitTests();

  setUp(() {
    ui_web.debugResetCustomUrlStrategy();
  });
  tearDown(() {
    ui_web.debugResetCustomUrlStrategy();
  });

  test('uses the default if no custom URL strategy is set', () {
    final ui_web.UrlStrategy defaultUrlStrategy = TestUrlStrategy();
    ui_web.debugDefaultUrlStrategyOverride = defaultUrlStrategy;

    expect(ui_web.urlStrategy, defaultUrlStrategy);
    expect(ui_web.isCustomUrlStrategySet, isFalse);
  });

  test('can set a custom URL strategy', () {
    final TestUrlStrategy customUrlStrategy = TestUrlStrategy();
    ui_web.urlStrategy = customUrlStrategy;

    expect(ui_web.urlStrategy, customUrlStrategy);
    expect(ui_web.isCustomUrlStrategySet, isTrue);
    // Does not allow custom URL strategy to be set again.
    expect(() {
      ui_web.urlStrategy = customUrlStrategy;
    }, throwsAssertionError);
  });

  test('custom URL strategy can be prevented manually', () {
    ui_web.preventCustomUrlStrategy();

    expect(ui_web.isCustomUrlStrategySet, isFalse);
    expect(() {
      ui_web.urlStrategy = TestUrlStrategy();
    }, throwsAssertionError);
  });
}
