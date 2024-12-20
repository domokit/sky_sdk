// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:flutter_driver/flutter_driver.dart';
import 'package:test/test.dart';
import 'package:webdriver/async_io.dart';

// TODO(web): Migrate this test to a normal integration_test with a WidgetTester.

/// The following test is used as a simple smoke test for verifying Flutter
/// Framework and Flutter Web Engine integration.
void main() {
  group('Hello World App', () {
    final SerializableFinder titleFinder = find.byValueKey('title');

    late FlutterDriver driver;

    // Connect to the Flutter driver before running any tests.
    setUpAll(() async {
      driver = await FlutterDriver.connect();
    });

    // Close the connection to the driver after the tests have completed.
    tearDownAll(() async {
      driver.close();
    });

    test('title is correct', () async {
      expect(await driver.getText(titleFinder), 'Hello, world!');
    });

    test('enable accessibility', () async {
      await driver.setSemantics(true);

      // TODO(ianh): this delay violates our style guide. We should instead wait for a triggering event.
      await Future<void>.delayed(const Duration(seconds: 2));

      final WebElement? fltSemantics =
          await driver.webDriver.execute(
                'return document.querySelector("flt-semantics")',
                <dynamic>[],
              )
              as WebElement?;
      expect(fltSemantics, isNotNull);
    });
  });
}
