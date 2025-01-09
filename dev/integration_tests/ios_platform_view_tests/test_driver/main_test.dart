// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_driver/flutter_driver.dart';

import 'package:test/test.dart' hide TypeMatcher, isInstanceOf;

void main() {
  group('FlutterDriver', () {
    late FlutterDriver driver;

    setUpAll(() async {
      driver = await FlutterDriver.connect();
    });

    tearDownAll(() => driver.close());

    test('Merge thread to create and remove platform views should not crash', () async {
      // Start pushing in a page with platform view, merge threads.
      final SerializableFinder platformViewButton = find.byValueKey('platform_view_button');
      await driver.waitFor(platformViewButton);
      await driver.tap(platformViewButton);

      // Wait for the platform view page to show.
      final SerializableFinder plusButton = find.byValueKey('plus_button');
      await driver.waitFor(plusButton);
      await driver.waitUntilNoTransientCallbacks();

      // Tapping an elevated button runs an animation that pumps enough frames to un-merge the threads.
      await driver.tap(plusButton);
      await driver.waitUntilNoTransientCallbacks();

      // Remove the page with platform view, merge threads again.
      final SerializableFinder backButton = find.pageBack();
      await driver.tap(backButton);
      await driver.waitUntilNoTransientCallbacks();

      final Health driverHealth = await driver.checkHealth();
      expect(driverHealth.status, HealthStatus.ok);
    }, timeout: Timeout.none);

    test('Merge thread to create and remove platform views should not crash', () async {
      // Start pushing in a page with platform view, merge threads.
      final SerializableFinder platformViewButton = find.byValueKey('platform_view_button');
      await driver.waitFor(platformViewButton);
      await driver.tap(platformViewButton);
      await driver.waitUntilNoTransientCallbacks();

      // Remove the page with platform view, threads are still merged.
      final SerializableFinder backButton = find.pageBack();
      await driver.tap(backButton);
      await driver.waitUntilNoTransientCallbacks();

      // The animation of tapping a `ElevatedButton` should pump enough frames to un-merge the thread.
      final SerializableFinder unmergeButton = find.byValueKey('unmerge_button');
      await driver.waitFor(unmergeButton);
      await driver.tap(unmergeButton);
      await driver.waitUntilNoTransientCallbacks();

      final Health driverHealth = await driver.checkHealth();
      expect(driverHealth.status, HealthStatus.ok);
    }, timeout: Timeout.none);
  });
}
