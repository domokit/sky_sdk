// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_driver/flutter_driver.dart';
import 'package:test/test.dart' hide TypeMatcher, isInstanceOf;

void main() {
  group('button tap test', () {
    late FlutterDriver driver;

    setUpAll(() async {
      driver = await FlutterDriver.connect();
    });

    tearDownAll(() async {
      driver.close();
    });

    test('tap on the button, verify result', () async {
      final SerializableFinder batteryLevelLabel = find.byValueKey('Battery level label');
      expect(batteryLevelLabel, isNotNull);

      final SerializableFinder button = find.text('Refresh');
      await driver.waitFor(button);
      await driver.tap(button);

      String? batteryLevel;
      while (batteryLevel == null || batteryLevel.contains('unknown')) {
        batteryLevel = await driver.getText(batteryLevelLabel);
      }

      // Allow either a battery percentage or "No battery" since it will vary
      // by device; either indicates that a known response came from the host
      // implementation.
      expect(batteryLevel.contains('%') || batteryLevel.contains('No battery'), isTrue);
    });
  });
}
