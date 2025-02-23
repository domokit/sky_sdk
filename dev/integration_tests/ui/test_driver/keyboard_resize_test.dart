// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_driver/flutter_driver.dart';
import 'package:integration_ui/keys.dart' as keys;
import 'package:test/test.dart' hide TypeMatcher, isInstanceOf;

void main() {
  group('end-to-end test', () {
    late FlutterDriver driver;

    setUpAll(() async {
      driver = await FlutterDriver.connect();
    });

    tearDownAll(() async {
      await driver.close();
    });

    test('Ensure keyboard dismissal resizes the view to original size', () async {
      final SerializableFinder heightText = find.byValueKey(keys.kHeightText);
      await driver.waitFor(heightText);

      // Measure the initial height.
      final String startHeight = await driver.getText(heightText);

      // Focus the text field to show the keyboard.
      final SerializableFinder defaultTextField = find.byValueKey(keys.kDefaultTextField);
      await driver.waitFor(defaultTextField);

      // The only practical way to detect the software keyboard opening or closing
      // is to use polling and wait for the layout to change.
      // We pick a short polling interval to speed up the test for most devices.
      // In local tests, Pixel 8 Pro API 36 usually only one poll iteration is needed,
      // older device like Galaxy Tab S3 API 28 takes 2-3 iterations.
      const Duration pollDelay300Ms = Duration(milliseconds: 300);

      bool heightTextDidShrink = false;
      // TODO(harri35): Reconsider this polling duration when the root cause is found.
      // Sometimes it can take up to 21.3 seconds for the keyboard to open,
      // so we allow ample time here (200 * pollDelay300Ms = 60 sec)
      for (int i = 0; i < 200; ++i) {
        await driver.tap(defaultTextField);
        await Future<void>.delayed(pollDelay300Ms);
        // Measure the height with keyboard displayed.
        final String heightWithKeyboardShown = await driver.getText(heightText);
        if (double.parse(heightWithKeyboardShown) < double.parse(startHeight)) {
          heightTextDidShrink = true;
          break;
        }
      }
      expect(heightTextDidShrink, isTrue);

      // Unfocus the text field to dismiss the keyboard.
      final SerializableFinder unfocusButton = find.byValueKey(keys.kUnfocusButton);
      await driver.waitFor(unfocusButton);
      await driver.tap(unfocusButton);

      bool heightTextDidExpand = false;
      for (int i = 0; i < 10; ++i) {
        await Future<void>.delayed(pollDelay300Ms);
        // Measure the final height.
        final String endHeight = await driver.getText(heightText);
        if (endHeight == startHeight) {
          heightTextDidExpand = true;
          break;
        }
      }
      expect(heightTextDidExpand, isTrue);
    }, timeout: Timeout.none);
  });
}
