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

    test('Textfield scrolls back into view after covered by keyboard', () async {
      await driver.setTextEntryEmulation(enabled: false); // we want the keyboard to come up

      final SerializableFinder listViewFinder = find.byValueKey(keys.kListView);
      final SerializableFinder textFieldFinder = find.byValueKey(keys.kDefaultTextField);
      final SerializableFinder offsetFinder = find.byValueKey(keys.kOffsetText);
      final SerializableFinder keyboardVisibilityIndicatorFinder = find.byValueKey(
        keys.kKeyboardVisibleView,
      );

      // Align TextField with bottom edge to ensure it would be covered when keyboard comes up.
      await driver.waitForAbsent(textFieldFinder);
      await driver.scrollUntilVisible(
        listViewFinder,
        textFieldFinder,
        alignment: 1.0,
        dyScroll: -20.0,
      );
      await driver.waitFor(textFieldFinder);
      final double scrollOffsetWithoutKeyboard = double.parse(await driver.getText(offsetFinder));

      // Bring up keyboard
      await driver.tap(textFieldFinder);

      // The blinking cursor may have animation. Do not wait for it to finish.
      await driver.runUnsynchronized(() async {
        await driver.waitFor(keyboardVisibilityIndicatorFinder);
      });

      // Ensure that TextField is visible again
      await driver.waitFor(textFieldFinder);
      final double scrollOffsetWithKeyboard = double.parse(await driver.getText(offsetFinder));

      // Ensure the scroll offset changed appropriately when TextField scrolled back into view.
      expect(scrollOffsetWithKeyboard, greaterThan(scrollOffsetWithoutKeyboard));
    }, timeout: Timeout.none);
  });
}
