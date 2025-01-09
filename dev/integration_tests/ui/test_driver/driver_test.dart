// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:flutter_driver/flutter_driver.dart';

import 'package:test/test.dart' hide TypeMatcher, isInstanceOf;

void main() {
  group('FlutterDriver', () {
    final SerializableFinder presentText = find.text('present');
    late FlutterDriver driver;

    setUpAll(() async {
      driver = await FlutterDriver.connect();
    });

    tearDownAll(() async {
      await driver.close();
    });

    test('waitFor should find text "present"', () async {
      await driver.waitFor(presentText);
    }, timeout: Timeout.none);

    test('waitForAbsent should time out waiting for text "present" to disappear', () async {
      await expectLater(
        () => driver.waitForAbsent(presentText, timeout: const Duration(seconds: 1)),
        throwsA(
          isA<DriverError>().having(
            (DriverError error) => error.message,
            'message',
            contains('Timeout while executing waitForAbsent'),
          ),
        ),
      );
    }, timeout: Timeout.none);

    test('waitForAbsent should resolve when text "present" disappears', () async {
      // Begin waiting for it to disappear
      final Completer<void> whenWaitForAbsentResolves = Completer<void>();
      driver
          .waitForAbsent(presentText)
          .then(
            whenWaitForAbsentResolves.complete,
            onError: whenWaitForAbsentResolves.completeError,
          );

      // Wait 1 second then make it disappear
      await Future<void>.delayed(const Duration(seconds: 1));
      await driver.tap(find.byValueKey('togglePresent'));

      // Ensure waitForAbsent resolves
      await whenWaitForAbsentResolves.future;
    }, timeout: Timeout.none);

    test('waitFor times out waiting for "present" to reappear', () async {
      await expectLater(
        () => driver.waitFor(presentText, timeout: const Duration(seconds: 1)),
        throwsA(
          isA<DriverError>().having(
            (DriverError error) => error.message,
            'message',
            contains('Timeout while executing waitFor'),
          ),
        ),
      );
    }, timeout: Timeout.none);

    test('waitFor should resolve when text "present" reappears', () async {
      // Begin waiting for it to reappear
      final Completer<void> whenWaitForResolves = Completer<void>();
      driver
          .waitFor(presentText)
          .then(whenWaitForResolves.complete, onError: whenWaitForResolves.completeError);

      // Wait 1 second then make it appear
      await Future<void>.delayed(const Duration(seconds: 1));
      await driver.tap(find.byValueKey('togglePresent'));

      // Ensure waitFor resolves
      await whenWaitForResolves.future;
    }, timeout: Timeout.none);

    test('waitForAbsent resolves immediately when the element does not exist', () async {
      await driver.waitForAbsent(find.text('that does not exist'));
    }, timeout: Timeout.none);

    test('uses hit test to determine tappable elements', () async {
      final SerializableFinder a = find.byValueKey('a');
      final SerializableFinder menu = find.byType('_DropdownMenu<Letter>');

      // Dropdown is closed
      await driver.waitForAbsent(menu);

      // Open dropdown
      await driver.tap(a);
      await driver.waitFor(menu);

      // Close it again
      await driver.tap(a);
      await driver.waitForAbsent(menu);
    }, timeout: Timeout.none);

    test('enters text in a text field', () async {
      final SerializableFinder textField = find.byValueKey('enter-text-field');
      await driver.tap(textField);
      await driver.enterText('Hello!');
      await driver.waitFor(find.text('Hello!'));
      await driver.enterText('World!');
      await driver.waitFor(find.text('World!'));
    }, timeout: Timeout.none);
  });
}
