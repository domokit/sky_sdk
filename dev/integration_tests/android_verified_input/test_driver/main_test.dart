// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:io';
import 'package:flutter_driver/flutter_driver.dart';
import 'package:test/test.dart';

Future<void> main() async {
  late FlutterDriver driver;

  setUpAll(() async {
    driver = await FlutterDriver.connect();
  });

  tearDownAll(() {
    driver.close();
  });

  test('verified input', () async {
    // Wait for the PlatformView to show up.
    await driver.waitFor(find.byValueKey('PlatformView'));
    final DriverOffset offset = await driver.getCenter(find.byValueKey('PlatformView'));

    // This future will complete when the input event is verified or fails
    // to be verified.
    final Future<String> inputEventWasVerified = driver.requestData('input_was_verified');

    // Keep issuing taps until we get the requested data. The actual setup
    // of the platform view is asynchronous so we might have to tap more than
    // once to  get a response.
    bool stop = false;
    inputEventWasVerified.whenComplete(() => stop = true);
    while (!stop) {
      // We must use the Android input tool to get verified input events.
      final ProcessResult result = await Process.run('adb', <String>[
        'shell',
        'input',
        'tap',
        '${offset.dx}',
        '${offset.dy}',
      ]);
      expect(result.exitCode, equals(0));
    }
    // Input
    expect(await inputEventWasVerified, equals('true'));
  }, timeout: Timeout.none);
}
