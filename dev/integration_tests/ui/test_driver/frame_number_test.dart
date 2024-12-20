// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_driver/flutter_driver.dart';
import 'package:test/test.dart' hide TypeMatcher, isInstanceOf;

bool _isNumeric(String s) {
  return double.tryParse(s) != null;
}

// Connect and disconnect from the empty app.
void main() {
  group('FrameNumber', () {
    late FlutterDriver driver;

    setUpAll(() async {
      driver = await FlutterDriver.connect();
    });

    tearDownAll(() async {
      await driver.close();
    });

    test('minFrameNumber is numeric', () async {
      final SerializableFinder minFrameNumberFinder = find.byValueKey('minFrameNumber');
      await driver.waitFor(minFrameNumberFinder, timeout: const Duration(seconds: 5));
      final String minFrameNumber = await driver.getText(minFrameNumberFinder);
      // TODO(iskakaushik): enable the stronger check of _minFrameNumber == '1',
      // once this is fixed. https://github.com/flutter/flutter/issues/86487
      expect(_isNumeric(minFrameNumber), true);
    }, timeout: Timeout.none);
  });
}
