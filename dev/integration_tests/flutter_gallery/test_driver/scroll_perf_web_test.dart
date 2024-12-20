// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_driver/flutter_driver.dart';
import 'package:test/test.dart' hide TypeMatcher, isInstanceOf;

void main() {
  group('scrolling performance test', () {
    late FlutterDriver driver;

    setUpAll(() async {
      driver = await FlutterDriver.connect();
    });

    tearDownAll(() async {
      driver.close();
    });

    test('measure', () async {
      await driver.tap(find.text('Material'));

      final SerializableFinder demoList = find.byValueKey('GalleryDemoList');

      // TODO(eseidel): These are very artificial scrolls, we should use better
      // https://github.com/flutter/flutter/issues/3316
      // Scroll down
      for (int i = 0; i < 5; i++) {
        await driver.scroll(demoList, 0.0, -300.0, const Duration(milliseconds: 300));
        await Future<void>.delayed(const Duration(milliseconds: 500));
      }

      // Scroll up
      for (int i = 0; i < 5; i++) {
        await driver.scroll(demoList, 0.0, 300.0, const Duration(milliseconds: 300));
        await Future<void>.delayed(const Duration(milliseconds: 500));
      }
    }, timeout: Timeout.none);
  });
}
