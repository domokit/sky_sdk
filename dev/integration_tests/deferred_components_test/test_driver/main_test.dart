// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_driver/flutter_driver.dart';
import 'package:test/test.dart' hide TypeMatcher, isInstanceOf;

Future<void> main() async {
  late FlutterDriver driver;

  setUpAll(() async {
    driver = await FlutterDriver.connect();
  });

  tearDownAll(() {
    driver.close();
  });

  // Run `run_release_test.sh` to also test release engine deferred components code. This
  // drive test runs as debug and thus only tests framework side deferred components handling.
  test('Install and load deferred component', () async {
    final String preloadText = await driver.getText(find.byValueKey('PreloadText'));
    expect(preloadText, 'preload');

    final SerializableFinder fab = find.byValueKey('FloatingActionButton');
    await driver.tap(fab);

    final String placeholderText = await driver.getText(find.byValueKey('PlaceholderText'));
    expect(placeholderText, 'placeholder');

    await driver.waitFor(find.byValueKey('DeferredWidget'));

    final String deferredText = await driver.getText(find.byValueKey('DeferredWidget'));
    expect(deferredText, 'DeferredWidget');
    await driver.waitFor(find.byValueKey('DeferredImage'));
  }, timeout: Timeout.none);
}
