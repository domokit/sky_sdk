// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:android_driver_extensions/native_driver.dart';
import 'package:android_driver_extensions/skia_gold.dart';
import 'package:flutter_driver/flutter_driver.dart';
import 'package:test/test.dart';

import '_luci_skia_gold_prelude.dart';

void main() async {
  // To test the golden file generation locally, comment out the following line.
  // autoUpdateGoldenFiles = true;

  const String appName = 'com.example.android_engine_test';
  late final FlutterDriver flutterDriver;
  late final NativeDriver nativeDriver;

  setUpAll(() async {
    if (isLuci) {
      await enableSkiaGoldComparator();
    }
    flutterDriver = await FlutterDriver.connect();
    nativeDriver = await AndroidNativeDriver.connect(flutterDriver);
    await nativeDriver.configureForScreenshotTesting();
  });

  tearDownAll(() async {
    await nativeDriver.close();
    await flutterDriver.close();
  });

  test('should screenshot and match an external smiley face texture', () async {
    await flutterDriver.waitFor(find.byType('Texture'));

    // On Android: Background the app, trim memory, and restore the app.
    if (nativeDriver case final AndroidNativeDriver nativeDriver) {
      print('Backgrounding the app, trimming memory, and resuming the app.');
      await nativeDriver.backgroundApp();

      print('Trimming memory.');
      await nativeDriver.simulateLowMemory(appName: appName);

      print('Resuming the app.');
      await nativeDriver.resumeApp(appName: appName);
    }

    await expectLater(
      nativeDriver.screenshot(),
      matchesGoldenFile('external_texture_smiley_face.android.png'),
    );
  }, timeout: Timeout.none);
}
