// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:android_driver_extensions/native_driver.dart';
import 'package:android_driver_extensions/skia_gold.dart';
import 'package:flutter_driver/flutter_driver.dart';
import 'package:test/test.dart';

import '../_luci_skia_gold_prelude.dart';

/// For local debugging, a (local) golden-file is required as a baseline:
///
/// ```sh
/// # Checkout HEAD, i.e. *before* changes you want to test.
/// UPDATE_GOLDENS=1 flutter drive lib/external_texture/surface_producer_smiley_face_main.dart
///
/// # Make your changes.
///
/// # Run the test against baseline.
/// flutter drive lib/external_texture/surface_producer_smiley_face_main.dart
/// ```
///
/// For a convenient way to deflake a test, see `tool/deflake.dart`.
void main() async {
  const String appName = 'com.example.android_engine_test';
  late final FlutterDriver flutterDriver;
  late final NativeDriver nativeDriver;

  setUpAll(() async {
    if (isLuci) {
      await enableSkiaGoldComparator(namePrefix: 'android_engine_test$goldenVariant');
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
      matchesGoldenFile('external_texture_surface_producer_smiley_face.png'),
    );
  }, timeout: Timeout.none);
}
