// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:android_driver_extensions/native_driver.dart';
import 'package:android_driver_extensions/skia_gold.dart';
import 'package:flutter_driver/flutter_driver.dart';
import 'package:test/test.dart';

import '../_luci_skia_gold_prelude.dart';

void main() async {
  // To test the golden file generation locally, comment out the following line.
  // autoUpdateGoldenFiles = true;

  const String goldenPrefix = 'texture_layer_hybrid_composition_platform_view';

  late final FlutterDriver flutterDriver;
  late final NativeDriver nativeDriver;

  setUpAll(() async {
    if (isLuci) {
      await enableSkiaGoldComparator();
    }
    flutterDriver = await FlutterDriver.connect();
    nativeDriver = await AndroidNativeDriver.connect(flutterDriver);
    await nativeDriver.configureForScreenshotTesting();
    await flutterDriver.waitUntilFirstFrameRasterized();

    // Double check that we are really probably testing using TLHC.
    // See https://github.com/flutter/flutter/blob/main/docs/platforms/android/Android-Platform-Views.md.
    if (await nativeDriver.sdkVersion case final int version when version < 23) {
      fail('Requires SDK >= 23, got $version');
    }
  });

  tearDownAll(() async {
    await nativeDriver.close();
    await flutterDriver.close();
  });

  test('should screenshot and match a blue -> orange gradient', () async {
    await flutterDriver.waitFor(find.byType('AndroidView'));
    await expectLater(
      nativeDriver.screenshot(),
      matchesGoldenFile('$goldenPrefix.blue_orange_gradient_portrait.android.png'),
    );
  }, timeout: Timeout.none);

  test('should rotate landscape and screenshot the gradient', () async {
    await flutterDriver.waitFor(find.byType('AndroidView'));
    await nativeDriver.rotateToLandscape();
    await expectLater(
      nativeDriver.screenshot(),
      matchesGoldenFile('$goldenPrefix.blue_orange_gradient_landscape_rotated.android.png'),
    );

    await nativeDriver.rotateResetDefault();
    await expectLater(
      nativeDriver.screenshot(),
      matchesGoldenFile('$goldenPrefix.blue_orange_gradient_portait_rotated_back.android.png'),
    );
  }, timeout: Timeout.none);
}
