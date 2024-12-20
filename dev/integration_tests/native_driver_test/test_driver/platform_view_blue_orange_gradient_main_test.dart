// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_driver/flutter_driver.dart';
import 'package:native_driver/native_driver.dart';
import 'package:native_driver/skia_gold.dart';
import 'package:test/test.dart';

import '_luci_skia_gold_prelude.dart';

void main() async {
  // To test the golden file generation locally, comment out the following line.
  // autoUpdateGoldenFiles = true;

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
  });

  tearDownAll(() async {
    await nativeDriver.close();
    await flutterDriver.close();
  });

  test('should screenshot and match a blue -> orange gradient', () async {
    await flutterDriver.waitFor(find.byType('AndroidView'));
    await expectLater(
      nativeDriver.screenshot(),
      matchesGoldenFile('platform_view_blue_orange_gradient_portrait.android.png'),
    );
  }, timeout: Timeout.none);

  test('should rotate landscape and screenshot the gradient', () async {
    await flutterDriver.waitFor(find.byType('AndroidView'));
    await nativeDriver.rotateToLandscape();
    await expectLater(
      nativeDriver.screenshot(),
      matchesGoldenFile('platform_view_blue_orange_gradient_landscape.android.png'),
    );

    await nativeDriver.rotateResetDefault();
    await expectLater(
      nativeDriver.screenshot(),
      matchesGoldenFile('platform_view_blue_orange_gradient_portrait_post_rotation.android.png'),
    );
  }, timeout: Timeout.none);
}
