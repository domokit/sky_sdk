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
/// UPDATE_GOLDENS=1 flutter drive lib/platform_view/hybrid_compoisition_platform_view_main.dart
///
/// # Make your changes.
///
/// # Run the test against baseline.
/// flutter drive lib/platform_view/hybrid_compoisition_platform_view_main.dart
/// ```
///
/// For a convenient way to deflake a test, see `tool/deflake.dart`.
void main() async {
  const String goldenPrefix = 'hybrid_composition_platform_view';

  late final FlutterDriver flutterDriver;
  late final NativeDriver nativeDriver;

  setUpAll(() async {
    if (isLuci) {
      await enableSkiaGoldComparator(namePrefix: 'android_engine_test$goldenVariant');
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
    // TODO(matanlurey): Determining if this is a failure (the screen is always black on CI)
    // or timing dependent (if we would have waited X more seconds it would have rendered).
    // See:
    // - Vulkan: https://github.com/flutter/flutter/issues/162362
    // - OpenGLES: https://github.com/flutter/flutter/issues/162363
    int retriesLeft = 2;
    do {
      try {
        await expectLater(
          nativeDriver.screenshot(),
          matchesGoldenFile('$goldenPrefix.blue_orange_gradient_portrait.png'),
        );
        break;
      } on TestFailure catch (e) {
        if (retriesLeft == 0) {
          rethrow;
        }
        print('Caught: $e. Retrying...');
        retriesLeft--;
      }
    } while (retriesLeft > 0);
  }, timeout: Timeout.none);

  test('should rotate landscape and screenshot the gradient', () async {
    await nativeDriver.rotateToLandscape();
    await expectLater(
      nativeDriver.screenshot(),
      matchesGoldenFile('$goldenPrefix.blue_orange_gradient_landscape_rotated.png'),
    );

    await nativeDriver.rotateResetDefault();
    await expectLater(
      nativeDriver.screenshot(),
      matchesGoldenFile('$goldenPrefix.blue_orange_gradient_portait_rotated_back.png'),
    );
  }, timeout: Timeout.none);
}
