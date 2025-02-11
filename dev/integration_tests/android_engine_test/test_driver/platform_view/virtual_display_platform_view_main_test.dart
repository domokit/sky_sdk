// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:android_driver_extensions/native_driver.dart';
import 'package:android_driver_extensions/skia_gold.dart';
import 'package:flutter_driver/flutter_driver.dart';
import 'package:test/test.dart';

import '../_luci_skia_gold_prelude.dart';
import '../_unstable_gold_retry.dart';

/// For local debugging, a (local) golden-file is required as a baseline:
///
/// ```sh
/// # Checkout HEAD, i.e. *before* changes you want to test.
/// UPDATE_GOLDENS=1 flutter drive lib/platform_view/virtual_display_platform_view_main.dart
///
/// # Make your changes.
///
/// # Run the test against baseline.
/// flutter drive lib/platform_view/virtual_display_platform_view_main.dart
/// ```
///
/// For a convenient way to deflake a test, see `tool/deflake.dart`.
void main() async {
  const String goldenPrefix = 'virtual_display_platform_view';

  late final FlutterDriver flutterDriver;
  late final NativeDriver nativeDriver;

  late final bool isEmulator;
  late final bool isVulkan;

  setUpAll(() async {
    if (isLuci) {
      await enableSkiaGoldComparator(namePrefix: 'android_engine_test$goldenVariant');
    }
    flutterDriver = await FlutterDriver.connect();
    nativeDriver = await AndroidNativeDriver.connect(flutterDriver);
    await nativeDriver.configureForScreenshotTesting();
    await flutterDriver.waitUntilFirstFrameRasterized();

    // Double check that we are really probably testing using Virtual Display.
    // See https://github.com/flutter/flutter/blob/main/docs/platforms/android/Android-Platform-Views.md.
    if (await nativeDriver.sdkVersion case final int version when version < 23) {
      fail('Requires SDK >= 23, got $version');
    }

    // TODO(matanlurey): https://github.com/flutter/flutter/issues/162362#issuecomment-2649555821.
    isEmulator = await nativeDriver.isEmulator;
    isVulkan = goldenVariant.contains('vulkan');
    if (isEmulator && isVulkan) {
      print('Detected running on a vulkan emulator. Will retry certain failures');
    }
  });

  tearDownAll(() async {
    await flutterDriver.tap(find.byValueKey('AddOverlay'));

    await nativeDriver.close();
    await flutterDriver.close();
  });

  test('should screenshot and match a blue -> orange gradient', () async {
    await expectLater(
      nativeDriver.screenshot(),
      matchesGoldenFile('$goldenPrefix.blue_orange_gradient_portrait.png'),
    );
  }, timeout: Timeout.none);

  test('should rotate landscape and screenshot the gradient', () async {
    await nativeDriver.rotateToLandscape();
    await expectLater(
      nativeDriver.screenshot(),
      matchesGoldenFileWithRetries(
        '$goldenPrefix.blue_orange_gradient_landscape_rotated.png',
        retries: isEmulator && isVulkan ? 2 : 0,
      ),
    );

    await nativeDriver.rotateResetDefault();
    await expectLater(
      nativeDriver.screenshot(),
      matchesGoldenFileWithRetries(
        '$goldenPrefix.blue_orange_gradient_portait_rotated_back.png',
        retries: isEmulator && isVulkan ? 2 : 0,
      ),
    );
  }, timeout: Timeout.none);

  test('should hide overlay layer', () async {
    await flutterDriver.tap(find.byValueKey('RemoveOverlay'));
    await Future<void>.delayed(const Duration(seconds: 1));

    await expectLater(
      nativeDriver.screenshot(),
      matchesGoldenFile('$goldenPrefix.hide_overlay.png'),
    );
  }, timeout: Timeout.none);
}
