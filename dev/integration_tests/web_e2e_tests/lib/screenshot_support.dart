// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math';

import 'package:flutter_driver/flutter_driver.dart';
import 'package:integration_test/integration_test_driver_extended.dart' as test;

/// Browser screen dimensions for the FlutterDriver test.
const int _kScreenshotWidth = 1024;
const int _kScreenshotHeight = 1024;

/// Convenience wrapper around [test.integrationDriver].
///
/// Adds the capability to take test screenshots.
Future<void> runTestWithScreenshots({
  int browserWidth = _kScreenshotWidth,
  int browserHeight = _kScreenshotHeight,
}) async {
  final WebFlutterDriver driver = await FlutterDriver.connect() as WebFlutterDriver;

  (await driver.webDriver.window).setSize(Rectangle<int>(0, 0, browserWidth, browserHeight));

  test.integrationDriver(
    driver: driver,
    onScreenshot: (
      String screenshotName,
      List<int> screenshotBytes, [
      Map<String, Object?>? args,
    ]) async {
      // TODO(yjbanov): implement, see https://github.com/flutter/flutter/issues/86120
      return true;
    },
  );
}
