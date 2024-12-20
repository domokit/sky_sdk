// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:typed_data';

import 'package:flutter_driver/flutter_driver.dart';
import 'package:test/test.dart' hide TypeMatcher, isInstanceOf;

void main() {
  late FlutterDriver driver;

  setUpAll(() async {
    driver = await FlutterDriver.connect();
    await driver.waitUntilFirstFrameRasterized();
  });

  test('Can render solid red', () async {
    // RGBA Encoded Bytes.
    final Uint8List data =
        (await driver.screenshot(format: ScreenshotFormat.rawStraightRgba)) as Uint8List;

    expect(data[0] << 24 | data[1] << 16 | data[2] << 8 | data[3], 0xFF0000FF);
  }, timeout: Timeout.none);

  tearDownAll(() async {
    await driver.close();
  });
}
