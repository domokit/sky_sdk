// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io' as io;
import 'dart:typed_data';

import 'package:android_driver_extensions/native_driver.dart';
import 'package:test/test.dart';

import 'src/fake_adb.dart';
import 'src/fake_driver.dart';

void main() async {
  test('screenshot calls ADB screencap', () async {
    final FakeAdb adb = FakeAdb(screencap: () async => Uint8List.fromList(<int>[1, 2, 3, 4]));
    final io.Directory tmpDir = io.Directory.systemTemp.createTempSync();
    final AndroidNativeDriver driver = AndroidNativeDriver.forTesting(
      adb: adb,
      driver: const NullFlutterDriver(),
      tempDirectory: tmpDir,
    );

    final NativeScreenshot screenshot = await driver.screenshot();
    await expectLater(screenshot.readAsBytes(), completion(<int>[1, 2, 3, 4]));

    final String path = await screenshot.saveAs();
    final io.File file = io.File(path);
    expect(file.readAsBytes(), completion(<int>[1, 2, 3, 4]));

    await driver.close();
  });

  test('closes deletes the temporary directory', () async {
    final FakeAdb adb = FakeAdb();
    final io.Directory tmpDir = io.Directory.systemTemp.createTempSync();
    final AndroidNativeDriver driver = AndroidNativeDriver.forTesting(
      adb: adb,
      driver: const NullFlutterDriver(),
      tempDirectory: tmpDir,
    );

    expect(tmpDir.existsSync(), isTrue);
    await driver.close();
    expect(tmpDir.existsSync(), isFalse);
  });
}
