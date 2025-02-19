// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert';

import 'package:android_driver_extensions/native_driver.dart';
import 'package:flutter_driver/flutter_driver.dart';
import 'package:test/test.dart';

late final FlutterDriver flutterDriver;
late final NativeDriver nativeDriver;

void main() async {
  setUpAll(() async {
    flutterDriver = await FlutterDriver.connect();
    nativeDriver = await AndroidNativeDriver.connect(flutterDriver);
  });

  tearDownAll(() async {
    await nativeDriver.close();
    await flutterDriver.close();
  });

  // TODO(matanlurey): Convert to use package:integration_test
  test('verify that engineId is set and works', () async {
    final Map<String, Object?> response =
        json.decode(await flutterDriver.requestData('')) as Map<String, Object?>;
    expect(
      response['engineId'],
      1,
      // Valid engine ids start at 1 to make detecting uninitialized
      // values easier.
      reason: 'engineId of first engine instance should be 1',
    );
  }, timeout: Timeout.none);
}
