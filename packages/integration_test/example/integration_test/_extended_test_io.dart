// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// This is a basic Flutter widget test.
//
// To perform an interaction with a widget in your test, use the WidgetTester
// utility that Flutter provides. For example, you can send tap and scroll
// gestures. You can also use WidgetTester to find child widgets in the widget
// tree, read text, and verify that the values of widget properties are correct.

import 'dart:io';
import 'package:flutter/foundation.dart';
import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:integration_test/integration_test.dart';

import 'package:integration_test_example/main.dart' as app;

import 'package:path/path.dart' as p;

void main() {
  final IntegrationTestWidgetsFlutterBinding binding =
      IntegrationTestWidgetsFlutterBinding.ensureInitialized();

  String _adbPath() {
  final String? androidHome =
      Platform.environment['ANDROID_HOME'] ?? Platform.environment['ANDROID_SDK_ROOT'];
  if (androidHome == null) {
    return 'adb';
  } else {
    return p.join(androidHome, 'platform-tools', 'adb');
  }
}

  // CAMILLE: would we need a driver?
  testWidgets('verify screenshot', (WidgetTester tester) async {
    /////// SANITY CHECK:
    /////////////// CAMILLE: this will not work because test runs on device
    // print('>>>> CAMILLE adb path: ${_adbPath()}'); // equals 'adb' because androidHome is null
    // final Process run = await Process.start(_adbPath(), const <String>[
    //     'shell',
    //     'screencap',
    //   ]);
    // await run.exitCode;
    ////////////////////////////////////////////////////////////////////////
    // CAMILLE: we may be able to call screncap without adb shell, adb shell means run this command on the device
    // if dart:ui is used it has to be on device (flutter, flutter_test -- flutter_tester device, integration_test) (so if any are in transitive deps, you know what it is)
    // flutter driver is a termainl, we know what vm service is being used on the app so let's connect to it and do stuff, so when we see FlutterDriver.connect() all this is doing
    // is looking at env var (FLUTTER_VM_URL) and connecting to app via VM service protocol. so really flutter_driver test is a dart script and you are running from host machine.
    // there, adb is available. platform channels or ffi or dart:ui, then it has to be integration_test, flutter, flutter_test (things running on device)

    // Build our app.
    app.main();

    // On Android, this is required prior to taking the screenshot.
    await binding.convertFlutterSurfaceToImage();

    // Pump a frame before taking the screenshot.
    await tester.pumpAndSettle();
    final List<int> firstPng = await binding.takeScreenshot('first');
    expect(firstPng.isNotEmpty, isTrue);

    // Pump another frame before taking the screenshot.
    await tester.pumpAndSettle();
    final List<int> secondPng = await binding.takeScreenshot('second');
    expect(secondPng.isNotEmpty, isTrue);

    expect(listEquals(firstPng, secondPng), isTrue);
  });
}
