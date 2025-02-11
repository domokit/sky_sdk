// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:android_driver_extensions/extension.dart';
import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_driver/driver_extension.dart';

import '../src/allow_list_devices.dart';
import '_shared.dart';

void main() async {
  ensureAndroidDevice();
  enableFlutterDriverExtension(commands: <CommandExtension>[nativeDriverCommands]);

  // Run on full screen.
  await SystemChrome.setEnabledSystemUIMode(SystemUiMode.immersive);
  runApp(
    // It is assumed:
    // - The Android SDK version is >= 23 (the test driver checks)
    // - This view does NOT use a SurfaceView
    //
    // See https://github.com/flutter/flutter/blob/main/docs/platforms/android/Android-Platform-Views.md.
    const MainApp(platformView: AndroidView(viewType: 'blue_orange_gradient_platform_view')),
  );
}
