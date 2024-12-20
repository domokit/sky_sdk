// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:flutter_driver/driver_extension.dart';
import 'package:native_driver/extension.dart';

import 'src/allow_list_devices.dart';

void main() async {
  ensureAndroidOrIosDevice();
  enableFlutterDriverExtension(commands: <CommandExtension>[nativeDriverCommands]);

  // Run on full screen.
  await SystemChrome.setEnabledSystemUIMode(SystemUiMode.immersive);
  runApp(const MainApp());
}

final class MainApp extends StatelessWidget {
  const MainApp({super.key});

  @override
  Widget build(BuildContext context) {
    return const MaterialApp(
      debugShowCheckedModeBanner: false,
      home: AndroidView(viewType: 'blue_orange_gradient_platform_view'),
    );
  }
}
