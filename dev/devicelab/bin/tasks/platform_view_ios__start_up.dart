// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io';

import 'package:flutter_devicelab/framework/devices.dart';
import 'package:flutter_devicelab/framework/framework.dart';
import 'package:flutter_devicelab/framework/utils.dart';
import 'package:flutter_devicelab/tasks/perf_tests.dart';

Future<void> main() async {
  deviceOperatingSystem = DeviceOperatingSystem.ios;
  await task(() async {
    final String platformViewDirectoryPath = '${flutterDirectory.path}/examples/platform_view';
    final Directory platformViewDirectory = dir(platformViewDirectoryPath);
    await inDirectory(platformViewDirectory, () async {
      await flutter('pub', options: <String>['get']);
      // Pre-cache the iOS artifacts; this may be the first test run on this machine.
      await flutter(
        'precache',
        options: <String>[
          '--no-android',
          '--no-fuchsia',
          '--no-linux',
          '--no-macos',
          '--no-web',
          '--no-windows',
        ],
      );
    });

    final TaskFunction taskFunction = createPlatformViewStartupTest();
    return taskFunction();
  });
}
