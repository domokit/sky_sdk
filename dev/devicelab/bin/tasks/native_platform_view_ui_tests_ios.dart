// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_devicelab/framework/devices.dart';
import 'package:flutter_devicelab/framework/framework.dart';
import 'package:flutter_devicelab/framework/ios.dart';
import 'package:flutter_devicelab/framework/task_result.dart';
import 'package:flutter_devicelab/framework/utils.dart';
import 'package:path/path.dart' as path;

Future<void> main() async {
  deviceOperatingSystem = DeviceOperatingSystem.ios;
  await task(() async {
    final String projectDirectory =
        '${flutterDirectory.path}/dev/integration_tests/ios_platform_view_tests';

    await inDirectory(projectDirectory, () async {
      // To address "Failed to terminate" failure.
      section('Uninstall previously installed app');

      await flutter('install', options: <String>['--uninstall-only']);

      section('Build clean');

      await flutter('clean');

      section('Build platform view app');

      await flutter('build', options: <String>['ios', '-v', '--release', '--config-only']);
    });

    section('Run platform view XCUITests');

    final Device device = await devices.workingDevice;
    if (!await runXcodeTests(
      platformDirectory: path.join(projectDirectory, 'ios'),
      destination: 'id=${device.deviceId}',
      testName: 'native_platform_view_ui_tests_ios',
    )) {
      return TaskResult.failure('Platform view XCUITests failed');
    }

    return TaskResult.success(null);
  });
}
