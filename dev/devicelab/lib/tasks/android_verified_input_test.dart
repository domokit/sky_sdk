// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import '../framework/devices.dart';
import '../framework/framework.dart';
import '../framework/task_result.dart';
import '../framework/utils.dart';

TaskFunction androidVerifiedInputTest({Map<String, String>? environment}) {
  return DriverTest(
    '${flutterDirectory.path}/dev/integration_tests/android_verified_input',
    'test_driver/main_test.dart',
    environment: environment,
  ).call;
}

class DriverTest {
  DriverTest(
    this.testDirectory,
    this.testTarget, {
    this.extraOptions = const <String>[],
    this.deviceIdOverride,
    this.environment,
  });

  final String testDirectory;
  final String testTarget;
  final List<String> extraOptions;
  final String? deviceIdOverride;
  final Map<String, String>? environment;

  Future<TaskResult> call() {
    return inDirectory<TaskResult>(testDirectory, () async {
      String deviceId;
      if (deviceIdOverride != null) {
        deviceId = deviceIdOverride!;
      } else {
        final Device device = await devices.workingDevice;
        await device.unlock();
        deviceId = device.deviceId;
      }
      await flutter('packages', options: <String>['get']);

      final List<String> options = <String>[
        '--no-android-gradle-daemon',
        '-v',
        '-t',
        testTarget,
        '-d',
        deviceId,
        ...extraOptions,
      ];
      await flutter('drive', options: options, environment: environment);

      return TaskResult.success(null);
    });
  }
}
