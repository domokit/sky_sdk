// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_devicelab/framework/devices.dart';
import 'package:flutter_devicelab/framework/framework.dart';
import 'package:flutter_devicelab/framework/task_result.dart';

/// Smoke test of a successful task.
Future<void> main() async {
  deviceOperatingSystem = DeviceOperatingSystem.fake;
  await task(() async {
    final Device device = await devices.workingDevice;
    if (device.deviceId == 'FAKE_SUCCESS') {
      return TaskResult.success(
        <String, dynamic>{'metric1': 42, 'metric2': 123, 'not_a_metric': 'something'},
        benchmarkScoreKeys: <String>['metric1', 'metric2'],
      );
    } else {
      return TaskResult.failure('Failed');
    }
  });
}
