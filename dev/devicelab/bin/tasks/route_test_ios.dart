// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io';

import 'package:flutter_devicelab/framework/devices.dart';
import 'package:flutter_devicelab/framework/framework.dart';
import 'package:flutter_devicelab/framework/task_result.dart';
import 'package:flutter_devicelab/framework/utils.dart';
import 'package:path/path.dart' as path;

void main() {
  task(() async {
    deviceOperatingSystem = DeviceOperatingSystem.ios;
    final Device device = await devices.workingDevice;
    await device.unlock();
    final Directory appDir = dir(path.join(flutterDirectory.path, 'dev/integration_tests/ui'));
    section('TEST WHETHER `flutter drive --route` WORKS on IOS');
    await inDirectory(appDir, () async {
      return flutter(
        'drive',
        options: <String>['-d', device.deviceId, '--route', '/smuggle-it', 'lib/route.dart'],
      );
    });
    return TaskResult.success(null);
  });
}
