// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io';

import 'package:flutter_devicelab/framework/devices.dart';
import 'package:flutter_devicelab/framework/framework.dart';
import 'package:flutter_devicelab/framework/task_result.dart';
import 'package:flutter_devicelab/framework/utils.dart';
import 'package:path/path.dart' as p;

void main() {
  task(() async {
    deviceOperatingSystem = DeviceOperatingSystem.android;

    final Device device = await devices.workingDevice;
    await device.unlock();
    final String deviceId = device.deviceId;
    await flutter('packages', options: <String>['get']);

    final String complexLayoutPath = p.join(
      flutterDirectory.path,
      'dev',
      'benchmarks',
      'complex_layout',
    );

    await inDirectory(complexLayoutPath, () async {
      await flutter(
        'drive',
        options: <String>[
          '--no-android-gradle-daemon',
          '-v',
          '--profile',
          '--trace-startup', // Enables "endless" timeline event buffering.
          '-t',
          p.join(complexLayoutPath, 'test_driver', 'semantics_perf.dart'),
          '-d',
          deviceId,
        ],
      );
    });

    final String outputPath =
        Platform.environment['FLUTTER_TEST_OUTPUTS_DIR'] ?? p.join(complexLayoutPath, 'build');
    final String dataPath = p.join(outputPath, 'complex_layout_semantics_perf.json');
    return TaskResult.successFromFile(
      file(dataPath),
      benchmarkScoreKeys: <String>['initialSemanticsTreeCreation'],
    );
  });
}
