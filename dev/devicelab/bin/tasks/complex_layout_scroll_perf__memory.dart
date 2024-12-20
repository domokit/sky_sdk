// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_devicelab/framework/devices.dart';
import 'package:flutter_devicelab/framework/framework.dart';
import 'package:flutter_devicelab/framework/utils.dart';
import 'package:flutter_devicelab/tasks/perf_tests.dart';

Future<void> main() async {
  deviceOperatingSystem = DeviceOperatingSystem.android;
  await task(
    MemoryTest(
      '${flutterDirectory.path}/dev/benchmarks/complex_layout',
      'test_memory/scroll_perf.dart',
      'com.yourcompany.complexLayout',
      requiresTapToStart: true,
    ).run,
  );
}
