// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_devicelab/framework/devices.dart';
import 'package:flutter_devicelab/framework/framework.dart';
import 'package:flutter_devicelab/framework/utils.dart';
import 'package:flutter_devicelab/tasks/perf_tests.dart';

void main() async {
  deviceOperatingSystem = DeviceOperatingSystem.android;
  await task(
    PerfTest(
      '${flutterDirectory.path}/dev/integration_tests/flutter_gallery',
      'test_driver/page_transitions_perf.dart',
      'page_transition_perf',
    ).run,
  );
}
