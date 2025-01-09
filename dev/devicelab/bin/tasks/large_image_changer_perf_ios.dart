// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_devicelab/framework/devices.dart';
import 'package:flutter_devicelab/framework/framework.dart';
import 'package:flutter_devicelab/framework/utils.dart';
import 'package:flutter_devicelab/tasks/perf_tests.dart';

Future<void> main() async {
  deviceOperatingSystem = DeviceOperatingSystem.ios;
  await task(
    PerfTest(
      '${flutterDirectory.path}/dev/benchmarks/macrobenchmarks',
      'test_driver/large_image_changer.dart',
      'large_image_changer',
      // This benchmark doesn't care about frame times, frame times will be heavily
      // impacted by IO time for loading the image initially.
      benchmarkScoreKeys: <String>[
        'average_cpu_usage',
        'average_gpu_usage',
        'average_memory_usage',
        '90th_percentile_memory_usage',
        '99th_percentile_memory_usage',
        'new_gen_gc_count',
        'old_gen_gc_count',
      ],
    ).run,
  );
}
