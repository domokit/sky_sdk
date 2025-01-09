// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert' show json;
import 'dart:io';

import 'package:flutter_devicelab/framework/devices.dart';
import 'package:flutter_devicelab/framework/task_result.dart';
import 'package:flutter_devicelab/tasks/perf_tests.dart';

import 'common.dart';

void main() {
  late Directory testDirectory;
  late File testTarget;
  late Device device;
  late String perfTestOutputPath;

  setUp(() async {
    testDirectory = Directory.systemTemp.createTempSync('test_dir');
    perfTestOutputPath = testOutputDirectory(testDirectory.absolute.path);
    testTarget = File('${testDirectory.absolute.path}/test_file')..createSync();
    device = const FakeDevice(deviceId: 'fakeDeviceId');
    deviceOperatingSystem = DeviceOperatingSystem.fake;
  });

  // This tests when keys like `30hz_frame_percentage`, `60hz_frame_percentage` are not in the generated file.
  test('runs perf tests, no crash if refresh rate percentage keys are not in the data', () async {
    final Map<String, dynamic> fakeData = <String, dynamic>{
      'frame_count': 5,
      'average_frame_build_time_millis': 0.1,
      'worst_frame_build_time_millis': 0.1,
      '90th_percentile_frame_build_time_millis': 0.1,
      '99th_percentile_frame_build_time_millis': 0.1,
      'average_frame_rasterizer_time_millis': 0.1,
      'worst_frame_rasterizer_time_millis': 0.1,
      '90th_percentile_frame_rasterizer_time_millis': 0.1,
      '99th_percentile_frame_rasterizer_time_millis': 0.1,
      'average_layer_cache_count': 1,
      '90th_percentile_layer_cache_count': 1,
      '99th_percentile_layer_cache_count': 1,
      'worst_layer_cache_count': 1,
      'average_layer_cache_memory': 1,
      '90th_percentile_layer_cache_memory': 1,
      '99th_percentile_layer_cache_memory': 1,
      'worst_layer_cache_memory': 1,
      'average_picture_cache_count': 1,
      '90th_percentile_picture_cache_count': 1,
      '99th_percentile_picture_cache_count': 1,
      'worst_picture_cache_count': 1,
      'average_picture_cache_memory': 1,
      '90th_percentile_picture_cache_memory': 1,
      '99th_percentile_picture_cache_memory': 1,
      'worst_picture_cache_memory': 1,
      'total_ui_gc_time': 1,
      'new_gen_gc_count': 1,
      'old_gen_gc_count': 1,
      'average_vsync_transitions_missed': 1,
      '90th_percentile_vsync_transitions_missed': 1,
      '99th_percentile_vsync_transitions_missed': 1,
      'average_frame_request_pending_latency': 0.1,
      '90th_percentile_frame_request_pending_latency': 0.1,
      '99th_percentile_frame_request_pending_latency': 0.1,
    };
    const String resultFileName = 'fake_result';
    void driveCallback(List<String> arguments) {
      final File resultFile = File('$perfTestOutputPath/$resultFileName.json')
        ..createSync(recursive: true);
      resultFile.writeAsStringSync(json.encode(fakeData));
    }

    final PerfTest perfTest = PerfTest(
      testDirectory.absolute.path,
      testTarget.absolute.path,
      'test_file',
      resultFilename: resultFileName,
      device: device,
      flutterDriveCallback: driveCallback,
    );
    final TaskResult result = await perfTest.run();
    expect(result.data!['frame_count'], 5);
  });

  test(
    'runs perf tests, successfully parse refresh rate percentage key-values from data`',
    () async {
      final Map<String, dynamic> fakeData = <String, dynamic>{
        'frame_count': 5,
        'average_frame_build_time_millis': 0.1,
        'worst_frame_build_time_millis': 0.1,
        '90th_percentile_frame_build_time_millis': 0.1,
        '99th_percentile_frame_build_time_millis': 0.1,
        'average_frame_rasterizer_time_millis': 0.1,
        'worst_frame_rasterizer_time_millis': 0.1,
        '90th_percentile_frame_rasterizer_time_millis': 0.1,
        '99th_percentile_frame_rasterizer_time_millis': 0.1,
        'average_layer_cache_count': 1,
        '90th_percentile_layer_cache_count': 1,
        '99th_percentile_layer_cache_count': 1,
        'worst_layer_cache_count': 1,
        'average_layer_cache_memory': 1,
        '90th_percentile_layer_cache_memory': 1,
        '99th_percentile_layer_cache_memory': 1,
        'worst_layer_cache_memory': 1,
        'average_picture_cache_count': 1,
        '90th_percentile_picture_cache_count': 1,
        '99th_percentile_picture_cache_count': 1,
        'worst_picture_cache_count': 1,
        'average_picture_cache_memory': 1,
        '90th_percentile_picture_cache_memory': 1,
        '99th_percentile_picture_cache_memory': 1,
        'worst_picture_cache_memory': 1,
        'total_ui_gc_time': 1,
        'new_gen_gc_count': 1,
        'old_gen_gc_count': 1,
        'average_vsync_transitions_missed': 1,
        '90th_percentile_vsync_transitions_missed': 1,
        '99th_percentile_vsync_transitions_missed': 1,
        '30hz_frame_percentage': 0.1,
        '60hz_frame_percentage': 0.2,
        '80hz_frame_percentage': 0.3,
        '90hz_frame_percentage': 0.4,
        '120hz_frame_percentage': 0.6,
        'illegal_refresh_rate_frame_count': 10,
        'average_frame_request_pending_latency': 0.1,
        '90th_percentile_frame_request_pending_latency': 0.1,
        '99th_percentile_frame_request_pending_latency': 0.1,
      };
      const String resultFileName = 'fake_result';
      void driveCallback(List<String> arguments) {
        final File resultFile = File('$perfTestOutputPath/$resultFileName.json')
          ..createSync(recursive: true);
        resultFile.writeAsStringSync(json.encode(fakeData));
      }

      final PerfTest perfTest = PerfTest(
        testDirectory.absolute.path,
        testTarget.absolute.path,
        'test_file',
        resultFilename: resultFileName,
        device: device,
        flutterDriveCallback: driveCallback,
      );
      final TaskResult result = await perfTest.run();
      expect(result.data!['30hz_frame_percentage'], 0.1);
      expect(result.data!['60hz_frame_percentage'], 0.2);
      expect(result.data!['80hz_frame_percentage'], 0.3);
      expect(result.data!['90hz_frame_percentage'], 0.4);
      expect(result.data!['120hz_frame_percentage'], 0.6);
      expect(result.data!['illegal_refresh_rate_frame_count'], 10);
    },
  );
}
