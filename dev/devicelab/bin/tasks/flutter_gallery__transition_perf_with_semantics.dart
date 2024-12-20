// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_devicelab/framework/devices.dart';
import 'package:flutter_devicelab/framework/framework.dart';
import 'package:flutter_devicelab/framework/task_result.dart';
import 'package:flutter_devicelab/tasks/gallery.dart';

Future<void> main() async {
  deviceOperatingSystem = DeviceOperatingSystem.android;
  await task(() async {
    final TaskResult withoutSemantics = await createGalleryTransitionTest()();
    final TaskResult withSemantics = await createGalleryTransitionTest(semanticsEnabled: true)();
    final bool withSemanticsDataMissing =
        withSemantics.benchmarkScoreKeys == null || withSemantics.benchmarkScoreKeys!.isEmpty;
    final bool withoutSemanticsDataMissing =
        withoutSemantics.benchmarkScoreKeys == null || withoutSemantics.benchmarkScoreKeys!.isEmpty;
    if (withSemanticsDataMissing || withoutSemanticsDataMissing) {
      String message = 'Lack of data';
      if (withSemanticsDataMissing) {
        message += ' for test with semantics';
        if (withoutSemanticsDataMissing) {
          message += ' and without semantics';
        }
      } else {
        message += 'for test without semantics';
      }
      return TaskResult.failure(message);
    }

    final List<String> benchmarkScoreKeys = <String>[];
    final Map<String, dynamic> data = <String, dynamic>{};
    for (final String key in withSemantics.benchmarkScoreKeys!) {
      final String deltaKey = 'delta_$key';
      data[deltaKey] = (withSemantics.data![key] as num) - (withoutSemantics.data![key] as num);
      data['semantics_$key'] = withSemantics.data![key];
      data[key] = withoutSemantics.data![key];
      benchmarkScoreKeys.add(deltaKey);
    }

    return TaskResult.success(data, benchmarkScoreKeys: benchmarkScoreKeys);
  });
}
