// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_devicelab/framework/framework.dart';
import 'package:flutter_devicelab/framework/utils.dart';
import 'package:flutter_devicelab/tasks/perf_tests.dart';

Future<void> main() async {
  await task(
    ReportedDurationTest(
      ReportedDurationTestFlavor.debug,
      '${flutterDirectory.path}/examples/image_list',
      'lib/main.dart',
      'com.example.image_list',
      RegExp(r'===image_list=== all loaded in ([\d]+)ms.'),
    ).run,
  );
}
