// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_devicelab/framework/framework.dart';
import 'package:flutter_devicelab/framework/utils.dart';
import 'package:flutter_devicelab/tasks/perf_tests.dart';

Future<void> main() async {
  await task(
    MemoryTest(
      '${flutterDirectory.path}/dev/integration_tests/flutter_gallery',
      'test_memory/image_cache_memory.dart',
      'io.flutter.demo.gallery',
    ).run,
  );
}
