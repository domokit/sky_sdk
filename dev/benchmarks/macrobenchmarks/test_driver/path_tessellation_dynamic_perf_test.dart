// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_driver/flutter_driver.dart';
import 'package:macrobenchmarks/common.dart';

import 'util.dart';

void main() {
  macroPerfTest(
    'tessellation_perf_dynamic',
    kPathTessellationRouteName,
    pageDelay: const Duration(seconds: 1),
    duration: const Duration(seconds: 10),
    setupOps: (FlutterDriver driver) async {
      final SerializableFinder animateButton = find.byValueKey('animate_button');
      await driver.tap(animateButton);
      await Future<void>.delayed(const Duration(seconds: 1));
    },
  );
}
