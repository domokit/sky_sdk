// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_driver/flutter_driver.dart';
import 'package:macrobenchmarks/common.dart';

import 'util.dart';

void main() {
  macroPerfTest(
    'tessellation_perf_static',
    kPathTessellationRouteName,
    pageDelay: const Duration(seconds: 1),
    driverOps: (FlutterDriver driver) async {
      final SerializableFinder listView = find.byValueKey('list_view');
      Future<void> scrollOnce(double offset) async {
        await driver.scroll(listView, 0.0, offset, const Duration(milliseconds: 450));
        await Future<void>.delayed(const Duration(milliseconds: 500));
      }

      for (int i = 0; i < 3; i += 1) {
        await scrollOnce(-600.0);
        await scrollOnce(-600.0);
        await scrollOnce(600.0);
        await scrollOnce(600.0);
      }
    },
  );
}
