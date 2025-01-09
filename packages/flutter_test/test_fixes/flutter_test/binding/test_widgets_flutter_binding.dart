// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui';

import 'package:clock/src/clock.dart';
import 'package:flutter_test/flutter_test.dart';

void main() async {
  TestWidgetsFlutterBinding binding =
      AutomatedTestWidgetsFlutterBinding.ensureInitialized();
  binding.runTest(
    () async {
      // This will be unchanged as there is no equivalent API.
      binding.addTime(Duration(seconds: 30));

      await binding.runAsync(
        () async {},
        // Changes made in https://github.com/flutter/flutter/pull/89952
        additionalTime: Duration(seconds: 25),
      );
    },
    () {},
    // This timeout will be removed and not replaced since there is no
    // equivalent API at this layer.
    timeout: Duration(minutes: 30),
  );
}
