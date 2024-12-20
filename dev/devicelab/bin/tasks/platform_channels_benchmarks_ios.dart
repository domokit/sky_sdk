// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_devicelab/framework/devices.dart' show DeviceOperatingSystem;
import 'package:flutter_devicelab/framework/framework.dart' show task;
import 'package:flutter_devicelab/tasks/platform_channels_benchmarks.dart'
    as platform_channels_benchmarks;

Future<void> main() async {
  await task(platform_channels_benchmarks.runTask(DeviceOperatingSystem.ios));
}
