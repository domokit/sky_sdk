// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_tools/src/custom_devices/custom_device_workflow.dart';

import '../../src/common.dart';
import '../../src/fakes.dart';

void main() {
  testWithoutContext(
    'CustomDeviceWorkflow reports false when custom devices feature is disabled',
    () {
      final CustomDeviceWorkflow workflow = CustomDeviceWorkflow(featureFlags: TestFeatureFlags());
      expect(workflow.appliesToHostPlatform, false);
      expect(workflow.canLaunchDevices, false);
      expect(workflow.canListDevices, false);
      expect(workflow.canListEmulators, false);
    },
  );

  testWithoutContext(
    'CustomDeviceWorkflow reports true for everything except canListEmulators when custom devices feature is enabled',
    () {
      final CustomDeviceWorkflow workflow = CustomDeviceWorkflow(
        featureFlags: TestFeatureFlags(areCustomDevicesEnabled: true),
      );
      expect(workflow.appliesToHostPlatform, true);
      expect(workflow.canLaunchDevices, true);
      expect(workflow.canListDevices, true);
      expect(workflow.canListEmulators, false);
    },
  );
}
