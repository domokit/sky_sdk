// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_tools/src/base/platform.dart';
import 'package:flutter_tools/src/windows/windows_workflow.dart';

import '../../src/common.dart';
import '../../src/fakes.dart';

void main() {
  final FakePlatform windows = FakePlatform(operatingSystem: 'windows');
  final FakePlatform notWindows = FakePlatform();

  testWithoutContext(
    'Windows workflow configuration when feature is enabled on Windows host machine',
    () {
      final WindowsWorkflow windowsWorkflow = WindowsWorkflow(
        platform: windows,
        featureFlags: TestFeatureFlags(isWindowsEnabled: true),
      );

      expect(windowsWorkflow.appliesToHostPlatform, true);
      expect(windowsWorkflow.canListDevices, true);
      expect(windowsWorkflow.canLaunchDevices, true);
      expect(windowsWorkflow.canListEmulators, false);
    },
  );

  testWithoutContext(
    'Windows workflow configuration when feature is disabled on Windows host machine',
    () {
      final WindowsWorkflow windowsWorkflow = WindowsWorkflow(
        platform: windows,
        featureFlags: TestFeatureFlags(),
      );

      expect(windowsWorkflow.appliesToHostPlatform, false);
      expect(windowsWorkflow.canListDevices, false);
      expect(windowsWorkflow.canLaunchDevices, false);
      expect(windowsWorkflow.canListEmulators, false);
    },
  );

  testWithoutContext(
    'Windows workflow configuration when feature is enabled on non-Windows host machine',
    () {
      final WindowsWorkflow windowsWorkflow = WindowsWorkflow(
        platform: notWindows,
        featureFlags: TestFeatureFlags(isWindowsEnabled: true),
      );

      expect(windowsWorkflow.appliesToHostPlatform, false);
      expect(windowsWorkflow.canListDevices, false);
      expect(windowsWorkflow.canLaunchDevices, false);
      expect(windowsWorkflow.canListEmulators, false);
    },
  );

  testWithoutContext(
    'Windows workflow configuration when feature is disabled on non-Windows host machine',
    () {
      final WindowsWorkflow windowsWorkflow = WindowsWorkflow(
        platform: notWindows,
        featureFlags: TestFeatureFlags(),
      );

      expect(windowsWorkflow.appliesToHostPlatform, false);
      expect(windowsWorkflow.canListDevices, false);
      expect(windowsWorkflow.canLaunchDevices, false);
      expect(windowsWorkflow.canListEmulators, false);
    },
  );
}
