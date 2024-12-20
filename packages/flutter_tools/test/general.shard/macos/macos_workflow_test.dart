// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_tools/src/base/platform.dart';
import 'package:flutter_tools/src/macos/macos_workflow.dart';

import '../../src/common.dart';
import '../../src/fakes.dart';

final FakePlatform macOS = FakePlatform(operatingSystem: 'macos');

final FakePlatform linux = FakePlatform();

void main() {
  testWithoutContext('Applies to macOS platform', () {
    final MacOSWorkflow macOSWorkflow = MacOSWorkflow(
      platform: macOS,
      featureFlags: TestFeatureFlags(isMacOSEnabled: true),
    );

    expect(macOSWorkflow.appliesToHostPlatform, true);
    expect(macOSWorkflow.canListDevices, true);
    expect(macOSWorkflow.canLaunchDevices, true);
    expect(macOSWorkflow.canListEmulators, false);
  });

  testWithoutContext('Does not apply to non-macOS platform', () {
    final MacOSWorkflow macOSWorkflow = MacOSWorkflow(
      platform: linux,
      featureFlags: TestFeatureFlags(isMacOSEnabled: true),
    );

    expect(macOSWorkflow.appliesToHostPlatform, false);
    expect(macOSWorkflow.canListDevices, false);
    expect(macOSWorkflow.canLaunchDevices, false);
    expect(macOSWorkflow.canListEmulators, false);
  });

  testWithoutContext('Does not apply when feature is disabled', () {
    final MacOSWorkflow macOSWorkflow = MacOSWorkflow(
      platform: macOS,
      featureFlags: TestFeatureFlags(),
    );

    expect(macOSWorkflow.appliesToHostPlatform, false);
    expect(macOSWorkflow.canListDevices, false);
    expect(macOSWorkflow.canLaunchDevices, false);
    expect(macOSWorkflow.canListEmulators, false);
  });
}
