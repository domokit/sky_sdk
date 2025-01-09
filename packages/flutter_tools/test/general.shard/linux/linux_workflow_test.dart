// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_tools/src/base/platform.dart';
import 'package:flutter_tools/src/features.dart';
import 'package:flutter_tools/src/linux/linux_workflow.dart';

import '../../src/common.dart';
import '../../src/fakes.dart';

void main() {
  final Platform linux = FakePlatform(environment: <String, String>{});
  final Platform notLinux = FakePlatform(
    operatingSystem: 'windows',
    environment: <String, String>{},
  );
  final FeatureFlags enabledFlags = TestFeatureFlags(isLinuxEnabled: true);
  final FeatureFlags disabledFlags = TestFeatureFlags();

  testWithoutContext('Applies to Linux platform', () {
    final LinuxWorkflow linuxWorkflow = LinuxWorkflow(platform: linux, featureFlags: enabledFlags);

    expect(linuxWorkflow.appliesToHostPlatform, true);
    expect(linuxWorkflow.canLaunchDevices, true);
    expect(linuxWorkflow.canListDevices, true);
    expect(linuxWorkflow.canListEmulators, false);
  });

  testWithoutContext('Does not apply to non-Linux platform', () {
    final LinuxWorkflow linuxWorkflow = LinuxWorkflow(
      platform: notLinux,
      featureFlags: enabledFlags,
    );

    expect(linuxWorkflow.appliesToHostPlatform, false);
    expect(linuxWorkflow.canLaunchDevices, false);
    expect(linuxWorkflow.canListDevices, false);
    expect(linuxWorkflow.canListEmulators, false);
  });

  testWithoutContext('Does not apply when the Linux desktop feature is disabled', () {
    final LinuxWorkflow linuxWorkflow = LinuxWorkflow(platform: linux, featureFlags: disabledFlags);

    expect(linuxWorkflow.appliesToHostPlatform, false);
    expect(linuxWorkflow.canLaunchDevices, false);
    expect(linuxWorkflow.canListDevices, false);
    expect(linuxWorkflow.canListEmulators, false);
  });
}
