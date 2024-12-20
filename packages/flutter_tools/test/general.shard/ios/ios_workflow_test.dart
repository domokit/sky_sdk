// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_tools/src/base/platform.dart';
import 'package:flutter_tools/src/base/version.dart';
import 'package:flutter_tools/src/ios/ios_workflow.dart';
import 'package:flutter_tools/src/ios/xcodeproj.dart';
import 'package:flutter_tools/src/macos/xcode.dart';

import '../../src/common.dart';
import '../../src/fake_process_manager.dart';
import '../../src/fakes.dart';

void main() {
  testWithoutContext('iOS workflow is disabled if feature is disabled', () {
    final IOSWorkflow iosWorkflow = IOSWorkflow(
      platform: FakePlatform(operatingSystem: 'macOS'),
      xcode: Xcode.test(processManager: FakeProcessManager.any()),
      featureFlags: TestFeatureFlags(isIOSEnabled: false),
    );

    expect(iosWorkflow.appliesToHostPlatform, false);
    expect(iosWorkflow.canLaunchDevices, false);
    expect(iosWorkflow.canListDevices, false);
  });

  testWithoutContext('iOS workflow is disabled on Linux', () {
    final IOSWorkflow iosWorkflow = IOSWorkflow(
      platform: FakePlatform(),
      xcode: Xcode.test(processManager: FakeProcessManager.any()),
      featureFlags: TestFeatureFlags(),
    );

    expect(iosWorkflow.appliesToHostPlatform, false);
    expect(iosWorkflow.canLaunchDevices, false);
    expect(iosWorkflow.canListDevices, false);
  });

  testWithoutContext('iOS workflow is disabled on windows', () {
    final IOSWorkflow iosWorkflow = IOSWorkflow(
      platform: FakePlatform(operatingSystem: 'windows'),
      xcode: Xcode.test(processManager: FakeProcessManager.any()),
      featureFlags: TestFeatureFlags(),
    );

    expect(iosWorkflow.appliesToHostPlatform, false);
    expect(iosWorkflow.canLaunchDevices, false);
    expect(iosWorkflow.canListDevices, false);
  });

  testWithoutContext('iOS workflow applies on macOS, no Xcode or simctl', () {
    final FakeProcessManager xcodeProcessManager = FakeProcessManager.list(<FakeCommand>[
      const FakeCommand(
        command: <String>['xcrun', 'simctl', 'list', 'devices', 'booted'],
        exitCode: 1,
      ),
    ]);
    final IOSWorkflow iosWorkflow = IOSWorkflow(
      platform: FakePlatform(operatingSystem: 'macos'),
      xcode: Xcode.test(
        processManager: xcodeProcessManager,
        xcodeProjectInterpreter: XcodeProjectInterpreter.test(
          processManager: FakeProcessManager.any(),
          version: null,
        ),
      ),
      featureFlags: TestFeatureFlags(),
    );

    expect(iosWorkflow.appliesToHostPlatform, true);
    expect(iosWorkflow.canLaunchDevices, false);
    expect(iosWorkflow.canListDevices, false);
    expect(iosWorkflow.canListEmulators, false);
    expect(xcodeProcessManager, hasNoRemainingExpectations);
  });

  testWithoutContext('iOS workflow can list devices even when Xcode version is too low', () {
    final Xcode xcode = Xcode.test(
      processManager: FakeProcessManager.any(),
      xcodeProjectInterpreter: XcodeProjectInterpreter.test(
        processManager: FakeProcessManager.any(),
        version: Version(1, 0, 0),
      ),
    );

    final IOSWorkflow iosWorkflow = IOSWorkflow(
      platform: FakePlatform(operatingSystem: 'macos'),
      xcode: xcode,
      featureFlags: TestFeatureFlags(),
    );

    // Make sure we're testing the right Xcode state.
    // expect(xcode.isInstalledAndMeetsVersionCheck, true);
    expect(xcode.isSimctlInstalled, true);
    expect(iosWorkflow.canLaunchDevices, false);
    expect(iosWorkflow.canListDevices, true);
    expect(iosWorkflow.canListEmulators, false);
  });

  testWithoutContext('iOS workflow can launch devices when Xcode is set up', () {
    final Xcode xcode = Xcode.test(
      processManager: FakeProcessManager.any(),
      xcodeProjectInterpreter: XcodeProjectInterpreter.test(
        processManager: FakeProcessManager.any(),
        version: Version(1000, 0, 0),
      ),
    );

    final IOSWorkflow iosWorkflow = IOSWorkflow(
      platform: FakePlatform(operatingSystem: 'macos'),
      xcode: xcode,
      featureFlags: TestFeatureFlags(),
    );

    // Make sure we're testing the right Xcode state.
    expect(xcode.isInstalledAndMeetsVersionCheck, true);
    expect(xcode.isSimctlInstalled, true);
    expect(iosWorkflow.canLaunchDevices, true);
    expect(iosWorkflow.canListDevices, true);
    expect(iosWorkflow.canListEmulators, false);
  });
}
