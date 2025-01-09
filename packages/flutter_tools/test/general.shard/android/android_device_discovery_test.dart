// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/memory.dart';
import 'package:flutter_tools/src/android/android_device_discovery.dart';
import 'package:flutter_tools/src/android/android_sdk.dart';
import 'package:flutter_tools/src/android/android_workflow.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/platform.dart';
import 'package:flutter_tools/src/base/user_messages.dart';
import 'package:flutter_tools/src/device.dart';
import 'package:test/fake.dart';

import '../../src/common.dart';
import '../../src/fake_process_manager.dart';
import '../../src/fakes.dart';

void main() {
  late AndroidWorkflow androidWorkflow;

  setUp(() {
    androidWorkflow = AndroidWorkflow(
      androidSdk: FakeAndroidSdk(),
      featureFlags: TestFeatureFlags(),
    );
  });

  testWithoutContext(
    'AndroidDevices returns empty device list and diagnostics on null adb',
    () async {
      final AndroidDevices androidDevices = AndroidDevices(
        androidSdk: FakeAndroidSdk(null),
        logger: BufferLogger.test(),
        androidWorkflow: AndroidWorkflow(
          androidSdk: FakeAndroidSdk(null),
          featureFlags: TestFeatureFlags(),
        ),
        processManager: FakeProcessManager.empty(),
        fileSystem: MemoryFileSystem.test(),
        platform: FakePlatform(),
        userMessages: UserMessages(),
      );

      expect(await androidDevices.pollingGetDevices(), isEmpty);
      expect(await androidDevices.getDiagnostics(), isEmpty);
    },
  );

  testWithoutContext(
    'AndroidDevices returns empty device list and diagnostics when adb cannot be run',
    () async {
      final FakeProcessManager fakeProcessManager = FakeProcessManager.empty();
      fakeProcessManager.excludedExecutables.add('adb');
      final AndroidDevices androidDevices = AndroidDevices(
        androidSdk: FakeAndroidSdk(),
        logger: BufferLogger.test(),
        androidWorkflow: AndroidWorkflow(
          androidSdk: FakeAndroidSdk(),
          featureFlags: TestFeatureFlags(),
        ),
        processManager: fakeProcessManager,
        fileSystem: MemoryFileSystem.test(),
        platform: FakePlatform(),
        userMessages: UserMessages(),
      );

      expect(await androidDevices.pollingGetDevices(), isEmpty);
      expect(await androidDevices.getDiagnostics(), isEmpty);
      expect(fakeProcessManager, hasNoRemainingExpectations);
    },
  );

  testWithoutContext(
    'AndroidDevices returns empty device list and diagnostics on null Android SDK',
    () async {
      final AndroidDevices androidDevices = AndroidDevices(
        logger: BufferLogger.test(),
        androidWorkflow: AndroidWorkflow(
          androidSdk: FakeAndroidSdk(null),
          featureFlags: TestFeatureFlags(),
        ),
        processManager: FakeProcessManager.empty(),
        fileSystem: MemoryFileSystem.test(),
        platform: FakePlatform(),
        userMessages: UserMessages(),
      );

      expect(await androidDevices.pollingGetDevices(), isEmpty);
      expect(await androidDevices.getDiagnostics(), isEmpty);
    },
  );

  testWithoutContext('AndroidDevices throwsToolExit on failing adb', () {
    final ProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
      const FakeCommand(
        command: <String>['adb', 'devices', '-l'],
        exitCode: 1,
        stderr: '<stderr from adb>',
      ),
    ]);
    final AndroidDevices androidDevices = AndroidDevices(
      androidSdk: FakeAndroidSdk(),
      logger: BufferLogger.test(),
      androidWorkflow: androidWorkflow,
      processManager: processManager,
      fileSystem: MemoryFileSystem.test(),
      platform: FakePlatform(),
      userMessages: UserMessages(),
    );

    expect(
      androidDevices.pollingGetDevices(),
      throwsToolExit(
        message:
            'Unable to run "adb", check your Android SDK installation and ANDROID_HOME environment variable: adb\n'
            'Error details: Process exited abnormally with exit code 1:\n'
            '<stderr from adb>',
      ),
    );
  });

  testWithoutContext('AndroidDevices is disabled if feature is disabled', () {
    final AndroidDevices androidDevices = AndroidDevices(
      androidSdk: FakeAndroidSdk(),
      logger: BufferLogger.test(),
      androidWorkflow: AndroidWorkflow(
        androidSdk: FakeAndroidSdk(),
        featureFlags: TestFeatureFlags(isAndroidEnabled: false),
      ),
      processManager: FakeProcessManager.any(),
      fileSystem: MemoryFileSystem.test(),
      platform: FakePlatform(),
      userMessages: UserMessages(),
    );

    expect(androidDevices.supportsPlatform, false);
  });

  testWithoutContext('AndroidDevices can parse output for physical attached devices', () async {
    final AndroidDevices androidDevices = AndroidDevices(
      userMessages: UserMessages(),
      androidWorkflow: androidWorkflow,
      androidSdk: FakeAndroidSdk(),
      logger: BufferLogger.test(),
      processManager: FakeProcessManager.list(<FakeCommand>[
        const FakeCommand(
          command: <String>['adb', 'devices', '-l'],
          stdout: '''
List of devices attached
05a02bac               device usb:336592896X product:razor model:Nexus_7 device:flo

  ''',
        ),
      ]),
      platform: FakePlatform(),
      fileSystem: MemoryFileSystem.test(),
    );

    final List<Device> devices = await androidDevices.pollingGetDevices();

    expect(devices, hasLength(1));
    expect(devices.first.name, 'Nexus 7');
    expect(devices.first.category, Category.mobile);
    expect(devices.first.connectionInterface, DeviceConnectionInterface.attached);
  });

  testWithoutContext('AndroidDevices can parse output for physical wireless devices', () async {
    final AndroidDevices androidDevices = AndroidDevices(
      userMessages: UserMessages(),
      androidWorkflow: androidWorkflow,
      androidSdk: FakeAndroidSdk(),
      logger: BufferLogger.test(),
      processManager: FakeProcessManager.list(<FakeCommand>[
        const FakeCommand(
          command: <String>['adb', 'devices', '-l'],
          stdout: '''
List of devices attached
05a02bac._adb-tls-connect._tcp.               device product:razor model:Nexus_7 device:flo

  ''',
        ),
      ]),
      platform: FakePlatform(),
      fileSystem: MemoryFileSystem.test(),
    );

    final List<Device> devices = await androidDevices.pollingGetDevices();

    expect(devices, hasLength(1));
    expect(devices.first.name, 'Nexus 7');
    expect(devices.first.category, Category.mobile);
    expect(devices.first.connectionInterface, DeviceConnectionInterface.wireless);
  });

  testWithoutContext('AndroidDevices can parse output for emulators and short listings', () async {
    final AndroidDevices androidDevices = AndroidDevices(
      userMessages: UserMessages(),
      androidWorkflow: androidWorkflow,
      androidSdk: FakeAndroidSdk(),
      logger: BufferLogger.test(),
      processManager: FakeProcessManager.list(<FakeCommand>[
        const FakeCommand(
          command: <String>['adb', 'devices', '-l'],
          stdout: '''
List of devices attached
localhost:36790        device
0149947A0D01500C       device usb:340787200X
emulator-5612          host features:shell_2

  ''',
        ),
      ]),
      platform: FakePlatform(),
      fileSystem: MemoryFileSystem.test(),
    );

    final List<Device> devices = await androidDevices.pollingGetDevices();

    expect(devices, hasLength(3));
    expect(devices[0].name, 'localhost:36790');
    expect(devices[1].name, '0149947A0D01500C');
    expect(devices[2].name, 'emulator-5612');
  });

  testWithoutContext('AndroidDevices can parse output from android n', () async {
    final AndroidDevices androidDevices = AndroidDevices(
      userMessages: UserMessages(),
      androidWorkflow: androidWorkflow,
      androidSdk: FakeAndroidSdk(),
      logger: BufferLogger.test(),
      processManager: FakeProcessManager.list(<FakeCommand>[
        const FakeCommand(
          command: <String>['adb', 'devices', '-l'],
          stdout: '''
List of devices attached
ZX1G22JJWR             device usb:3-3 product:shamu model:Nexus_6 device:shamu features:cmd,shell_v2

''',
        ),
      ]),
      platform: FakePlatform(),
      fileSystem: MemoryFileSystem.test(),
    );

    final List<Device> devices = await androidDevices.pollingGetDevices();

    expect(devices, hasLength(1));
    expect(devices.first.name, 'Nexus 6');
  });

  testWithoutContext('AndroidDevices provides adb error message as diagnostics', () async {
    final AndroidDevices androidDevices = AndroidDevices(
      userMessages: UserMessages(),
      androidWorkflow: androidWorkflow,
      androidSdk: FakeAndroidSdk(),
      logger: BufferLogger.test(),
      processManager: FakeProcessManager.list(<FakeCommand>[
        const FakeCommand(
          command: <String>['adb', 'devices', '-l'],
          stdout: '''
It appears you do not have 'Android SDK Platform-tools' installed.
Use the 'android' tool to install them:
  android update sdk --no-ui --filter 'platform-tools'
''',
        ),
      ]),
      platform: FakePlatform(),
      fileSystem: MemoryFileSystem.test(),
    );

    final List<String> diagnostics = await androidDevices.getDiagnostics();

    expect(diagnostics, hasLength(1));
    expect(diagnostics.first, contains('you do not have'));
  });
}

class FakeAndroidSdk extends Fake implements AndroidSdk {
  FakeAndroidSdk([this.adbPath = 'adb']);

  @override
  final String? adbPath;
}
