// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/memory.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/os.dart';
import 'package:flutter_tools/src/base/platform.dart';
import 'package:flutter_tools/src/build_info.dart';
import 'package:flutter_tools/src/device.dart';
import 'package:flutter_tools/src/linux/application_package.dart';
import 'package:flutter_tools/src/linux/linux_device.dart';
import 'package:flutter_tools/src/project.dart';
import 'package:test/fake.dart';

import '../../src/common.dart';
import '../../src/fake_process_manager.dart';
import '../../src/fakes.dart';

final FakePlatform linux = FakePlatform();
final FakePlatform windows = FakePlatform(operatingSystem: 'windows');

void main() {
  testWithoutContext('LinuxDevice defaults', () async {
    final LinuxDevice device = LinuxDevice(
      processManager: FakeProcessManager.any(),
      logger: BufferLogger.test(),
      fileSystem: MemoryFileSystem.test(),
      operatingSystemUtils: FakeOperatingSystemUtils(),
    );

    final PrebuiltLinuxApp linuxApp = PrebuiltLinuxApp(executable: 'foo');
    expect(await device.targetPlatform, TargetPlatform.linux_x64);
    expect(device.name, 'Linux');
    expect(await device.installApp(linuxApp), true);
    expect(await device.uninstallApp(linuxApp), true);
    expect(await device.isLatestBuildInstalled(linuxApp), true);
    expect(await device.isAppInstalled(linuxApp), true);
    expect(await device.stopApp(linuxApp), true);
    expect(device.category, Category.desktop);

    expect(device.supportsRuntimeMode(BuildMode.debug), true);
    expect(device.supportsRuntimeMode(BuildMode.profile), true);
    expect(device.supportsRuntimeMode(BuildMode.release), true);
    expect(device.supportsRuntimeMode(BuildMode.jitRelease), false);
  });

  testWithoutContext('LinuxDevice on arm64 hosts is arm64', () async {
    final LinuxDevice deviceArm64Host = LinuxDevice(
      processManager: FakeProcessManager.any(),
      logger: BufferLogger.test(),
      fileSystem: MemoryFileSystem.test(),
      operatingSystemUtils: FakeOperatingSystemUtils(hostPlatform: HostPlatform.linux_arm64),
    );
    expect(await deviceArm64Host.targetPlatform, TargetPlatform.linux_arm64);
  });

  testWithoutContext('LinuxDevice: no devices listed if platform unsupported', () async {
    expect(
      await LinuxDevices(
        fileSystem: MemoryFileSystem.test(),
        platform: windows,
        featureFlags: TestFeatureFlags(isLinuxEnabled: true),
        logger: BufferLogger.test(),
        processManager: FakeProcessManager.any(),
        operatingSystemUtils: FakeOperatingSystemUtils(),
      ).devices(),
      <Device>[],
    );
  });

  testWithoutContext('LinuxDevice: no devices listed if Linux feature flag disabled', () async {
    expect(
      await LinuxDevices(
        fileSystem: MemoryFileSystem.test(),
        platform: linux,
        featureFlags: TestFeatureFlags(),
        logger: BufferLogger.test(),
        processManager: FakeProcessManager.any(),
        operatingSystemUtils: FakeOperatingSystemUtils(),
      ).devices(),
      <Device>[],
    );
  });

  testWithoutContext('LinuxDevice: devices', () async {
    expect(
      await LinuxDevices(
        fileSystem: MemoryFileSystem.test(),
        platform: linux,
        featureFlags: TestFeatureFlags(isLinuxEnabled: true),
        logger: BufferLogger.test(),
        processManager: FakeProcessManager.any(),
        operatingSystemUtils: FakeOperatingSystemUtils(),
      ).devices(),
      hasLength(1),
    );
  });

  testWithoutContext('LinuxDevice has well known id "linux"', () async {
    expect(
      LinuxDevices(
        fileSystem: MemoryFileSystem.test(),
        platform: linux,
        featureFlags: TestFeatureFlags(isLinuxEnabled: true),
        logger: BufferLogger.test(),
        processManager: FakeProcessManager.any(),
        operatingSystemUtils: FakeOperatingSystemUtils(),
      ).wellKnownIds,
      <String>['linux'],
    );
  });

  testWithoutContext('LinuxDevice: discoverDevices', () async {
    // Timeout ignored.
    final List<Device> devices = await LinuxDevices(
      fileSystem: MemoryFileSystem.test(),
      platform: linux,
      featureFlags: TestFeatureFlags(isLinuxEnabled: true),
      logger: BufferLogger.test(),
      processManager: FakeProcessManager.any(),
      operatingSystemUtils: FakeOperatingSystemUtils(),
    ).discoverDevices(timeout: const Duration(seconds: 10));
    expect(devices, hasLength(1));
  });

  testWithoutContext('LinuxDevice.isSupportedForProject is true with editable host app', () async {
    final FileSystem fileSystem = MemoryFileSystem.test();
    fileSystem.file('pubspec.yaml').createSync();
    fileSystem.directory('linux').createSync();
    final FlutterProject flutterProject = setUpFlutterProject(fileSystem.currentDirectory);

    expect(
      LinuxDevice(
        logger: BufferLogger.test(),
        processManager: FakeProcessManager.any(),
        fileSystem: fileSystem,
        operatingSystemUtils: FakeOperatingSystemUtils(),
      ).isSupportedForProject(flutterProject),
      true,
    );
  });

  testWithoutContext('LinuxDevice.isSupportedForProject is false with no host app', () async {
    final FileSystem fileSystem = MemoryFileSystem.test();
    fileSystem.file('pubspec.yaml').createSync();
    final FlutterProject flutterProject = setUpFlutterProject(fileSystem.currentDirectory);

    expect(
      LinuxDevice(
        logger: BufferLogger.test(),
        processManager: FakeProcessManager.any(),
        fileSystem: fileSystem,
        operatingSystemUtils: FakeOperatingSystemUtils(),
      ).isSupportedForProject(flutterProject),
      false,
    );
  });

  testWithoutContext(
    'LinuxDevice.executablePathForDevice uses the correct package executable',
    () async {
      final FakeLinuxApp mockApp = FakeLinuxApp();
      final LinuxDevice device = LinuxDevice(
        logger: BufferLogger.test(),
        processManager: FakeProcessManager.any(),
        fileSystem: MemoryFileSystem.test(),
        operatingSystemUtils: FakeOperatingSystemUtils(),
      );

      expect(device.executablePathForDevice(mockApp, BuildInfo.debug), 'debug/executable');
      expect(device.executablePathForDevice(mockApp, BuildInfo.profile), 'profile/executable');
      expect(device.executablePathForDevice(mockApp, BuildInfo.release), 'release/executable');
    },
  );
}

FlutterProject setUpFlutterProject(Directory directory) {
  final FlutterProjectFactory flutterProjectFactory = FlutterProjectFactory(
    fileSystem: directory.fileSystem,
    logger: BufferLogger.test(),
  );
  return flutterProjectFactory.fromDirectory(directory);
}

class FakeLinuxApp extends Fake implements LinuxApp {
  @override
  String executable(BuildMode buildMode) => switch (buildMode) {
    BuildMode.debug => 'debug/executable',
    BuildMode.profile => 'profile/executable',
    BuildMode.release => 'release/executable',
    _ => throw StateError('Invalid mode: $buildMode'),
  };
}

class FakeOperatingSystemUtils extends Fake implements OperatingSystemUtils {
  FakeOperatingSystemUtils({HostPlatform hostPlatform = HostPlatform.linux_x64})
    : _hostPlatform = hostPlatform;

  final HostPlatform _hostPlatform;

  @override
  String get name => 'Linux';

  @override
  HostPlatform get hostPlatform => _hostPlatform;
}
