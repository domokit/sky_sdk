// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/memory.dart';
import 'package:flutter_tools/src/artifacts.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/platform.dart';
import 'package:flutter_tools/src/build_info.dart';
import 'package:flutter_tools/src/cache.dart';
import 'package:flutter_tools/src/device.dart';
import 'package:flutter_tools/src/ios/core_devices.dart';
import 'package:flutter_tools/src/ios/devices.dart';
import 'package:flutter_tools/src/ios/ios_deploy.dart';
import 'package:flutter_tools/src/ios/iproxy.dart';
import 'package:flutter_tools/src/ios/mac.dart';
import 'package:flutter_tools/src/ios/xcode_debug.dart';
import 'package:flutter_tools/src/project.dart';
import 'package:test/fake.dart';

import '../../src/common.dart';
import '../../src/context.dart';

// FlutterProject still depends on context.
void main() {
  late FileSystem fileSystem;

  // This setup is required to inject the context.
  setUp(() {
    fileSystem = MemoryFileSystem.test();
  });

  testUsingContext(
    'IOSDevice.isSupportedForProject is true on module project',
    () async {
      fileSystem.file('pubspec.yaml')
        ..createSync()
        ..writeAsStringSync(r'''
name: example
flutter:
  module: {}
  ''');
      final FlutterProject flutterProject = FlutterProject.fromDirectory(
        fileSystem.currentDirectory,
      );
      final IOSDevice device = setUpIOSDevice(fileSystem);

      expect(device.isSupportedForProject(flutterProject), true);
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => FakeProcessManager.any(),
    },
  );

  testUsingContext(
    'IOSDevice.isSupportedForProject is true with editable host app',
    () async {
      final FileSystem fileSystem = MemoryFileSystem.test();
      fileSystem.file('pubspec.yaml').createSync();
      fileSystem.directory('ios').createSync();
      final FlutterProject flutterProject = FlutterProject.fromDirectory(
        fileSystem.currentDirectory,
      );
      final IOSDevice device = setUpIOSDevice(fileSystem);

      expect(device.isSupportedForProject(flutterProject), true);
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => FakeProcessManager.any(),
    },
  );

  testUsingContext(
    'IOSDevice.isSupportedForProject is false with no host app and no module',
    () async {
      final FileSystem fileSystem = MemoryFileSystem.test();
      fileSystem.file('pubspec.yaml').createSync();
      final FlutterProject flutterProject = FlutterProject.fromDirectory(
        fileSystem.currentDirectory,
      );
      final IOSDevice device = setUpIOSDevice(fileSystem);

      expect(device.isSupportedForProject(flutterProject), false);
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => FakeProcessManager.any(),
    },
  );
}

IOSDevice setUpIOSDevice(FileSystem fileSystem) {
  final Platform platform = FakePlatform(operatingSystem: 'macos');
  final Logger logger = BufferLogger.test();
  final ProcessManager processManager = FakeProcessManager.any();
  return IOSDevice(
    'test',
    fileSystem: fileSystem,
    logger: logger,
    iosDeploy: IOSDeploy(
      platform: platform,
      logger: logger,
      processManager: processManager,
      artifacts: Artifacts.test(),
      cache: Cache.test(processManager: processManager),
    ),
    iMobileDevice: IMobileDevice.test(processManager: processManager),
    coreDeviceControl: FakeIOSCoreDeviceControl(),
    xcodeDebug: FakeXcodeDebug(),
    platform: platform,
    name: 'iPhone 1',
    sdkVersion: '13.3',
    cpuArchitecture: DarwinArch.arm64,
    iProxy: IProxy.test(logger: logger, processManager: processManager),
    connectionInterface: DeviceConnectionInterface.attached,
    isConnected: true,
    isPaired: true,
    devModeEnabled: true,
    isCoreDevice: false,
  );
}

class FakeXcodeDebug extends Fake implements XcodeDebug {}

class FakeIOSCoreDeviceControl extends Fake implements IOSCoreDeviceControl {}
