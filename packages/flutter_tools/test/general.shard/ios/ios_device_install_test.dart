// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/memory.dart';
import 'package:flutter_tools/src/artifacts.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/io.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/platform.dart';
import 'package:flutter_tools/src/build_info.dart';
import 'package:flutter_tools/src/cache.dart';
import 'package:flutter_tools/src/device.dart';
import 'package:flutter_tools/src/ios/application_package.dart';
import 'package:flutter_tools/src/ios/core_devices.dart';
import 'package:flutter_tools/src/ios/devices.dart';
import 'package:flutter_tools/src/ios/ios_deploy.dart';
import 'package:flutter_tools/src/ios/iproxy.dart';
import 'package:flutter_tools/src/ios/mac.dart';
import 'package:flutter_tools/src/ios/xcode_debug.dart';
import 'package:test/fake.dart';

import '../../src/common.dart';
import '../../src/fake_process_manager.dart';
import '../../src/fakes.dart';

const Map<String, String> kDyLdLibEntry = <String, String>{
  'DYLD_LIBRARY_PATH': '/path/to/libraries',
};

void main() {
  late Artifacts artifacts;
  late String iosDeployPath;
  late FileSystem fileSystem;
  late Directory bundleDirectory;

  setUp(() {
    artifacts = Artifacts.test();
    fileSystem = MemoryFileSystem.test();
    bundleDirectory = fileSystem.directory('bundle');
    iosDeployPath = artifacts.getHostArtifact(HostArtifact.iosDeploy).path;
  });

  testWithoutContext('IOSDevice.installApp calls ios-deploy correctly with USB', () async {
    final IOSApp iosApp = PrebuiltIOSApp(
      projectBundleId: 'app',
      uncompressedBundle: fileSystem.currentDirectory,
      applicationPackage: bundleDirectory,
    );
    final FakeProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
      FakeCommand(
        command: <String>[iosDeployPath, '--id', '1234', '--bundle', '/', '--no-wifi'],
        environment: const <String, String>{'PATH': '/usr/bin:null', ...kDyLdLibEntry},
      ),
    ]);
    final IOSDevice device = setUpIOSDevice(
      processManager: processManager,
      fileSystem: fileSystem,
      interfaceType: DeviceConnectionInterface.attached,
      artifacts: artifacts,
    );
    final bool wasInstalled = await device.installApp(iosApp);

    expect(wasInstalled, true);
    expect(processManager, hasNoRemainingExpectations);
  });

  testWithoutContext('IOSDevice.installApp calls ios-deploy correctly with network', () async {
    final IOSApp iosApp = PrebuiltIOSApp(
      projectBundleId: 'app',
      uncompressedBundle: fileSystem.currentDirectory,
      applicationPackage: bundleDirectory,
    );
    final FakeProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
      FakeCommand(
        command: <String>[iosDeployPath, '--id', '1234', '--bundle', '/'],
        environment: const <String, String>{'PATH': '/usr/bin:null', ...kDyLdLibEntry},
      ),
    ]);
    final IOSDevice device = setUpIOSDevice(
      processManager: processManager,
      fileSystem: fileSystem,
      interfaceType: DeviceConnectionInterface.wireless,
      artifacts: artifacts,
    );
    final bool wasInstalled = await device.installApp(iosApp);

    expect(wasInstalled, true);
    expect(processManager, hasNoRemainingExpectations);
  });

  testWithoutContext('IOSDevice.installApp uses devicectl for CoreDevices', () async {
    final IOSApp iosApp = PrebuiltIOSApp(
      projectBundleId: 'app',
      uncompressedBundle: fileSystem.currentDirectory,
      applicationPackage: bundleDirectory,
    );

    final FakeProcessManager processManager = FakeProcessManager.empty();

    final IOSDevice device = setUpIOSDevice(
      processManager: processManager,
      fileSystem: fileSystem,
      interfaceType: DeviceConnectionInterface.attached,
      artifacts: artifacts,
      isCoreDevice: true,
    );
    final bool wasInstalled = await device.installApp(iosApp);

    expect(wasInstalled, true);
    expect(processManager, hasNoRemainingExpectations);
  });

  testWithoutContext('IOSDevice.uninstallApp calls ios-deploy correctly', () async {
    final IOSApp iosApp = PrebuiltIOSApp(
      projectBundleId: 'app',
      uncompressedBundle: bundleDirectory,
      applicationPackage: bundleDirectory,
    );
    final FakeProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
      FakeCommand(
        command: <String>[iosDeployPath, '--id', '1234', '--uninstall_only', '--bundle_id', 'app'],
        environment: const <String, String>{'PATH': '/usr/bin:null', ...kDyLdLibEntry},
      ),
    ]);
    final IOSDevice device = setUpIOSDevice(processManager: processManager, artifacts: artifacts);
    final bool wasUninstalled = await device.uninstallApp(iosApp);

    expect(wasUninstalled, true);
    expect(processManager, hasNoRemainingExpectations);
  });

  testWithoutContext('IOSDevice.uninstallApp uses devicectl for CoreDevices', () async {
    final IOSApp iosApp = PrebuiltIOSApp(
      projectBundleId: 'app',
      uncompressedBundle: fileSystem.currentDirectory,
      applicationPackage: bundleDirectory,
    );

    final FakeProcessManager processManager = FakeProcessManager.empty();

    final IOSDevice device = setUpIOSDevice(
      processManager: processManager,
      fileSystem: fileSystem,
      interfaceType: DeviceConnectionInterface.attached,
      artifacts: artifacts,
      isCoreDevice: true,
    );
    final bool wasUninstalled = await device.uninstallApp(iosApp);

    expect(wasUninstalled, true);
    expect(processManager, hasNoRemainingExpectations);
  });

  group('isAppInstalled', () {
    testWithoutContext('catches ProcessException from ios-deploy', () async {
      final IOSApp iosApp = PrebuiltIOSApp(
        projectBundleId: 'app',
        uncompressedBundle: bundleDirectory,
        applicationPackage: bundleDirectory,
      );
      final FakeProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
        FakeCommand(
          command: <String>[
            iosDeployPath,
            '--id',
            '1234',
            '--exists',
            '--timeout',
            '10',
            '--bundle_id',
            'app',
          ],
          environment: const <String, String>{'PATH': '/usr/bin:null', ...kDyLdLibEntry},
          exception: const ProcessException('ios-deploy', <String>[]),
        ),
      ]);
      final IOSDevice device = setUpIOSDevice(processManager: processManager, artifacts: artifacts);
      final bool isAppInstalled = await device.isAppInstalled(iosApp);

      expect(isAppInstalled, false);
      expect(processManager, hasNoRemainingExpectations);
    });

    testWithoutContext('returns true when app is installed', () async {
      final IOSApp iosApp = PrebuiltIOSApp(
        projectBundleId: 'app',
        uncompressedBundle: bundleDirectory,
        applicationPackage: bundleDirectory,
      );
      final FakeProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
        FakeCommand(
          command: <String>[
            iosDeployPath,
            '--id',
            '1234',
            '--exists',
            '--timeout',
            '10',
            '--bundle_id',
            'app',
          ],
          environment: const <String, String>{'PATH': '/usr/bin:null', ...kDyLdLibEntry},
        ),
      ]);
      final IOSDevice device = setUpIOSDevice(processManager: processManager, artifacts: artifacts);
      final bool isAppInstalled = await device.isAppInstalled(iosApp);

      expect(isAppInstalled, isTrue);
      expect(processManager, hasNoRemainingExpectations);
    });

    testWithoutContext('returns false when app is not installed', () async {
      final IOSApp iosApp = PrebuiltIOSApp(
        projectBundleId: 'app',
        uncompressedBundle: bundleDirectory,
        applicationPackage: bundleDirectory,
      );
      final FakeProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
        FakeCommand(
          command: <String>[
            iosDeployPath,
            '--id',
            '1234',
            '--exists',
            '--timeout',
            '10',
            '--bundle_id',
            'app',
          ],
          environment: const <String, String>{'PATH': '/usr/bin:null', ...kDyLdLibEntry},
          exitCode: 255,
        ),
      ]);
      final BufferLogger logger = BufferLogger.test();
      final IOSDevice device = setUpIOSDevice(
        processManager: processManager,
        logger: logger,
        artifacts: artifacts,
      );
      final bool isAppInstalled = await device.isAppInstalled(iosApp);

      expect(isAppInstalled, isFalse);
      expect(processManager, hasNoRemainingExpectations);
      expect(logger.traceText, contains('${iosApp.id} not installed on ${device.id}'));
    });

    testWithoutContext('returns false on command timeout or other error', () async {
      final IOSApp iosApp = PrebuiltIOSApp(
        projectBundleId: 'app',
        uncompressedBundle: bundleDirectory,
        applicationPackage: bundleDirectory,
      );
      const String stderr =
          '2020-03-26 17:48:43.484 ios-deploy[21518:5501783] [ !! ] Timed out waiting for device';
      final FakeProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
        FakeCommand(
          command: <String>[
            iosDeployPath,
            '--id',
            '1234',
            '--exists',
            '--timeout',
            '10',
            '--bundle_id',
            'app',
          ],
          environment: const <String, String>{'PATH': '/usr/bin:null', ...kDyLdLibEntry},
          stderr: stderr,
          exitCode: 253,
        ),
      ]);
      final BufferLogger logger = BufferLogger.test();
      final IOSDevice device = setUpIOSDevice(
        processManager: processManager,
        logger: logger,
        artifacts: artifacts,
      );
      final bool isAppInstalled = await device.isAppInstalled(iosApp);

      expect(isAppInstalled, isFalse);
      expect(processManager, hasNoRemainingExpectations);
      expect(logger.traceText, contains(stderr));
    });

    testWithoutContext('uses devicectl for CoreDevices', () async {
      final IOSApp iosApp = PrebuiltIOSApp(
        projectBundleId: 'app',
        uncompressedBundle: fileSystem.currentDirectory,
        applicationPackage: bundleDirectory,
      );

      final FakeProcessManager processManager = FakeProcessManager.empty();

      final IOSDevice device = setUpIOSDevice(
        processManager: processManager,
        fileSystem: fileSystem,
        interfaceType: DeviceConnectionInterface.attached,
        artifacts: artifacts,
        isCoreDevice: true,
      );
      final bool wasInstalled = await device.isAppInstalled(iosApp);

      expect(wasInstalled, true);
      expect(processManager, hasNoRemainingExpectations);
    });
  });

  testWithoutContext('IOSDevice.installApp catches ProcessException from ios-deploy', () async {
    final IOSApp iosApp = PrebuiltIOSApp(
      projectBundleId: 'app',
      uncompressedBundle: fileSystem.currentDirectory,
      applicationPackage: bundleDirectory,
    );
    final FakeProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
      FakeCommand(
        command: <String>[iosDeployPath, '--id', '1234', '--bundle', '/', '--no-wifi'],
        environment: const <String, String>{'PATH': '/usr/bin:null', ...kDyLdLibEntry},
        exception: const ProcessException('ios-deploy', <String>[]),
      ),
    ]);
    final IOSDevice device = setUpIOSDevice(processManager: processManager, artifacts: artifacts);
    final bool wasAppInstalled = await device.installApp(iosApp);

    expect(wasAppInstalled, false);
  });

  testWithoutContext('IOSDevice.uninstallApp catches ProcessException from ios-deploy', () async {
    final IOSApp iosApp = PrebuiltIOSApp(
      projectBundleId: 'app',
      uncompressedBundle: bundleDirectory,
      applicationPackage: bundleDirectory,
    );
    final FakeProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
      FakeCommand(
        command: <String>[iosDeployPath, '--id', '1234', '--uninstall_only', '--bundle_id', 'app'],
        environment: const <String, String>{'PATH': '/usr/bin:null', ...kDyLdLibEntry},
        exception: const ProcessException('ios-deploy', <String>[]),
      ),
    ]);
    final IOSDevice device = setUpIOSDevice(processManager: processManager, artifacts: artifacts);
    final bool wasAppUninstalled = await device.uninstallApp(iosApp);

    expect(wasAppUninstalled, false);
  });
}

IOSDevice setUpIOSDevice({
  required ProcessManager processManager,
  FileSystem? fileSystem,
  Logger? logger,
  DeviceConnectionInterface? interfaceType,
  Artifacts? artifacts,
  bool isCoreDevice = false,
}) {
  logger ??= BufferLogger.test();
  final FakePlatform platform = FakePlatform(
    operatingSystem: 'macos',
    environment: <String, String>{},
  );
  artifacts ??= Artifacts.test();
  final Cache cache = Cache.test(
    platform: platform,
    artifacts: <ArtifactSet>[FakeDyldEnvironmentArtifact()],
    processManager: FakeProcessManager.any(),
  );
  return IOSDevice(
    '1234',
    name: 'iPhone 1',
    logger: logger,
    fileSystem: fileSystem ?? MemoryFileSystem.test(),
    sdkVersion: '13.3',
    cpuArchitecture: DarwinArch.arm64,
    platform: platform,
    iMobileDevice: IMobileDevice(
      logger: logger,
      processManager: processManager,
      artifacts: artifacts,
      cache: cache,
    ),
    iosDeploy: IOSDeploy(
      logger: logger,
      platform: platform,
      processManager: processManager,
      artifacts: artifacts,
      cache: cache,
    ),
    coreDeviceControl: FakeIOSCoreDeviceControl(),
    xcodeDebug: FakeXcodeDebug(),
    iProxy: IProxy.test(logger: logger, processManager: processManager),
    connectionInterface: interfaceType ?? DeviceConnectionInterface.attached,
    isConnected: true,
    isPaired: true,
    devModeEnabled: true,
    isCoreDevice: isCoreDevice,
  );
}

class FakeXcodeDebug extends Fake implements XcodeDebug {}

class FakeIOSCoreDeviceControl extends Fake implements IOSCoreDeviceControl {
  @override
  Future<bool> installApp({required String deviceId, required String bundlePath}) async {
    return true;
  }

  @override
  Future<bool> uninstallApp({required String deviceId, required String bundleId}) async {
    return true;
  }

  @override
  Future<bool> isAppInstalled({required String deviceId, required String bundleId}) async {
    return true;
  }
}
