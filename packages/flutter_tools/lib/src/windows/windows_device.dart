// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:process/process.dart';

import '../base/file_system.dart';
import '../base/logger.dart';
import '../base/os.dart';
import '../build_info.dart';
import '../desktop_device.dart';
import '../device.dart';
import '../project.dart';
import 'application_package.dart';
import 'build_windows.dart';
import 'windows_workflow.dart';

/// A device that represents a desktop Windows target.
class WindowsDevice extends DesktopDevice {
  WindowsDevice({
    required ProcessManager processManager,
    required Logger logger,
    required FileSystem fileSystem,
    required OperatingSystemUtils operatingSystemUtils,
  }) : _operatingSystemUtils = operatingSystemUtils,
       super(
         'windows',
         platformType: PlatformType.windows,
         ephemeral: false,
         processManager: processManager,
         logger: logger,
         fileSystem: fileSystem,
         operatingSystemUtils: operatingSystemUtils,
       );

  final OperatingSystemUtils _operatingSystemUtils;

  @override
  bool isSupported() => true;

  @override
  String get name => 'Windows';

  @override
  Future<TargetPlatform> get targetPlatform async => _targetPlatform;

  TargetPlatform get _targetPlatform => switch (_operatingSystemUtils.hostPlatform) {
    HostPlatform.windows_arm64 => TargetPlatform.windows_arm64,
    _ => TargetPlatform.windows_x64,
  };

  @override
  bool isSupportedForProject(FlutterProject flutterProject) {
    return flutterProject.windows.existsSync();
  }

  @override
  Future<void> buildForDevice({
    String? mainPath,
    required BuildInfo buildInfo,
    bool usingCISystem = false,
  }) async {
    await buildWindows(
      FlutterProject.current().windows,
      buildInfo,
      _targetPlatform,
      target: mainPath,
    );
  }

  @override
  String executablePathForDevice(covariant WindowsApp package, BuildInfo buildInfo) {
    return package.executable(buildInfo.mode, _targetPlatform);
  }
}

class WindowsDevices extends PollingDeviceDiscovery {
  WindowsDevices({
    required ProcessManager processManager,
    required Logger logger,
    required FileSystem fileSystem,
    required OperatingSystemUtils operatingSystemUtils,
    required WindowsWorkflow windowsWorkflow,
  }) : _fileSystem = fileSystem,
       _logger = logger,
       _processManager = processManager,
       _operatingSystemUtils = operatingSystemUtils,
       _windowsWorkflow = windowsWorkflow,
       super('windows devices');

  final FileSystem _fileSystem;
  final Logger _logger;
  final ProcessManager _processManager;
  final OperatingSystemUtils _operatingSystemUtils;
  final WindowsWorkflow _windowsWorkflow;

  @override
  bool get supportsPlatform => _windowsWorkflow.appliesToHostPlatform;

  @override
  bool get canListAnything => _windowsWorkflow.canListDevices;

  @override
  Future<List<Device>> pollingGetDevices({Duration? timeout}) async {
    if (!canListAnything) {
      return const <Device>[];
    }
    return <Device>[
      WindowsDevice(
        fileSystem: _fileSystem,
        logger: _logger,
        processManager: _processManager,
        operatingSystemUtils: _operatingSystemUtils,
      ),
    ];
  }

  @override
  Future<List<String>> getDiagnostics() async => const <String>[];

  @override
  List<String> get wellKnownIds => const <String>['windows'];
}
