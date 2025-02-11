// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:file/file.dart';
import 'package:flutter_tools/src/application_package.dart';
import 'package:flutter_tools/src/base/dds.dart';
import 'package:flutter_tools/src/base/io.dart' as io;
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/platform.dart';
import 'package:flutter_tools/src/base/process.dart';
import 'package:flutter_tools/src/build_info.dart';
import 'package:flutter_tools/src/device.dart';
import 'package:flutter_tools/src/drive/drive_service.dart';
import 'package:flutter_tools/src/project.dart';
import 'package:flutter_tools/src/resident_runner.dart';
import 'package:flutter_tools/src/vmservice.dart';
import 'package:package_config/package_config_types.dart';
import 'package:test/fake.dart';
import 'package:vm_service/vm_service.dart' as vm_service;

import '../../src/common.dart';
import '../../src/context.dart';
import '../../src/fake_vm_services.dart';
import '../../src/fakes.dart';

final vm_service.Isolate fakeUnpausedIsolate = vm_service.Isolate(
  id: '1',
  pauseEvent: vm_service.Event(kind: vm_service.EventKind.kResume, timestamp: 0),
  breakpoints: <vm_service.Breakpoint>[],
  libraries: <vm_service.LibraryRef>[
    vm_service.LibraryRef(id: '1', uri: 'file:///hello_world/main.dart', name: ''),
  ],
  livePorts: 0,
  name: 'test',
  number: '1',
  pauseOnExit: false,
  runnable: true,
  startTime: 0,
  isSystemIsolate: false,
  isolateFlags: <vm_service.IsolateFlag>[],
);

final vm_service.VM fakeVM = vm_service.VM(
  isolates: <vm_service.IsolateRef>[fakeUnpausedIsolate],
  pid: 1,
  hostCPU: '',
  isolateGroups: <vm_service.IsolateGroupRef>[],
  targetCPU: '',
  startTime: 0,
  name: 'dart',
  architectureBits: 64,
  operatingSystem: '',
  version: '',
  systemIsolateGroups: <vm_service.IsolateGroupRef>[],
  systemIsolates: <vm_service.IsolateRef>[],
);

final FakeVmServiceRequest getVM = FakeVmServiceRequest(
  method: 'getVM',
  args: <String, Object>{},
  jsonResponse: fakeVM.toJson(),
);

void main() {
  testWithoutContext('Exits if device fails to start', () {
    final DriverService driverService = setUpDriverService();
    final Device device = FakeDevice(LaunchResult.failed());

    expect(
      () => driverService.start(
        BuildInfo.profile,
        device,
        DebuggingOptions.enabled(BuildInfo.profile, ipv6: true),
      ),
      throwsToolExit(message: 'Application failed to start. Will not run test. Quitting.'),
    );
  });

  testWithoutContext('Retries application launch if it fails the first time', () async {
    final FakeVmServiceHost fakeVmServiceHost = FakeVmServiceHost(
      requests: <FakeVmServiceRequest>[getVM],
    );
    final FakeProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
      const FakeCommand(
        command: <String>['dart', '--enable-experiment=non-nullable', 'foo.test'],
        exitCode: 23,
        environment: <String, String>{
          'FOO': 'BAR',
          'VM_SERVICE_URL': 'http://127.0.0.1:1234/', // dds forwarded URI
        },
      ),
    ]);
    final DriverService driverService = setUpDriverService(
      processManager: processManager,
      vmService: fakeVmServiceHost.vmService,
    );
    final Device device = FakeDevice(
      LaunchResult.succeeded(vmServiceUri: Uri.parse('http://127.0.0.1:63426/1UasC_ihpXY=/')),
    )..failOnce = true;

    await expectLater(
      () async => driverService.start(
        BuildInfo.profile,
        device,
        DebuggingOptions.enabled(BuildInfo.profile, ipv6: true),
      ),
      returnsNormally,
    );
  });

  testWithoutContext('Connects to device VM Service and runs test application', () async {
    final FakeVmServiceHost fakeVmServiceHost = FakeVmServiceHost(
      requests: <FakeVmServiceRequest>[getVM],
    );
    final FakeProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
      const FakeCommand(
        command: <String>['dart', '--enable-experiment=non-nullable', 'foo.test'],
        exitCode: 23,
        environment: <String, String>{
          'FOO': 'BAR',
          'VM_SERVICE_URL': 'http://127.0.0.1:1234/', // dds forwarded URI
        },
      ),
    ]);
    final DriverService driverService = setUpDriverService(
      processManager: processManager,
      vmService: fakeVmServiceHost.vmService,
      platform: FakePlatform(environment: <String, String>{'FOO': 'BAR'}),
    );
    final Device device = FakeDevice(
      LaunchResult.succeeded(vmServiceUri: Uri.parse('http://127.0.0.1:63426/1UasC_ihpXY=/')),
    );

    await driverService.start(
      BuildInfo.profile,
      device,
      DebuggingOptions.enabled(BuildInfo.profile, ipv6: true),
    );
    final int testResult = await driverService.startTest('foo.test', <String>[
      '--enable-experiment=non-nullable',
    ], PackageConfig(<Package>[Package('test', Uri.base)]));

    expect(testResult, 23);
  });

  testWithoutContext(
    'Connects to device VM Service and runs test application with devtools memory profile',
    () async {
      final FakeVmServiceHost fakeVmServiceHost = FakeVmServiceHost(
        requests: <FakeVmServiceRequest>[getVM],
      );
      final FakeProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
        const FakeCommand(
          command: <String>['dart', '--enable-experiment=non-nullable', 'foo.test'],
          exitCode: 23,
          environment: <String, String>{
            'FOO': 'BAR',
            'VM_SERVICE_URL': 'http://127.0.0.1:1234/', // dds forwarded URI
          },
        ),
      ]);
      final FakeDevtoolsLauncher launcher = FakeDevtoolsLauncher();
      final DriverService driverService = setUpDriverService(
        processManager: processManager,
        vmService: fakeVmServiceHost.vmService,
        devtoolsLauncher: launcher,
        platform: FakePlatform(environment: <String, String>{'FOO': 'BAR'}),
      );
      final Device device = FakeDevice(
        LaunchResult.succeeded(vmServiceUri: Uri.parse('http://127.0.0.1:63426/1UasC_ihpXY=/')),
      );

      await driverService.start(
        BuildInfo.profile,
        device,
        DebuggingOptions.enabled(BuildInfo.profile, ipv6: true),
      );
      final int testResult = await driverService.startTest(
        'foo.test',
        <String>['--enable-experiment=non-nullable'],
        PackageConfig(<Package>[Package('test', Uri.base)]),
        profileMemory: 'devtools_memory.json',
      );

      expect(launcher.closed, true);
      expect(testResult, 23);
    },
  );

  testWithoutContext(
    'Uses dart to execute the test if there is no package:test dependency',
    () async {
      final FakeVmServiceHost fakeVmServiceHost = FakeVmServiceHost(
        requests: <FakeVmServiceRequest>[getVM],
      );
      final FakeProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
        const FakeCommand(
          command: <String>['dart', '--enable-experiment=non-nullable', 'foo.test'],
          exitCode: 23,
          environment: <String, String>{
            'FOO': 'BAR',
            'VM_SERVICE_URL': 'http://127.0.0.1:1234/', // dds forwarded URI
          },
        ),
      ]);
      final DriverService driverService = setUpDriverService(
        processManager: processManager,
        vmService: fakeVmServiceHost.vmService,
        platform: FakePlatform(environment: <String, String>{'FOO': 'BAR'}),
      );
      final Device device = FakeDevice(
        LaunchResult.succeeded(vmServiceUri: Uri.parse('http://127.0.0.1:63426/1UasC_ihpXY=/')),
      );

      await driverService.start(
        BuildInfo.profile,
        device,
        DebuggingOptions.enabled(BuildInfo.profile, ipv6: true),
      );
      final int testResult = await driverService.startTest('foo.test', <String>[
        '--enable-experiment=non-nullable',
      ], PackageConfig.empty);

      expect(testResult, 23);
    },
  );

  testWithoutContext(
    'Connects to device VM Service and runs test application without dds',
    () async {
      final FakeVmServiceHost fakeVmServiceHost = FakeVmServiceHost(
        requests: <FakeVmServiceRequest>[getVM],
      );
      final FakeProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
        const FakeCommand(
          command: <String>['dart', 'foo.test'],
          exitCode: 11,
          environment: <String, String>{'VM_SERVICE_URL': 'http://127.0.0.1:63426/1UasC_ihpXY=/'},
        ),
      ]);
      final DriverService driverService = setUpDriverService(
        processManager: processManager,
        vmService: fakeVmServiceHost.vmService,
      );
      final Device device = FakeDevice(
        LaunchResult.succeeded(vmServiceUri: Uri.parse('http://127.0.0.1:63426/1UasC_ihpXY=/')),
      );
      final FakeDartDevelopmentService dds = device.dds as FakeDartDevelopmentService;

      expect(dds.started, false);
      await driverService.start(
        BuildInfo.profile,
        device,
        DebuggingOptions.enabled(BuildInfo.profile, enableDds: false, ipv6: true),
      );
      expect(dds.started, false);

      final int testResult = await driverService.startTest(
        'foo.test',
        <String>[],
        PackageConfig(<Package>[Package('test', Uri.base)]),
      );

      expect(testResult, 11);
      expect(dds.started, false);
    },
  );

  testWithoutContext('Safely stops and uninstalls application', () async {
    final FakeVmServiceHost fakeVmServiceHost = FakeVmServiceHost(
      requests: <FakeVmServiceRequest>[getVM],
    );
    final FakeProcessManager processManager = FakeProcessManager.empty();
    final DriverService driverService = setUpDriverService(
      processManager: processManager,
      vmService: fakeVmServiceHost.vmService,
    );
    final FakeDevice device = FakeDevice(
      LaunchResult.succeeded(vmServiceUri: Uri.parse('http://127.0.0.1:63426/1UasC_ihpXY=/')),
    );

    await driverService.start(
      BuildInfo.profile,
      device,
      DebuggingOptions.enabled(BuildInfo.profile, ipv6: true),
    );
    await driverService.stop();

    expect(device.didStopApp, true);
    expect(device.didUninstallApp, true);
    expect(device.didDispose, true);
  });

  testWithoutContext('Can connect to existing application and stop it during cleanup', () async {
    final FakeVmServiceHost fakeVmServiceHost = FakeVmServiceHost(
      requests: <FakeVmServiceRequest>[
        getVM,
        const FakeVmServiceRequest(
          method: 'ext.flutter.exit',
          args: <String, Object>{'isolateId': '1'},
        ),
      ],
    );
    final FakeProcessManager processManager = FakeProcessManager.empty();
    final DriverService driverService = setUpDriverService(
      processManager: processManager,
      vmService: fakeVmServiceHost.vmService,
    );
    final FakeDevice device = FakeDevice(LaunchResult.failed());

    await driverService.reuseApplication(
      Uri.parse('http://127.0.0.1:63426/1UasC_ihpXY=/'),
      device,
      DebuggingOptions.enabled(BuildInfo.debug),
    );
    await driverService.stop();
  });

  testWithoutContext('Can connect to existing application using ws URI', () async {
    final FakeVmServiceHost fakeVmServiceHost = FakeVmServiceHost(
      requests: <FakeVmServiceRequest>[
        getVM,
        const FakeVmServiceRequest(
          method: 'ext.flutter.exit',
          args: <String, Object>{'isolateId': '1'},
        ),
      ],
    );
    final FakeProcessManager processManager = FakeProcessManager.empty();
    final DriverService driverService = setUpDriverService(
      processManager: processManager,
      vmService: fakeVmServiceHost.vmService,
    );
    final FakeDevice device = FakeDevice(LaunchResult.failed());

    await driverService.reuseApplication(
      Uri.parse('ws://127.0.0.1:63426/1UasC_ihpXY=/ws/'),
      device,
      DebuggingOptions.enabled(BuildInfo.debug),
    );
    await driverService.stop();
  });

  testWithoutContext(
    'Can connect to existing application using ws URI (no trailing slash)',
    () async {
      final FakeVmServiceHost fakeVmServiceHost = FakeVmServiceHost(
        requests: <FakeVmServiceRequest>[
          getVM,
          const FakeVmServiceRequest(
            method: 'ext.flutter.exit',
            args: <String, Object>{'isolateId': '1'},
          ),
        ],
      );
      final FakeProcessManager processManager = FakeProcessManager.empty();
      final DriverService driverService = setUpDriverService(
        processManager: processManager,
        vmService: fakeVmServiceHost.vmService,
      );
      final FakeDevice device = FakeDevice(LaunchResult.failed());

      await driverService.reuseApplication(
        Uri.parse('ws://127.0.0.1:63426/1UasC_ihpXY=/ws'),
        device,
        DebuggingOptions.enabled(BuildInfo.debug),
      );
      await driverService.stop();
    },
  );

  testWithoutContext(
    'Can connect to existing application using ws URI (no trailing slash, ws in auth code)',
    () async {
      final FakeVmServiceHost fakeVmServiceHost = FakeVmServiceHost(
        requests: <FakeVmServiceRequest>[
          getVM,
          const FakeVmServiceRequest(
            method: 'ext.flutter.exit',
            args: <String, Object>{'isolateId': '1'},
          ),
        ],
      );
      final FakeProcessManager processManager = FakeProcessManager.empty();
      final DriverService driverService = setUpDriverService(
        processManager: processManager,
        vmService: fakeVmServiceHost.vmService,
      );
      final FakeDevice device = FakeDevice(LaunchResult.failed());

      await driverService.reuseApplication(
        Uri.parse('ws://127.0.0.1:63426/wsasC_ihpXY=/ws'),
        device,
        DebuggingOptions.enabled(BuildInfo.debug),
      );
      await driverService.stop();
    },
  );

  testWithoutContext('Does not call flutterExit on device types that do not support it', () async {
    final FakeVmServiceHost fakeVmServiceHost = FakeVmServiceHost(
      requests: <FakeVmServiceRequest>[getVM],
    );
    final FakeProcessManager processManager = FakeProcessManager.empty();
    final DriverService driverService = setUpDriverService(
      processManager: processManager,
      vmService: fakeVmServiceHost.vmService,
    );
    final FakeDevice device = FakeDevice(LaunchResult.failed(), supportsFlutterExit: false);

    await driverService.reuseApplication(
      Uri.parse('http://127.0.0.1:63426/1UasC_ihpXY=/'),
      device,
      DebuggingOptions.enabled(BuildInfo.debug),
    );
    await driverService.stop();
  });
}

FlutterDriverService setUpDriverService({
  Logger? logger,
  Platform? platform,
  ProcessManager? processManager,
  FlutterVmService? vmService,
  DevtoolsLauncher? devtoolsLauncher,
}) {
  logger ??= BufferLogger.test();
  return FlutterDriverService(
    applicationPackageFactory: FakeApplicationPackageFactory(FakeApplicationPackage()),
    logger: logger,
    platform: platform ?? FakePlatform(),
    processUtils: ProcessUtils(
      logger: logger,
      processManager: processManager ?? FakeProcessManager.any(),
    ),
    dartSdkPath: 'dart',
    devtoolsLauncher: devtoolsLauncher ?? FakeDevtoolsLauncher(),
    vmServiceConnector: (
      Uri httpUri, {
      ReloadSources? reloadSources,
      Restart? restart,
      CompileExpression? compileExpression,
      FlutterProject? flutterProject,
      PrintStructuredErrorLogMethod? printStructuredErrorLogMethod,
      io.CompressionOptions compression = io.CompressionOptions.compressionDefault,
      Device? device,
      required Logger logger,
    }) async {
      if (httpUri.scheme != 'http') {
        fail('Expected an HTTP scheme, found $httpUri');
      }
      if (httpUri.path.endsWith('/ws')) {
        fail('Expected HTTP uri to not contain `/ws`, found $httpUri');
      }
      return vmService!;
    },
  );
}

class FakeApplicationPackageFactory extends Fake implements ApplicationPackageFactory {
  FakeApplicationPackageFactory(this.applicationPackage);

  ApplicationPackage applicationPackage;

  @override
  Future<ApplicationPackage> getPackageForPlatform(
    TargetPlatform platform, {
    BuildInfo? buildInfo,
    File? applicationBinary,
  }) async => applicationPackage;
}

class FakeApplicationPackage extends Fake implements ApplicationPackage {}

class FakeDevice extends Fake implements Device {
  FakeDevice(this.result, {this.supportsFlutterExit = true});

  LaunchResult result;
  bool didStopApp = false;
  bool didUninstallApp = false;
  bool didDispose = false;
  bool failOnce = false;
  @override
  final PlatformType platformType = PlatformType.web;

  @override
  String get name => 'test';

  @override
  final bool supportsFlutterExit;

  @override
  final DartDevelopmentService dds = FakeDartDevelopmentService();

  @override
  Future<TargetPlatform> get targetPlatform async => TargetPlatform.android_arm;

  @override
  Future<DeviceLogReader> getLogReader({
    ApplicationPackage? app,
    bool includePastLogs = false,
  }) async => NoOpDeviceLogReader('test');

  @override
  Future<LaunchResult> startApp(
    ApplicationPackage? package, {
    String? mainPath,
    String? route,
    required DebuggingOptions debuggingOptions,
    Map<String, Object?> platformArgs = const <String, Object?>{},
    bool prebuiltApplication = false,
    bool ipv6 = false,
    String? userIdentifier,
  }) async {
    if (failOnce) {
      failOnce = false;
      return LaunchResult.failed();
    }
    return result;
  }

  @override
  Future<bool> stopApp(ApplicationPackage? app, {String? userIdentifier}) async {
    didStopApp = true;
    return true;
  }

  @override
  Future<bool> uninstallApp(ApplicationPackage app, {String? userIdentifier}) async {
    didUninstallApp = true;
    return true;
  }

  @override
  Future<void> dispose() async {
    didDispose = true;
  }
}

class FakeDartDevelopmentService extends Fake
    with DartDevelopmentServiceLocalOperationsMixin
    implements DartDevelopmentService {
  bool started = false;
  bool disposed = false;

  @override
  final Uri uri = Uri.parse('http://127.0.0.1:1234/');

  @override
  Future<void> startDartDevelopmentService(
    Uri vmServiceUri, {
    FlutterDevice? device,
    int? ddsPort,
    bool? ipv6,
    bool? disableServiceAuthCodes,
    bool enableDevTools = false,
    bool cacheStartupProfile = false,
    String? google3WorkspaceRoot,
    Uri? devToolsServerAddress,
  }) async {
    started = true;
  }

  @override
  Future<void> shutdown() async {
    disposed = true;
  }
}
