// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:convert';

import 'package:fake_async/fake_async.dart';
import 'package:file/memory.dart';
import 'package:flutter_tools/src/artifacts.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/platform.dart';
import 'package:flutter_tools/src/base/process.dart';
import 'package:flutter_tools/src/base/template.dart';
import 'package:flutter_tools/src/build_info.dart';
import 'package:flutter_tools/src/cache.dart';
import 'package:flutter_tools/src/device.dart';
import 'package:flutter_tools/src/device_port_forwarder.dart';
import 'package:flutter_tools/src/ios/application_package.dart';
import 'package:flutter_tools/src/ios/core_devices.dart';
import 'package:flutter_tools/src/ios/devices.dart';
import 'package:flutter_tools/src/ios/ios_deploy.dart';
import 'package:flutter_tools/src/ios/iproxy.dart';
import 'package:flutter_tools/src/ios/mac.dart';
import 'package:flutter_tools/src/ios/xcode_debug.dart';
import 'package:flutter_tools/src/mdns_discovery.dart';
import 'package:test/fake.dart';

import '../../src/common.dart';
import '../../src/context.dart';
import '../../src/fake_devices.dart';
import '../../src/fake_process_manager.dart';
import '../../src/fakes.dart';

// The command used to actually launch the app with args in release/profile.
const FakeCommand kLaunchReleaseCommand = FakeCommand(
  command: <String>[
    'HostArtifact.iosDeploy',
    '--id',
    '123',
    '--bundle',
    '/',
    '--no-wifi',
    '--justlaunch',
    // These args are the default on DebuggingOptions.
    '--args',
    '--enable-dart-profiling',
  ],
  environment: <String, String>{'PATH': '/usr/bin:null', 'DYLD_LIBRARY_PATH': '/path/to/libraries'},
);

// The command used to just launch the app with args in debug.
const FakeCommand kLaunchDebugCommand = FakeCommand(
  command: <String>[
    'HostArtifact.iosDeploy',
    '--id',
    '123',
    '--bundle',
    '/',
    '--no-wifi',
    '--justlaunch',
    '--args',
    '--enable-dart-profiling --enable-checked-mode --verify-entry-points',
  ],
  environment: <String, String>{'PATH': '/usr/bin:null', 'DYLD_LIBRARY_PATH': '/path/to/libraries'},
);

// The command used to actually launch the app and attach the debugger with args in debug.
FakeCommand attachDebuggerCommand({
  IOSink? stdin,
  String stdout = '(lldb)     run\nsuccess',
  Completer<void>? completer,
  bool isWirelessDevice = false,
  bool uninstallFirst = false,
  bool skipInstall = false,
}) {
  return FakeCommand(
    command: <String>[
      'script',
      '-t',
      '0',
      '/dev/null',
      'HostArtifact.iosDeploy',
      '--id',
      '123',
      '--bundle',
      '/',
      if (uninstallFirst) '--uninstall',
      if (skipInstall) '--noinstall',
      '--debug',
      if (!isWirelessDevice) '--no-wifi',
      '--args',
      if (isWirelessDevice)
        '--enable-dart-profiling --enable-checked-mode --verify-entry-points --vm-service-host=0.0.0.0'
      else
        '--enable-dart-profiling --enable-checked-mode --verify-entry-points',
    ],
    completer: completer,
    environment: const <String, String>{
      'PATH': '/usr/bin:null',
      'DYLD_LIBRARY_PATH': '/path/to/libraries',
    },
    stdout: stdout,
    stdin: stdin,
  );
}

void main() {
  testWithoutContext('disposing device disposes the portForwarder and logReader', () async {
    final IOSDevice device = setUpIOSDevice();
    final FakeDevicePortForwarder devicePortForwarder = FakeDevicePortForwarder();
    final FakeDeviceLogReader deviceLogReader = FakeDeviceLogReader();
    final IOSApp iosApp = PrebuiltIOSApp(
      projectBundleId: 'app',
      bundleName: 'Runner',
      uncompressedBundle: MemoryFileSystem.test().directory('bundle'),
      applicationPackage: MemoryFileSystem.test().directory('bundle'),
    );

    device.portForwarder = devicePortForwarder;
    device.setLogReader(iosApp, deviceLogReader);
    await device.dispose();

    expect(deviceLogReader.disposed, true);
    expect(devicePortForwarder.disposed, true);
  });

  testWithoutContext(
    'IOSDevice.startApp attaches in debug mode via log reading on iOS 13+',
    () async {
      final FileSystem fileSystem = MemoryFileSystem.test();
      final FakeProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
        attachDebuggerCommand(),
      ]);
      final IOSDevice device = setUpIOSDevice(
        processManager: processManager,
        fileSystem: fileSystem,
      );
      final IOSApp iosApp = PrebuiltIOSApp(
        projectBundleId: 'app',
        bundleName: 'Runner',
        uncompressedBundle: fileSystem.currentDirectory,
        applicationPackage: fileSystem.currentDirectory,
      );
      final FakeDeviceLogReader deviceLogReader = FakeDeviceLogReader();

      device.portForwarder = const NoOpDevicePortForwarder();
      device.setLogReader(iosApp, deviceLogReader);

      // Start writing messages to the log reader.
      Timer.run(() {
        deviceLogReader.addLine('Foo');
        deviceLogReader.addLine('The Dart VM service is listening on http://127.0.0.1:456');
      });

      final LaunchResult launchResult = await device.startApp(
        iosApp,
        prebuiltApplication: true,
        debuggingOptions: DebuggingOptions.enabled(BuildInfo.debug),
        platformArgs: <String, dynamic>{},
      );

      expect(launchResult.started, true);
      expect(launchResult.hasVmService, true);
      expect(await device.stopApp(iosApp), false);
    },
  );

  testWithoutContext(
    'IOSDevice.startApp twice in a row where ios-deploy fails the first time',
    () async {
      final BufferLogger logger = BufferLogger.test();
      final FileSystem fileSystem = MemoryFileSystem.test();
      final Completer<void> completer = Completer<void>();
      final FakeProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
        attachDebuggerCommand(stdout: 'PROCESS_EXITED'),
        attachDebuggerCommand(
          stdout:
              '(lldb)     run\nsuccess\nThe Dart VM service is listening on http://127.0.0.1:456',
          completer: completer,
        ),
      ]);
      final IOSDevice device = setUpIOSDevice(
        processManager: processManager,
        fileSystem: fileSystem,
        logger: logger,
      );
      final IOSApp iosApp = PrebuiltIOSApp(
        projectBundleId: 'app',
        bundleName: 'Runner',
        uncompressedBundle: fileSystem.currentDirectory,
        applicationPackage: fileSystem.currentDirectory,
      );

      device.portForwarder = const NoOpDevicePortForwarder();

      final LaunchResult launchResult = await device.startApp(
        iosApp,
        prebuiltApplication: true,
        debuggingOptions: DebuggingOptions.enabled(BuildInfo.debug),
        platformArgs: <String, dynamic>{},
      );

      expect(launchResult.started, false);
      expect(launchResult.hasVmService, false);

      final LaunchResult secondLaunchResult = await device.startApp(
        iosApp,
        prebuiltApplication: true,
        debuggingOptions: DebuggingOptions.enabled(BuildInfo.debug),
        platformArgs: <String, dynamic>{},
        discoveryTimeout: Duration.zero,
      );
      completer.complete();
      expect(secondLaunchResult.started, true);
      expect(secondLaunchResult.hasVmService, true);
      expect(await device.stopApp(iosApp), true);
    },
  );

  testWithoutContext(
    'IOSDevice.startApp launches in debug mode via log reading on <iOS 13',
    () async {
      final FileSystem fileSystem = MemoryFileSystem.test();
      final FakeProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
        kLaunchDebugCommand,
      ]);
      final IOSDevice device = setUpIOSDevice(
        sdkVersion: '12.4.4',
        processManager: processManager,
        fileSystem: fileSystem,
      );
      final IOSApp iosApp = PrebuiltIOSApp(
        projectBundleId: 'app',
        bundleName: 'Runner',
        uncompressedBundle: fileSystem.currentDirectory,
        applicationPackage: fileSystem.currentDirectory,
      );
      final FakeDeviceLogReader deviceLogReader = FakeDeviceLogReader();

      device.portForwarder = const NoOpDevicePortForwarder();
      device.setLogReader(iosApp, deviceLogReader);

      // Start writing messages to the log reader.
      Timer.run(() {
        deviceLogReader.addLine('Foo');
        deviceLogReader.addLine('The Dart VM service is listening on http://127.0.0.1:456');
      });

      final LaunchResult launchResult = await device.startApp(
        iosApp,
        prebuiltApplication: true,
        debuggingOptions: DebuggingOptions.enabled(BuildInfo.debug),
        platformArgs: <String, dynamic>{},
      );

      expect(launchResult.started, true);
      expect(launchResult.hasVmService, true);
      expect(await device.stopApp(iosApp), false);
    },
  );

  testWithoutContext(
    'IOSDevice.startApp prints warning message if discovery takes longer than configured timeout for wired device',
    () async {
      final FileSystem fileSystem = MemoryFileSystem.test();
      final BufferLogger logger = BufferLogger.test();
      final CompleterIOSink stdin = CompleterIOSink();
      final Completer<void> completer = Completer<void>();
      final FakeProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
        attachDebuggerCommand(stdin: stdin, completer: completer),
      ]);
      final IOSDevice device = setUpIOSDevice(
        processManager: processManager,
        fileSystem: fileSystem,
        logger: logger,
      );
      final IOSApp iosApp = PrebuiltIOSApp(
        projectBundleId: 'app',
        bundleName: 'Runner',
        uncompressedBundle: fileSystem.currentDirectory,
        applicationPackage: fileSystem.currentDirectory,
      );
      final FakeDeviceLogReader deviceLogReader = FakeDeviceLogReader();

      device.portForwarder = const NoOpDevicePortForwarder();
      device.setLogReader(iosApp, deviceLogReader);

      // Start writing messages to the log reader.
      deviceLogReader.addLine('Foo');
      deviceLogReader.addLine('The Dart VM service is listening on http://127.0.0.1:456');

      final LaunchResult launchResult = await device.startApp(
        iosApp,
        prebuiltApplication: true,
        debuggingOptions: DebuggingOptions.enabled(BuildInfo.debug),
        platformArgs: <String, dynamic>{},
        discoveryTimeout: Duration.zero,
      );

      expect(launchResult.started, true);
      expect(launchResult.hasVmService, true);
      expect(await device.stopApp(iosApp), true);
      expect(
        logger.errorText,
        contains(
          'The Dart VM Service was not discovered after 30 seconds. This is taking much longer than expected...',
        ),
      );
      expect(utf8.decoder.convert(stdin.writes.first), contains('process interrupt'));
      completer.complete();
      expect(processManager, hasNoRemainingExpectations);
    },
  );

  testUsingContext(
    'IOSDevice.startApp prints warning message if discovery takes longer than configured timeout for wireless device',
    () async {
      final FileSystem fileSystem = MemoryFileSystem.test();
      final BufferLogger logger = BufferLogger.test();
      final CompleterIOSink stdin = CompleterIOSink();
      final Completer<void> completer = Completer<void>();
      final FakeProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
        attachDebuggerCommand(stdin: stdin, completer: completer, isWirelessDevice: true),
      ]);
      final IOSDevice device = setUpIOSDevice(
        processManager: processManager,
        fileSystem: fileSystem,
        logger: logger,
        interfaceType: DeviceConnectionInterface.wireless,
      );
      final IOSApp iosApp = PrebuiltIOSApp(
        projectBundleId: 'app',
        bundleName: 'Runner',
        uncompressedBundle: fileSystem.currentDirectory,
        applicationPackage: fileSystem.currentDirectory,
      );
      final FakeDeviceLogReader deviceLogReader = FakeDeviceLogReader();

      device.portForwarder = const NoOpDevicePortForwarder();
      device.setLogReader(iosApp, deviceLogReader);

      // Start writing messages to the log reader.
      deviceLogReader.addLine('Foo');
      deviceLogReader.addLine('The Dart VM service is listening on http://127.0.0.1:456');

      final LaunchResult launchResult = await device.startApp(
        iosApp,
        prebuiltApplication: true,
        debuggingOptions: DebuggingOptions.enabled(BuildInfo.debug),
        platformArgs: <String, dynamic>{},
        discoveryTimeout: Duration.zero,
      );

      expect(launchResult.started, true);
      expect(launchResult.hasVmService, true);
      expect(await device.stopApp(iosApp), true);
      expect(
        logger.errorText,
        contains(
          'The Dart VM Service was not discovered after 45 seconds. This is taking much longer than expected...',
        ),
      );
      expect(
        logger.errorText,
        contains(
          'Your debugging device seems wirelessly connected. Consider plugging it in and trying again.',
        ),
      );
      expect(
        logger.errorText,
        contains(
          'Click "Allow" to the prompt asking if you would like to find and connect devices on your local network.',
        ),
      );
      completer.complete();
      expect(processManager, hasNoRemainingExpectations);
    },
    overrides: <Type, Generator>{MDnsVmServiceDiscovery: () => FakeMDnsVmServiceDiscovery()},
  );

  testWithoutContext(
    'IOSDevice.startApp retries when ios-deploy loses connection the first time in CI',
    () async {
      final BufferLogger logger = BufferLogger.test();
      final FileSystem fileSystem = MemoryFileSystem.test();
      final Completer<void> completer = Completer<void>();
      final FakeProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
        attachDebuggerCommand(
          stdout:
              '(lldb)     run\nsuccess\nProcess 525 exited with status = -1 (0xffffffff) lost connection',
          uninstallFirst: true,
        ),
        attachDebuggerCommand(
          stdout:
              '(lldb)     run\nsuccess\nThe Dart VM service is listening on http://127.0.0.1:456',
          completer: completer,
          skipInstall: true,
        ),
      ]);
      final IOSDevice device = setUpIOSDevice(
        processManager: processManager,
        fileSystem: fileSystem,
        logger: logger,
      );
      final IOSApp iosApp = PrebuiltIOSApp(
        projectBundleId: 'app',
        bundleName: 'Runner',
        uncompressedBundle: fileSystem.currentDirectory,
        applicationPackage: fileSystem.currentDirectory,
      );

      device.portForwarder = const NoOpDevicePortForwarder();

      final LaunchResult launchResult = await device.startApp(
        iosApp,
        prebuiltApplication: true,
        debuggingOptions: DebuggingOptions.enabled(
          BuildInfo.debug,
          usingCISystem: true,
          uninstallFirst: true,
        ),
        platformArgs: <String, dynamic>{},
      );
      completer.complete();

      expect(processManager, hasNoRemainingExpectations);
      expect(launchResult.started, true);
      expect(launchResult.hasVmService, true);
      expect(await device.stopApp(iosApp), true);
    },
  );

  testWithoutContext(
    'IOSDevice.startApp does not retry when ios-deploy loses connection if not in CI',
    () async {
      final BufferLogger logger = BufferLogger.test();
      final FileSystem fileSystem = MemoryFileSystem.test();
      final FakeProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
        attachDebuggerCommand(
          stdout:
              '(lldb)     run\nsuccess\nProcess 525 exited with status = -1 (0xffffffff) lost connection',
        ),
      ]);
      final IOSDevice device = setUpIOSDevice(
        processManager: processManager,
        fileSystem: fileSystem,
        logger: logger,
      );
      final IOSApp iosApp = PrebuiltIOSApp(
        projectBundleId: 'app',
        bundleName: 'Runner',
        uncompressedBundle: fileSystem.currentDirectory,
        applicationPackage: fileSystem.currentDirectory,
      );

      device.portForwarder = const NoOpDevicePortForwarder();

      final LaunchResult launchResult = await device.startApp(
        iosApp,
        prebuiltApplication: true,
        debuggingOptions: DebuggingOptions.enabled(BuildInfo.debug),
        platformArgs: <String, dynamic>{},
      );

      expect(processManager, hasNoRemainingExpectations);
      expect(launchResult.started, false);
      expect(launchResult.hasVmService, false);
      expect(await device.stopApp(iosApp), false);
    },
  );

  testWithoutContext('IOSDevice.startApp succeeds in release mode', () async {
    final FileSystem fileSystem = MemoryFileSystem.test();
    final FakeProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
      kLaunchReleaseCommand,
    ]);
    final IOSDevice device = setUpIOSDevice(processManager: processManager, fileSystem: fileSystem);
    final IOSApp iosApp = PrebuiltIOSApp(
      projectBundleId: 'app',
      bundleName: 'Runner',
      uncompressedBundle: fileSystem.currentDirectory,
      applicationPackage: fileSystem.currentDirectory,
    );

    final LaunchResult launchResult = await device.startApp(
      iosApp,
      prebuiltApplication: true,
      debuggingOptions: DebuggingOptions.disabled(BuildInfo.release),
      platformArgs: <String, dynamic>{},
    );

    expect(launchResult.started, true);
    expect(launchResult.hasVmService, false);
    expect(await device.stopApp(iosApp), false);
    expect(processManager, hasNoRemainingExpectations);
  });

  testWithoutContext('IOSDevice.startApp forwards all supported debugging options', () async {
    final FileSystem fileSystem = MemoryFileSystem.test();
    final FakeProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
      FakeCommand(
        command: <String>[
          'script',
          '-t',
          '0',
          '/dev/null',
          'HostArtifact.iosDeploy',
          '--id',
          '123',
          '--bundle',
          '/',
          '--debug',
          '--no-wifi',
          // The arguments below are determined by what is passed into
          // the debugging options argument to startApp.
          '--args',
          <String>[
            '--enable-dart-profiling',
            '--disable-service-auth-codes',
            '--disable-vm-service-publication',
            '--start-paused',
            '--dart-flags="--foo,--null_assertions"',
            '--use-test-fonts',
            '--enable-checked-mode',
            '--verify-entry-points',
            '--enable-software-rendering',
            '--trace-systrace',
            '--trace-to-file="path/to/trace.binpb"',
            '--skia-deterministic-rendering',
            '--trace-skia',
            '--trace-allowlist="foo"',
            '--trace-skia-allowlist="skia.a,skia.b"',
            '--endless-trace-buffer',
            '--dump-skp-on-shader-compilation',
            '--verbose-logging',
            '--cache-sksl',
            '--purge-persistent-cache',
            '--enable-impeller=false',
            '--enable-embedder-api',
          ].join(' '),
        ],
        environment: const <String, String>{
          'PATH': '/usr/bin:null',
          'DYLD_LIBRARY_PATH': '/path/to/libraries',
        },
        stdout: '(lldb)     run\nsuccess',
      ),
    ]);
    final IOSDevice device = setUpIOSDevice(
      sdkVersion: '13.3',
      processManager: processManager,
      fileSystem: fileSystem,
    );
    final IOSApp iosApp = PrebuiltIOSApp(
      projectBundleId: 'app',
      bundleName: 'Runner',
      uncompressedBundle: fileSystem.currentDirectory,
      applicationPackage: fileSystem.currentDirectory,
    );
    final FakeDeviceLogReader deviceLogReader = FakeDeviceLogReader();

    device.portForwarder = const NoOpDevicePortForwarder();
    device.setLogReader(iosApp, deviceLogReader);

    // Start writing messages to the log reader.
    Timer.run(() {
      deviceLogReader.addLine('The Dart VM service is listening on http://127.0.0.1:1234');
    });

    final LaunchResult launchResult = await device.startApp(
      iosApp,
      prebuiltApplication: true,
      debuggingOptions: DebuggingOptions.enabled(
        BuildInfo.debug,
        startPaused: true,
        disableServiceAuthCodes: true,
        disablePortPublication: true,
        dartFlags: '--foo',
        useTestFonts: true,
        enableSoftwareRendering: true,
        skiaDeterministicRendering: true,
        traceSkia: true,
        traceAllowlist: 'foo',
        traceSkiaAllowlist: 'skia.a,skia.b',
        traceSystrace: true,
        traceToFile: 'path/to/trace.binpb',
        endlessTraceBuffer: true,
        dumpSkpOnShaderCompilation: true,
        cacheSkSL: true,
        purgePersistentCache: true,
        verboseSystemLogs: true,
        enableImpeller: ImpellerStatus.disabled,
        nullAssertions: true,
        enableEmbedderApi: true,
      ),
      platformArgs: <String, dynamic>{},
    );

    expect(launchResult.started, true);
    expect(await device.stopApp(iosApp), false);
    expect(processManager, hasNoRemainingExpectations);
  });

  testWithoutContext('startApp using route', () async {
    final FileSystem fileSystem = MemoryFileSystem.test();
    final FakeProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
      FakeCommand(
        command: <String>[
          'script',
          '-t',
          '0',
          '/dev/null',
          'HostArtifact.iosDeploy',
          '--id',
          '123',
          '--bundle',
          '/',
          '--debug',
          '--no-wifi',
          '--args',
          <String>[
            '--enable-dart-profiling',
            '--enable-checked-mode',
            '--verify-entry-points',
            // The --route argument below is determined by what is passed into
            // route argument to startApp.
            '--route=/animation',
          ].join(' '),
        ],
        environment: const <String, String>{
          'PATH': '/usr/bin:null',
          'DYLD_LIBRARY_PATH': '/path/to/libraries',
        },
        stdout: '(lldb)     run\nsuccess',
      ),
    ]);
    final IOSDevice device = setUpIOSDevice(
      sdkVersion: '13.3',
      processManager: processManager,
      fileSystem: fileSystem,
    );
    final IOSApp iosApp = PrebuiltIOSApp(
      projectBundleId: 'app',
      bundleName: 'Runner',
      uncompressedBundle: fileSystem.currentDirectory,
      applicationPackage: fileSystem.currentDirectory,
    );
    final FakeDeviceLogReader deviceLogReader = FakeDeviceLogReader();

    device.portForwarder = const NoOpDevicePortForwarder();
    device.setLogReader(iosApp, deviceLogReader);

    // Start writing messages to the log reader.
    Timer.run(() {
      deviceLogReader.addLine('The Dart VM service is listening on http://127.0.0.1:1234');
    });

    final LaunchResult launchResult = await device.startApp(
      iosApp,
      prebuiltApplication: true,
      debuggingOptions: DebuggingOptions.enabled(BuildInfo.debug),
      platformArgs: <String, dynamic>{},
      route: '/animation',
    );

    expect(launchResult.started, true);
    expect(await device.stopApp(iosApp), false);
    expect(processManager, hasNoRemainingExpectations);
  });

  testWithoutContext('startApp using trace-startup', () async {
    final FileSystem fileSystem = MemoryFileSystem.test();
    final FakeProcessManager processManager = FakeProcessManager.list(<FakeCommand>[
      FakeCommand(
        command: <String>[
          'script',
          '-t',
          '0',
          '/dev/null',
          'HostArtifact.iosDeploy',
          '--id',
          '123',
          '--bundle',
          '/',
          '--debug',
          '--no-wifi',
          '--args',
          <String>[
            '--enable-dart-profiling',
            '--enable-checked-mode',
            '--verify-entry-points',
            // The --trace-startup argument below is determined by what is passed into
            // platformArgs argument to startApp.
            '--trace-startup',
          ].join(' '),
        ],
        environment: const <String, String>{
          'PATH': '/usr/bin:null',
          'DYLD_LIBRARY_PATH': '/path/to/libraries',
        },
        stdout: '(lldb)     run\nsuccess',
      ),
    ]);
    final IOSDevice device = setUpIOSDevice(
      sdkVersion: '13.3',
      processManager: processManager,
      fileSystem: fileSystem,
    );
    final IOSApp iosApp = PrebuiltIOSApp(
      projectBundleId: 'app',
      bundleName: 'Runner',
      uncompressedBundle: fileSystem.currentDirectory,
      applicationPackage: fileSystem.currentDirectory,
    );
    final FakeDeviceLogReader deviceLogReader = FakeDeviceLogReader();

    device.portForwarder = const NoOpDevicePortForwarder();
    device.setLogReader(iosApp, deviceLogReader);

    // Start writing messages to the log reader.
    Timer.run(() {
      deviceLogReader.addLine('The Dart VM service is listening on http://127.0.0.1:1234');
    });

    final LaunchResult launchResult = await device.startApp(
      iosApp,
      prebuiltApplication: true,
      debuggingOptions: DebuggingOptions.enabled(BuildInfo.debug),
      platformArgs: <String, dynamic>{'trace-startup': true},
    );

    expect(launchResult.started, true);
    expect(await device.stopApp(iosApp), false);
    expect(processManager, hasNoRemainingExpectations);
  });

  group('IOSDevice.startApp for CoreDevice', () {
    group('in debug mode', () {
      testUsingContext('succeeds', () async {
        final FileSystem fileSystem = MemoryFileSystem.test();
        final FakeProcessManager processManager = FakeProcessManager.empty();

        final Directory temporaryXcodeProjectDirectory = fileSystem.systemTempDirectory
            .childDirectory('flutter_empty_xcode.rand0');
        final Directory bundleLocation = fileSystem.currentDirectory;
        final IOSDevice device = setUpIOSDevice(
          processManager: processManager,
          fileSystem: fileSystem,
          isCoreDevice: true,
          coreDeviceControl: FakeIOSCoreDeviceControl(),
          xcodeDebug: FakeXcodeDebug(
            expectedProject: XcodeDebugProject(
              scheme: 'Runner',
              xcodeWorkspace: temporaryXcodeProjectDirectory.childDirectory('Runner.xcworkspace'),
              xcodeProject: temporaryXcodeProjectDirectory.childDirectory('Runner.xcodeproj'),
              hostAppProjectName: 'Runner',
            ),
            expectedDeviceId: '123',
            expectedLaunchArguments: <String>['--enable-dart-profiling'],
            expectedBundlePath: bundleLocation.path,
          ),
        );
        final IOSApp iosApp = PrebuiltIOSApp(
          projectBundleId: 'app',
          bundleName: 'Runner',
          uncompressedBundle: bundleLocation,
          applicationPackage: bundleLocation,
        );
        final FakeDeviceLogReader deviceLogReader = FakeDeviceLogReader();

        device.portForwarder = const NoOpDevicePortForwarder();
        device.setLogReader(iosApp, deviceLogReader);

        // Start writing messages to the log reader.
        Timer.run(() {
          deviceLogReader.addLine('Foo');
          deviceLogReader.addLine('The Dart VM service is listening on http://127.0.0.1:456');
        });

        final LaunchResult launchResult = await device.startApp(
          iosApp,
          prebuiltApplication: true,
          debuggingOptions: DebuggingOptions.enabled(BuildInfo.debug),
          platformArgs: <String, dynamic>{},
        );

        expect(launchResult.started, true);
      });

      testUsingContext('prints warning message if it takes too long to start debugging', () async {
        final FileSystem fileSystem = MemoryFileSystem.test();
        final FakeProcessManager processManager = FakeProcessManager.empty();
        final BufferLogger logger = BufferLogger.test();
        final Directory temporaryXcodeProjectDirectory = fileSystem.systemTempDirectory
            .childDirectory('flutter_empty_xcode.rand0');
        final Directory bundleLocation = fileSystem.currentDirectory;
        final Completer<void> completer = Completer<void>();
        final FakeXcodeDebug xcodeDebug = FakeXcodeDebug(
          expectedProject: XcodeDebugProject(
            scheme: 'Runner',
            xcodeWorkspace: temporaryXcodeProjectDirectory.childDirectory('Runner.xcworkspace'),
            xcodeProject: temporaryXcodeProjectDirectory.childDirectory('Runner.xcodeproj'),
            hostAppProjectName: 'Runner',
          ),
          expectedDeviceId: '123',
          expectedLaunchArguments: <String>['--enable-dart-profiling'],
          expectedBundlePath: bundleLocation.path,
          completer: completer,
        );
        final IOSDevice device = setUpIOSDevice(
          processManager: processManager,
          fileSystem: fileSystem,
          logger: logger,
          isCoreDevice: true,
          coreDeviceControl: FakeIOSCoreDeviceControl(),
          xcodeDebug: xcodeDebug,
        );
        final IOSApp iosApp = PrebuiltIOSApp(
          projectBundleId: 'app',
          bundleName: 'Runner',
          uncompressedBundle: bundleLocation,
          applicationPackage: bundleLocation,
        );
        final FakeDeviceLogReader deviceLogReader = FakeDeviceLogReader();

        device.portForwarder = const NoOpDevicePortForwarder();
        device.setLogReader(iosApp, deviceLogReader);

        // Start writing messages to the log reader.
        Timer.run(() {
          deviceLogReader.addLine('Foo');
          deviceLogReader.addLine('The Dart VM service is listening on http://127.0.0.1:456');
        });

        FakeAsync().run((FakeAsync fakeAsync) {
          device.startApp(
            iosApp,
            prebuiltApplication: true,
            debuggingOptions: DebuggingOptions.enabled(BuildInfo.debug),
            platformArgs: <String, dynamic>{},
          );

          fakeAsync.flushTimers();
          expect(
            logger.errorText,
            contains(
              'Xcode is taking longer than expected to start debugging the app. Ensure the project is opened in Xcode.',
            ),
          );
          completer.complete();
        });
      });

      testUsingContext('succeeds with shutdown hook added when running from CI', () async {
        final FileSystem fileSystem = MemoryFileSystem.test();
        final FakeProcessManager processManager = FakeProcessManager.empty();

        final Directory temporaryXcodeProjectDirectory = fileSystem.systemTempDirectory
            .childDirectory('flutter_empty_xcode.rand0');
        final Directory bundleLocation = fileSystem.currentDirectory;
        final IOSDevice device = setUpIOSDevice(
          processManager: processManager,
          fileSystem: fileSystem,
          isCoreDevice: true,
          coreDeviceControl: FakeIOSCoreDeviceControl(),
          xcodeDebug: FakeXcodeDebug(
            expectedProject: XcodeDebugProject(
              scheme: 'Runner',
              xcodeWorkspace: temporaryXcodeProjectDirectory.childDirectory('Runner.xcworkspace'),
              xcodeProject: temporaryXcodeProjectDirectory.childDirectory('Runner.xcodeproj'),
              hostAppProjectName: 'Runner',
            ),
            expectedDeviceId: '123',
            expectedLaunchArguments: <String>['--enable-dart-profiling'],
            expectedBundlePath: bundleLocation.path,
          ),
        );
        final IOSApp iosApp = PrebuiltIOSApp(
          projectBundleId: 'app',
          bundleName: 'Runner',
          uncompressedBundle: bundleLocation,
          applicationPackage: bundleLocation,
        );
        final FakeDeviceLogReader deviceLogReader = FakeDeviceLogReader();

        device.portForwarder = const NoOpDevicePortForwarder();
        device.setLogReader(iosApp, deviceLogReader);

        // Start writing messages to the log reader.
        Timer.run(() {
          deviceLogReader.addLine('Foo');
          deviceLogReader.addLine('The Dart VM service is listening on http://127.0.0.1:456');
        });

        final FakeShutDownHooks shutDownHooks = FakeShutDownHooks();

        final LaunchResult launchResult = await device.startApp(
          iosApp,
          prebuiltApplication: true,
          debuggingOptions: DebuggingOptions.enabled(BuildInfo.debug, usingCISystem: true),
          platformArgs: <String, dynamic>{},
          shutdownHooks: shutDownHooks,
        );

        expect(launchResult.started, true);
        expect(shutDownHooks.hooks.length, 1);
      });

      testUsingContext(
        'IOSDevice.startApp attaches in debug mode via mDNS when device logging fails',
        () async {
          final FileSystem fileSystem = MemoryFileSystem.test();
          final FakeProcessManager processManager = FakeProcessManager.empty();

          final Directory temporaryXcodeProjectDirectory = fileSystem.systemTempDirectory
              .childDirectory('flutter_empty_xcode.rand0');
          final Directory bundleLocation = fileSystem.currentDirectory;
          final IOSDevice device = setUpIOSDevice(
            processManager: processManager,
            fileSystem: fileSystem,
            isCoreDevice: true,
            coreDeviceControl: FakeIOSCoreDeviceControl(),
            xcodeDebug: FakeXcodeDebug(
              expectedProject: XcodeDebugProject(
                scheme: 'Runner',
                xcodeWorkspace: temporaryXcodeProjectDirectory.childDirectory('Runner.xcworkspace'),
                xcodeProject: temporaryXcodeProjectDirectory.childDirectory('Runner.xcodeproj'),
                hostAppProjectName: 'Runner',
              ),
              expectedDeviceId: '123',
              expectedLaunchArguments: <String>['--enable-dart-profiling'],
              expectedBundlePath: bundleLocation.path,
            ),
          );
          final IOSApp iosApp = PrebuiltIOSApp(
            projectBundleId: 'app',
            bundleName: 'Runner',
            uncompressedBundle: bundleLocation,
            applicationPackage: bundleLocation,
          );
          final FakeDeviceLogReader deviceLogReader = FakeDeviceLogReader();

          device.portForwarder = const NoOpDevicePortForwarder();
          device.setLogReader(iosApp, deviceLogReader);

          final LaunchResult launchResult = await device.startApp(
            iosApp,
            prebuiltApplication: true,
            debuggingOptions: DebuggingOptions.enabled(BuildInfo.debug),
            platformArgs: <String, dynamic>{},
          );

          expect(launchResult.started, true);
          expect(launchResult.hasVmService, true);
          expect(await device.stopApp(iosApp), true);
        },
        overrides: <Type, Generator>{MDnsVmServiceDiscovery: () => FakeMDnsVmServiceDiscovery()},
      );

      group('IOSDevice.startApp attaches in debug mode via device logging', () {
        late FakeMDnsVmServiceDiscovery mdnsDiscovery;
        setUp(() {
          mdnsDiscovery = FakeMDnsVmServiceDiscovery(returnsNull: true);
        });

        testUsingContext('when mDNS fails', () async {
          final FileSystem fileSystem = MemoryFileSystem.test();
          final FakeProcessManager processManager = FakeProcessManager.empty();

          final Directory temporaryXcodeProjectDirectory = fileSystem.systemTempDirectory
              .childDirectory('flutter_empty_xcode.rand0');
          final Directory bundleLocation = fileSystem.currentDirectory;
          final IOSDevice device = setUpIOSDevice(
            processManager: processManager,
            fileSystem: fileSystem,
            isCoreDevice: true,
            coreDeviceControl: FakeIOSCoreDeviceControl(),
            xcodeDebug: FakeXcodeDebug(
              expectedProject: XcodeDebugProject(
                scheme: 'Runner',
                xcodeWorkspace: temporaryXcodeProjectDirectory.childDirectory('Runner.xcworkspace'),
                xcodeProject: temporaryXcodeProjectDirectory.childDirectory('Runner.xcodeproj'),
                hostAppProjectName: 'Runner',
              ),
              expectedDeviceId: '123',
              expectedLaunchArguments: <String>['--enable-dart-profiling'],
              expectedBundlePath: bundleLocation.path,
            ),
          );
          final IOSApp iosApp = PrebuiltIOSApp(
            projectBundleId: 'app',
            bundleName: 'Runner',
            uncompressedBundle: bundleLocation,
            applicationPackage: bundleLocation,
          );
          final FakeDeviceLogReader deviceLogReader = FakeDeviceLogReader();

          device.portForwarder = const NoOpDevicePortForwarder();
          device.setLogReader(iosApp, deviceLogReader);

          unawaited(
            mdnsDiscovery.completer.future.whenComplete(() {
              // Start writing messages to the log reader.
              Timer.run(() {
                deviceLogReader.addLine('Foo');
                deviceLogReader.addLine('The Dart VM service is listening on http://127.0.0.1:456');
              });
            }),
          );

          final LaunchResult launchResult = await device.startApp(
            iosApp,
            prebuiltApplication: true,
            debuggingOptions: DebuggingOptions.enabled(BuildInfo.debug),
            platformArgs: <String, dynamic>{},
          );

          expect(launchResult.started, true);
          expect(launchResult.hasVmService, true);
          expect(await device.stopApp(iosApp), true);
        }, overrides: <Type, Generator>{MDnsVmServiceDiscovery: () => mdnsDiscovery});
      });

      testUsingContext(
        'IOSDevice.startApp fails to find Dart VM in CI',
        () async {
          final FileSystem fileSystem = MemoryFileSystem.test();
          final FakeProcessManager processManager = FakeProcessManager.empty();

          const String pathToFlutterLogs = '/path/to/flutter/logs';
          const String pathToHome = '/path/to/home';

          final Directory temporaryXcodeProjectDirectory = fileSystem.systemTempDirectory
              .childDirectory('flutter_empty_xcode.rand0');
          final Directory bundleLocation = fileSystem.currentDirectory;
          final IOSDevice device = setUpIOSDevice(
            processManager: processManager,
            fileSystem: fileSystem,
            isCoreDevice: true,
            coreDeviceControl: FakeIOSCoreDeviceControl(),
            xcodeDebug: FakeXcodeDebug(
              expectedProject: XcodeDebugProject(
                scheme: 'Runner',
                xcodeWorkspace: temporaryXcodeProjectDirectory.childDirectory('Runner.xcworkspace'),
                xcodeProject: temporaryXcodeProjectDirectory.childDirectory('Runner.xcodeproj'),
                hostAppProjectName: 'Runner',
              ),
              expectedDeviceId: '123',
              expectedLaunchArguments: <String>['--enable-dart-profiling'],
              expectedBundlePath: bundleLocation.path,
            ),
            platform: FakePlatform(
              operatingSystem: 'macos',
              environment: <String, String>{'HOME': pathToHome},
            ),
          );

          final IOSApp iosApp = PrebuiltIOSApp(
            projectBundleId: 'app',
            bundleName: 'Runner',
            uncompressedBundle: bundleLocation,
            applicationPackage: bundleLocation,
          );
          final FakeDeviceLogReader deviceLogReader = FakeDeviceLogReader();

          device.portForwarder = const NoOpDevicePortForwarder();
          device.setLogReader(iosApp, deviceLogReader);

          const String projectLogsPath = 'Runner-project1/Logs/Launch/Runner.xcresults';
          fileSystem
              .directory('$pathToHome/Library/Developer/Xcode/DerivedData/$projectLogsPath')
              .createSync(recursive: true);

          final Completer<void> completer = Completer<void>();
          await FakeAsync().run((FakeAsync time) {
            final Future<LaunchResult> futureLaunchResult = device.startApp(
              iosApp,
              prebuiltApplication: true,
              debuggingOptions: DebuggingOptions.enabled(
                BuildInfo.debug,
                usingCISystem: true,
                debugLogsDirectoryPath: pathToFlutterLogs,
              ),
              platformArgs: <String, dynamic>{},
            );
            futureLaunchResult.then((LaunchResult launchResult) {
              expect(launchResult.started, false);
              expect(launchResult.hasVmService, false);
              expect(
                fileSystem
                    .directory('$pathToFlutterLogs/DerivedDataLogs/$projectLogsPath')
                    .existsSync(),
                true,
              );
              completer.complete();
            });
            time.elapse(const Duration(minutes: 15));
            time.flushMicrotasks();
            return completer.future;
          });
        },
        overrides: <Type, Generator>{
          MDnsVmServiceDiscovery: () => FakeMDnsVmServiceDiscovery(returnsNull: true),
        },
      );
    });
  });
}

IOSDevice setUpIOSDevice({
  String sdkVersion = '13.0.1',
  FileSystem? fileSystem,
  Logger? logger,
  ProcessManager? processManager,
  IOSDeploy? iosDeploy,
  DeviceConnectionInterface interfaceType = DeviceConnectionInterface.attached,
  bool isCoreDevice = false,
  IOSCoreDeviceControl? coreDeviceControl,
  FakeXcodeDebug? xcodeDebug,
  FakePlatform? platform,
}) {
  final Artifacts artifacts = Artifacts.test();
  final FakePlatform macPlatform =
      platform ?? FakePlatform(operatingSystem: 'macos', environment: <String, String>{});

  final Cache cache = Cache.test(
    platform: macPlatform,
    artifacts: <ArtifactSet>[FakeDyldEnvironmentArtifact()],
    processManager: FakeProcessManager.any(),
  );
  logger ??= BufferLogger.test();
  return IOSDevice(
    '123',
    name: 'iPhone 1',
    sdkVersion: sdkVersion,
    fileSystem: fileSystem ?? MemoryFileSystem.test(),
    platform: macPlatform,
    iProxy: IProxy.test(logger: logger, processManager: processManager ?? FakeProcessManager.any()),
    logger: logger,
    iosDeploy:
        iosDeploy ??
        IOSDeploy(
          logger: logger,
          platform: macPlatform,
          processManager: processManager ?? FakeProcessManager.any(),
          artifacts: artifacts,
          cache: cache,
        ),
    iMobileDevice: IMobileDevice(
      logger: logger,
      processManager: processManager ?? FakeProcessManager.any(),
      artifacts: artifacts,
      cache: cache,
    ),
    coreDeviceControl: coreDeviceControl ?? FakeIOSCoreDeviceControl(),
    xcodeDebug: xcodeDebug ?? FakeXcodeDebug(),
    cpuArchitecture: DarwinArch.arm64,
    connectionInterface: interfaceType,
    isConnected: true,
    isPaired: true,
    devModeEnabled: true,
    isCoreDevice: isCoreDevice,
  );
}

class FakeDevicePortForwarder extends Fake implements DevicePortForwarder {
  bool disposed = false;

  @override
  Future<void> dispose() async {
    disposed = true;
  }
}

class FakeMDnsVmServiceDiscovery extends Fake implements MDnsVmServiceDiscovery {
  FakeMDnsVmServiceDiscovery({this.returnsNull = false});
  bool returnsNull;

  Completer<void> completer = Completer<void>();
  @override
  Future<Uri?> getVMServiceUriForLaunch(
    String applicationId,
    Device device, {
    bool usesIpv6 = false,
    int? hostVmservicePort,
    int? deviceVmservicePort,
    bool useDeviceIPAsHost = false,
    Duration timeout = Duration.zero,
  }) async {
    completer.complete();
    if (returnsNull) {
      return null;
    }

    return Uri.tryParse('http://0.0.0.0:1234');
  }
}

class FakeXcodeDebug extends Fake implements XcodeDebug {
  FakeXcodeDebug({
    this.debugSuccess = true,
    this.expectedProject,
    this.expectedDeviceId,
    this.expectedLaunchArguments,
    this.expectedBundlePath,
    this.completer,
  });

  final bool debugSuccess;
  final XcodeDebugProject? expectedProject;
  final String? expectedDeviceId;
  final List<String>? expectedLaunchArguments;
  final String? expectedBundlePath;
  final Completer<void>? completer;

  @override
  bool debugStarted = false;

  @override
  Future<XcodeDebugProject> createXcodeProjectWithCustomBundle(
    String deviceBundlePath, {
    required TemplateRenderer templateRenderer,
    Directory? projectDestination,
    bool verboseLogging = false,
  }) async {
    if (expectedBundlePath != null) {
      expect(expectedBundlePath, deviceBundlePath);
    }
    return expectedProject!;
  }

  @override
  Future<bool> debugApp({
    required XcodeDebugProject project,
    required String deviceId,
    required List<String> launchArguments,
  }) async {
    if (expectedProject != null) {
      expect(project.scheme, expectedProject!.scheme);
      expect(project.xcodeWorkspace.path, expectedProject!.xcodeWorkspace.path);
      expect(project.xcodeProject.path, expectedProject!.xcodeProject.path);
      expect(project.isTemporaryProject, expectedProject!.isTemporaryProject);
    }
    if (expectedDeviceId != null) {
      expect(deviceId, expectedDeviceId);
    }
    if (expectedLaunchArguments != null) {
      expect(expectedLaunchArguments, launchArguments);
    }
    debugStarted = debugSuccess;

    if (completer != null) {
      await completer!.future;
    }
    return debugSuccess;
  }

  @override
  Future<bool> exit({bool force = false, bool skipDelay = false}) async {
    return true;
  }
}

class FakeIOSCoreDeviceControl extends Fake implements IOSCoreDeviceControl {}

class FakeShutDownHooks extends Fake implements ShutdownHooks {
  List<ShutdownHook> hooks = <ShutdownHook>[];
  @override
  void addShutdownHook(ShutdownHook shutdownHook) {
    hooks.add(shutdownHook);
  }
}
