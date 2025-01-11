// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:convert';
import 'dart:io';

import 'package:dwds/dwds.dart';
import 'package:file/memory.dart';
import 'package:flutter_tools/src/application_package.dart';
import 'package:flutter_tools/src/asset.dart';
import 'package:flutter_tools/src/base/dds.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/platform.dart';
import 'package:flutter_tools/src/base/time.dart';
import 'package:flutter_tools/src/build_info.dart';
import 'package:flutter_tools/src/build_system/tools/scene_importer.dart';
import 'package:flutter_tools/src/build_system/tools/shader_compiler.dart';
import 'package:flutter_tools/src/compile.dart';
import 'package:flutter_tools/src/dart/pub.dart';
import 'package:flutter_tools/src/devfs.dart';
import 'package:flutter_tools/src/device.dart';
import 'package:flutter_tools/src/features.dart';
import 'package:flutter_tools/src/globals.dart' as globals;
import 'package:flutter_tools/src/isolated/devfs_web.dart';
import 'package:flutter_tools/src/isolated/resident_web_runner.dart';
import 'package:flutter_tools/src/project.dart';
import 'package:flutter_tools/src/reporting/reporting.dart';
import 'package:flutter_tools/src/resident_devtools_handler.dart';
import 'package:flutter_tools/src/resident_runner.dart';
import 'package:flutter_tools/src/vmservice.dart';
import 'package:flutter_tools/src/web/chrome.dart';
import 'package:flutter_tools/src/web/web_device.dart';
import 'package:package_config/package_config.dart';
import 'package:package_config/package_config_types.dart';
import 'package:test/fake.dart';
import 'package:unified_analytics/unified_analytics.dart';
import 'package:vm_service/vm_service.dart' as vm_service;
import 'package:webkit_inspection_protocol/webkit_inspection_protocol.dart';

import '../src/common.dart';
import '../src/context.dart';
import '../src/fake_process_manager.dart';
import '../src/fake_pub_deps.dart';
import '../src/fake_vm_services.dart';
import '../src/fakes.dart' as test_fakes;

const List<VmServiceExpectation> kAttachLogExpectations = <VmServiceExpectation>[
  FakeVmServiceRequest(method: 'streamListen', args: <String, Object>{'streamId': 'Stdout'}),
  FakeVmServiceRequest(method: 'streamListen', args: <String, Object>{'streamId': 'Stderr'}),
];

const List<VmServiceExpectation> kAttachIsolateExpectations = <VmServiceExpectation>[
  FakeVmServiceRequest(method: 'streamListen', args: <String, Object>{'streamId': 'Service'}),
  FakeVmServiceRequest(method: 'streamListen', args: <String, Object>{'streamId': 'Isolate'}),
  FakeVmServiceRequest(
    method: 'registerService',
    args: <String, Object>{'service': kReloadSourcesServiceName, 'alias': kFlutterToolAlias},
  ),
  FakeVmServiceRequest(
    method: 'registerService',
    args: <String, Object>{'service': kFlutterVersionServiceName, 'alias': kFlutterToolAlias},
  ),
  FakeVmServiceRequest(
    method: 'registerService',
    args: <String, Object>{'service': kFlutterMemoryInfoServiceName, 'alias': kFlutterToolAlias},
  ),
  FakeVmServiceRequest(method: 'streamListen', args: <String, Object>{'streamId': 'Extension'}),
];

const List<VmServiceExpectation> kAttachExpectations = <VmServiceExpectation>[
  ...kAttachLogExpectations,
  ...kAttachIsolateExpectations,
];

void main() {
  late FakeDebugConnection debugConnection;
  late FakeChromeDevice chromeDevice;
  late FakeAppConnection appConnection;
  late FakeFlutterDevice flutterDevice;
  late FakeWebDevFS webDevFS;
  late FakeResidentCompiler residentCompiler;
  late FakeChromeConnection chromeConnection;
  late FakeChromeTab chromeTab;
  late FakeWebServerDevice webServerDevice;
  late FakeDevice mockDevice;
  late FakeVmServiceHost fakeVmServiceHost;
  late MemoryFileSystem fileSystem;
  late ProcessManager processManager;
  late TestUsage testUsage;
  late FakeAnalytics fakeAnalytics;

  // TODO(matanlurey): Remove after `explicit-package-dependencies` is enabled by default.
  // See https://github.com/flutter/flutter/issues/160257 for details.
  FeatureFlags enableExplicitPackageDependencies() {
    return test_fakes.TestFeatureFlags(isExplicitPackageDependenciesEnabled: true);
  }

  setUp(() {
    testUsage = TestUsage();
    fileSystem = MemoryFileSystem.test();
    processManager = FakeProcessManager.any();
    debugConnection = FakeDebugConnection();
    mockDevice = FakeDevice();
    appConnection = FakeAppConnection();
    webDevFS = FakeWebDevFS();
    residentCompiler = FakeResidentCompiler();
    chromeDevice = FakeChromeDevice();
    chromeConnection = FakeChromeConnection();
    chromeTab = FakeChromeTab('index.html');
    webServerDevice = FakeWebServerDevice();
    flutterDevice =
        FakeFlutterDevice()
          .._devFS = webDevFS
          ..device = mockDevice
          ..generator = residentCompiler;
    fileSystem.directory('.dart_tool').childFile('package_config.json').createSync(recursive: true);
    fakeAnalytics = getInitializedFakeAnalyticsInstance(
      fs: fileSystem,
      fakeFlutterVersion: test_fakes.FakeFlutterVersion(),
    );
  });

  void setupMocks() {
    fileSystem.file('pubspec.yaml').createSync();
    fileSystem.file('lib/main.dart').createSync(recursive: true);
    fileSystem.file('web/index.html').createSync(recursive: true);
    webDevFS.report = UpdateFSReport(success: true);
    debugConnection.fakeVmServiceHost = () => fakeVmServiceHost;
    webDevFS.result = ConnectionResult(appConnection, debugConnection, debugConnection.vmService);
    debugConnection.uri = 'ws://127.0.0.1/abcd/';
    chromeConnection.tabs.add(chromeTab);
  }

  testUsingContext(
    'runner with web server device does not support debugging without --start-paused',
    () {
      final ResidentRunner residentWebRunner = setUpResidentRunner(flutterDevice);
      flutterDevice.device = WebServerDevice(logger: BufferLogger.test());
      fakeVmServiceHost = FakeVmServiceHost(requests: <VmServiceExpectation>[]);
      final ResidentRunner profileResidentWebRunner = ResidentWebRunner(
        flutterDevice,
        flutterProject: FlutterProject.fromDirectoryTest(fileSystem.currentDirectory),
        debuggingOptions: DebuggingOptions.enabled(BuildInfo.debug),
        fileSystem: fileSystem,
        logger: BufferLogger.test(),
        usage: globals.flutterUsage,
        analytics: globals.analytics,
        systemClock: globals.systemClock,
      );

      expect(profileResidentWebRunner.debuggingEnabled, false);

      flutterDevice.device = chromeDevice;

      expect(residentWebRunner.debuggingEnabled, true);
      expect(fakeVmServiceHost.hasRemainingExpectations, false);
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => processManager,
    },
  );

  testUsingContext(
    'runner with web server device supports debugging with --start-paused',
    () {
      fakeVmServiceHost = FakeVmServiceHost(requests: <VmServiceExpectation>[]);
      setupMocks();
      flutterDevice.device = WebServerDevice(logger: BufferLogger.test());
      final ResidentRunner profileResidentWebRunner = ResidentWebRunner(
        flutterDevice,
        flutterProject: FlutterProject.fromDirectoryTest(fileSystem.currentDirectory),
        debuggingOptions: DebuggingOptions.enabled(BuildInfo.debug, startPaused: true),
        fileSystem: fileSystem,
        logger: BufferLogger.test(),
        usage: globals.flutterUsage,
        analytics: globals.analytics,
        systemClock: globals.systemClock,
      );

      expect(profileResidentWebRunner.uri, webDevFS.baseUri);
      expect(profileResidentWebRunner.debuggingEnabled, true);
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => processManager,
    },
  );
  testUsingContext(
    'profile does not supportsServiceProtocol',
    () {
      final ResidentRunner residentWebRunner = ResidentWebRunner(
        flutterDevice,
        flutterProject: FlutterProject.fromDirectoryTest(fileSystem.currentDirectory),
        debuggingOptions: DebuggingOptions.enabled(BuildInfo.debug),
        fileSystem: fileSystem,
        logger: BufferLogger.test(),
        usage: globals.flutterUsage,
        analytics: globals.analytics,
        systemClock: globals.systemClock,
      );
      fakeVmServiceHost = FakeVmServiceHost(requests: <VmServiceExpectation>[]);
      flutterDevice.device = chromeDevice;
      final ResidentRunner profileResidentWebRunner = ResidentWebRunner(
        flutterDevice,
        flutterProject: FlutterProject.fromDirectoryTest(fileSystem.currentDirectory),
        debuggingOptions: DebuggingOptions.enabled(BuildInfo.profile),
        fileSystem: fileSystem,
        logger: BufferLogger.test(),
        usage: globals.flutterUsage,
        analytics: globals.analytics,
        systemClock: globals.systemClock,
      );

      expect(profileResidentWebRunner.supportsServiceProtocol, false);
      expect(residentWebRunner.supportsServiceProtocol, true);
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => processManager,
    },
  );

  testUsingContext(
    'Can successfully run and connect to vmservice',
    () async {
      final BufferLogger logger = BufferLogger.test();
      final ResidentRunner residentWebRunner = setUpResidentRunner(flutterDevice, logger: logger);
      fakeVmServiceHost = FakeVmServiceHost(requests: kAttachExpectations.toList());
      setupMocks();

      final Completer<DebugConnectionInfo> connectionInfoCompleter =
          Completer<DebugConnectionInfo>();
      unawaited(residentWebRunner.run(connectionInfoCompleter: connectionInfoCompleter));
      final DebugConnectionInfo debugConnectionInfo = await connectionInfoCompleter.future;

      expect(appConnection.ranMain, true);
      expect(logger.statusText, contains('Debug service listening on ws://127.0.0.1/abcd/'));
      expect(debugConnectionInfo.wsUri.toString(), 'ws://127.0.0.1/abcd/');
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => processManager,
      FeatureFlags: enableExplicitPackageDependencies,
      Pub: FakePubWithPrimedDeps.new,
    },
  );

  testUsingContext(
    'WebRunner copies compiled app.dill to cache during startup',
    () async {
      final DebuggingOptions debuggingOptions = DebuggingOptions.enabled(
        const BuildInfo(
          BuildMode.debug,
          null,
          treeShakeIcons: false,
          packageConfigPath: '.dart_tool/package_config.json',
        ),
      );
      final ResidentRunner residentWebRunner = setUpResidentRunner(
        flutterDevice,
        debuggingOptions: debuggingOptions,
      );
      fakeVmServiceHost = FakeVmServiceHost(requests: kAttachExpectations.toList());
      setupMocks();

      residentWebRunner.artifactDirectory.childFile('app.dill').writeAsStringSync('ABC');
      final Completer<DebugConnectionInfo> connectionInfoCompleter =
          Completer<DebugConnectionInfo>();
      unawaited(residentWebRunner.run(connectionInfoCompleter: connectionInfoCompleter));
      await connectionInfoCompleter.future;

      expect(
        await fileSystem.file(fileSystem.path.join('build', 'cache.dill')).readAsString(),
        'ABC',
      );
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => processManager,
      FeatureFlags: enableExplicitPackageDependencies,
      Pub: FakePubWithPrimedDeps.new,
    },
  );

  testUsingContext(
    'WebRunner copies compiled app.dill to cache during startup with track-widget-creation',
    () async {
      final ResidentRunner residentWebRunner = setUpResidentRunner(flutterDevice);
      fakeVmServiceHost = FakeVmServiceHost(requests: kAttachExpectations.toList());
      setupMocks();

      residentWebRunner.artifactDirectory.childFile('app.dill').writeAsStringSync('ABC');
      final Completer<DebugConnectionInfo> connectionInfoCompleter =
          Completer<DebugConnectionInfo>();
      unawaited(residentWebRunner.run(connectionInfoCompleter: connectionInfoCompleter));
      await connectionInfoCompleter.future;

      expect(
        await fileSystem
            .file(fileSystem.path.join('build', 'cache.dill.track.dill'))
            .readAsString(),
        'ABC',
      );
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => processManager,
      FeatureFlags: enableExplicitPackageDependencies,
      Pub: FakePubWithPrimedDeps.new,
    },
  );

  // Regression test for https://github.com/flutter/flutter/issues/60613
  testUsingContext(
    'ResidentWebRunner calls appFailedToStart if initial compilation fails',
    () async {
      fakeVmServiceHost = FakeVmServiceHost(requests: kAttachExpectations.toList());
      setupMocks();
      final ResidentRunner residentWebRunner = setUpResidentRunner(flutterDevice);
      fileSystem.file(globals.fs.path.join('lib', 'main.dart')).createSync(recursive: true);
      webDevFS.report = UpdateFSReport();

      expect(await residentWebRunner.run(), 1);
      // Completing this future ensures that the daemon can exit correctly.
      expect(await residentWebRunner.waitForAppToFinish(), 1);
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => processManager,
      FeatureFlags: enableExplicitPackageDependencies,
      Pub: FakePubWithPrimedDeps.new,
    },
  );

  testUsingContext(
    'Can successfully run without an index.html including status warning',
    () async {
      final BufferLogger logger = BufferLogger.test();
      fakeVmServiceHost = FakeVmServiceHost(requests: kAttachExpectations.toList());
      setupMocks();
      fileSystem.directory('web').deleteSync(recursive: true);
      final ResidentWebRunner residentWebRunner = ResidentWebRunner(
        flutterDevice,
        flutterProject: FlutterProject.fromDirectoryTest(fileSystem.currentDirectory),
        debuggingOptions: DebuggingOptions.enabled(BuildInfo.debug),
        stayResident: false,
        fileSystem: fileSystem,
        logger: logger,
        usage: globals.flutterUsage,
        analytics: globals.analytics,
        systemClock: globals.systemClock,
        devtoolsHandler: createNoOpHandler,
      );

      expect(await residentWebRunner.run(), 0);
      expect(logger.statusText, contains('This application is not configured to build on the web'));
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => processManager,
      FeatureFlags: enableExplicitPackageDependencies,
      Pub: FakePubWithPrimedDeps.new,
    },
  );

  testUsingContext(
    'Can successfully run and disconnect with --no-resident',
    () async {
      fakeVmServiceHost = FakeVmServiceHost(requests: kAttachExpectations.toList());
      setupMocks();
      final ResidentRunner residentWebRunner = ResidentWebRunner(
        flutterDevice,
        flutterProject: FlutterProject.fromDirectoryTest(fileSystem.currentDirectory),
        debuggingOptions: DebuggingOptions.enabled(BuildInfo.debug),
        stayResident: false,
        fileSystem: fileSystem,
        logger: BufferLogger.test(),
        usage: globals.flutterUsage,
        analytics: globals.analytics,
        systemClock: globals.systemClock,
        devtoolsHandler: createNoOpHandler,
      );

      expect(await residentWebRunner.run(), 0);
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => processManager,
      FeatureFlags: enableExplicitPackageDependencies,
      Pub: FakePubWithPrimedDeps.new,
    },
  );

  testUsingContext(
    'Listens to stdout and stderr streams before running main',
    () async {
      final BufferLogger logger = BufferLogger.test();
      final ResidentRunner residentWebRunner = setUpResidentRunner(flutterDevice, logger: logger);
      fakeVmServiceHost = FakeVmServiceHost(
        requests: <VmServiceExpectation>[
          ...kAttachLogExpectations,
          FakeVmServiceStreamResponse(
            streamId: 'Stdout',
            event: vm_service.Event(
              timestamp: 0,
              kind: vm_service.EventStreams.kStdout,
              bytes: base64.encode(utf8.encode('THIS MESSAGE IS IMPORTANT')),
            ),
          ),
          FakeVmServiceStreamResponse(
            streamId: 'Stderr',
            event: vm_service.Event(
              timestamp: 0,
              kind: vm_service.EventStreams.kStderr,
              bytes: base64.encode(utf8.encode('SO IS THIS')),
            ),
          ),
          ...kAttachIsolateExpectations,
        ],
      );
      setupMocks();
      final Completer<DebugConnectionInfo> connectionInfoCompleter =
          Completer<DebugConnectionInfo>();
      unawaited(residentWebRunner.run(connectionInfoCompleter: connectionInfoCompleter));
      await connectionInfoCompleter.future;

      expect(logger.statusText, contains('THIS MESSAGE IS IMPORTANT'));
      expect(logger.statusText, contains('SO IS THIS'));
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => processManager,
      FeatureFlags: enableExplicitPackageDependencies,
      Pub: FakePubWithPrimedDeps.new,
    },
  );

  testUsingContext(
    'Listens to extension events with structured errors',
    () async {
      final ResidentRunner residentWebRunner = setUpResidentRunner(
        flutterDevice,
        logger: testLogger,
      );
      final List<VmServiceExpectation> requests = <VmServiceExpectation>[
        ...kAttachExpectations,
        // This is the first error message of a session.
        FakeVmServiceStreamResponse(
          streamId: 'Extension',
          event: vm_service.Event(
            timestamp: 0,
            extensionKind: 'Flutter.Error',
            extensionData: vm_service.ExtensionData.parse(<String, Object?>{
              'errorsSinceReload': 0,
              'renderedErrorText': 'first',
            }),
            kind: vm_service.EventStreams.kExtension,
          ),
        ),
        // This is the second error message of a session.
        FakeVmServiceStreamResponse(
          streamId: 'Extension',
          event: vm_service.Event(
            timestamp: 0,
            extensionKind: 'Flutter.Error',
            extensionData: vm_service.ExtensionData.parse(<String, Object?>{
              'errorsSinceReload': 1,
              'renderedErrorText': 'second',
            }),
            kind: vm_service.EventStreams.kExtension,
          ),
        ),
        // This is not Flutter.Error kind data, so it should not be logged, even though it has a renderedErrorText field.
        FakeVmServiceStreamResponse(
          streamId: 'Extension',
          event: vm_service.Event(
            timestamp: 0,
            extensionKind: 'Other',
            extensionData: vm_service.ExtensionData.parse(<String, Object?>{
              'errorsSinceReload': 2,
              'renderedErrorText': 'not an error',
            }),
            kind: vm_service.EventStreams.kExtension,
          ),
        ),
        // This is the third error message of a session.
        FakeVmServiceStreamResponse(
          streamId: 'Extension',
          event: vm_service.Event(
            timestamp: 0,
            extensionKind: 'Flutter.Error',
            extensionData: vm_service.ExtensionData.parse(<String, Object?>{
              'errorsSinceReload': 2,
              'renderedErrorText': 'third',
            }),
            kind: vm_service.EventStreams.kExtension,
          ),
        ),
        // This is bogus error data.
        FakeVmServiceStreamResponse(
          streamId: 'Extension',
          event: vm_service.Event(
            timestamp: 0,
            extensionKind: 'Flutter.Error',
            extensionData: vm_service.ExtensionData.parse(<String, Object?>{'other': 'bad stuff'}),
            kind: vm_service.EventStreams.kExtension,
          ),
        ),
        // Empty error text should not break anything.
        FakeVmServiceStreamResponse(
          streamId: 'Extension',
          event: vm_service.Event(
            timestamp: 0,
            extensionKind: 'Flutter.Error',
            extensionData: vm_service.ExtensionData.parse(<String, Object?>{
              'test': 'data',
              'renderedErrorText': '',
            }),
            kind: vm_service.EventStreams.kExtension,
          ),
        ),
        // Messages without errorsSinceReload should act as if errorsSinceReload: 0
        FakeVmServiceStreamResponse(
          streamId: 'Extension',
          event: vm_service.Event(
            timestamp: 0,
            extensionKind: 'Flutter.Error',
            extensionData: vm_service.ExtensionData.parse(<String, Object?>{
              'test': 'data',
              'renderedErrorText': 'error text',
            }),
            kind: vm_service.EventStreams.kExtension,
          ),
        ),
        // When adding things here, make sure the last one is supposed to output something
        // to the statusLog, otherwise you won't be able to distinguish the absence of output
        // due to it passing the test from absence due to it not running the test.
      ];
      // We use requests below, so make a copy of it here (FakeVmServiceHost will
      // clear its copy internally, which would affect our pumping below).
      fakeVmServiceHost = FakeVmServiceHost(requests: requests.toList());

      setupMocks();
      final Completer<DebugConnectionInfo> connectionInfoCompleter =
          Completer<DebugConnectionInfo>();
      unawaited(residentWebRunner.run(connectionInfoCompleter: connectionInfoCompleter));
      await connectionInfoCompleter.future;
      assert(requests.length > 5, 'requests was modified');
      for (final Object _ in requests) {
        // pump the task queue once for each message
        await null;
      }

      expect(
        testLogger.statusText,
        'Launching lib/main.dart on FakeDevice in debug mode...\n'
        'Waiting for connection from debug service on FakeDevice...\n'
        'Debug service listening on ws://127.0.0.1/abcd/\n'
        '\n'
        'first\n'
        '\n'
        'second\n'
        'third\n'
        '\n'
        '\n' // the empty message
        '\n'
        '\n'
        'error text\n'
        '\n',
      );

      expect(
        testLogger.errorText,
        'Received an invalid Flutter.Error message from app: {other: bad stuff}\n',
      );
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => processManager,
      FeatureFlags: enableExplicitPackageDependencies,
      Pub: FakePubWithPrimedDeps.new,
    },
  );

  testUsingContext(
    'Does not run main with --start-paused',
    () async {
      final ResidentRunner residentWebRunner = ResidentWebRunner(
        flutterDevice,
        flutterProject: FlutterProject.fromDirectoryTest(fileSystem.currentDirectory),
        debuggingOptions: DebuggingOptions.enabled(BuildInfo.debug, startPaused: true),
        fileSystem: fileSystem,
        logger: BufferLogger.test(),
        usage: globals.flutterUsage,
        analytics: globals.analytics,
        systemClock: globals.systemClock,
        devtoolsHandler: createNoOpHandler,
      );
      fakeVmServiceHost = FakeVmServiceHost(requests: kAttachExpectations.toList());
      setupMocks();
      final Completer<DebugConnectionInfo> connectionInfoCompleter =
          Completer<DebugConnectionInfo>();

      unawaited(residentWebRunner.run(connectionInfoCompleter: connectionInfoCompleter));
      await connectionInfoCompleter.future;

      expect(appConnection.ranMain, false);
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => processManager,
      FeatureFlags: enableExplicitPackageDependencies,
      Pub: FakePubWithPrimedDeps.new,
    },
  );

  testUsingContext(
    'Can hot reload after attaching',
    () async {
      final BufferLogger logger = BufferLogger.test();
      final ResidentRunner residentWebRunner = setUpResidentRunner(
        flutterDevice,
        logger: logger,
        systemClock: SystemClock.fixed(DateTime(2001)),
      );
      fakeVmServiceHost = FakeVmServiceHost(
        requests: <VmServiceExpectation>[
          ...kAttachExpectations,
          const FakeVmServiceRequest(
            method: kHotRestartServiceName,
            jsonResponse: <String, Object>{'type': 'Success'},
          ),
          const FakeVmServiceRequest(
            method: 'streamListen',
            args: <String, Object>{'streamId': 'Isolate'},
          ),
        ],
      );
      setupMocks();
      final TestChromiumLauncher chromiumLauncher = TestChromiumLauncher();
      final FakeProcess process = FakeProcess();
      final Chromium chrome = Chromium(
        1,
        chromeConnection,
        chromiumLauncher: chromiumLauncher,
        process: process,
        logger: logger,
      );
      chromiumLauncher.setInstance(chrome);

      flutterDevice.device = GoogleChromeDevice(
        fileSystem: fileSystem,
        chromiumLauncher: chromiumLauncher,
        logger: BufferLogger.test(),
        platform: FakePlatform(),
        processManager: FakeProcessManager.any(),
      );
      webDevFS.report = UpdateFSReport(success: true);

      final Completer<DebugConnectionInfo> connectionInfoCompleter =
          Completer<DebugConnectionInfo>();
      unawaited(residentWebRunner.run(connectionInfoCompleter: connectionInfoCompleter));
      final DebugConnectionInfo debugConnectionInfo = await connectionInfoCompleter.future;

      expect(debugConnectionInfo, isNotNull);

      final OperationResult result = await residentWebRunner.restart();

      expect(logger.statusText, contains('Restarted application in'));
      expect(result.code, 0);
      expect(webDevFS.mainUri.toString(), contains('entrypoint.dart'));

      // ensure that analytics are sent.
      expect(testUsage.events, <TestUsageEvent>[
        TestUsageEvent(
          'hot',
          'restart',
          parameters: CustomDimensions.fromMap(<String, String>{
            'cd27': 'web-javascript',
            'cd28': '',
            'cd29': 'false',
            'cd30': 'true',
            'cd13': '0',
            'cd48': 'false',
          }),
        ),
      ]);
      expect(
        fakeAnalytics.sentEvents,
        contains(
          Event.hotRunnerInfo(
            label: 'restart',
            targetPlatform: 'web-javascript',
            sdkName: '',
            emulator: false,
            fullRestart: true,
            overallTimeInMs: 0,
          ),
        ),
      );
      expect(testUsage.timings, const <TestTimingEvent>[
        TestTimingEvent('hot', 'web-incremental-restart', Duration.zero),
      ]);
      expect(
        fakeAnalytics.sentEvents,
        contains(
          Event.timing(
            workflow: 'hot',
            variableName: 'web-incremental-restart',
            elapsedMilliseconds: 0,
          ),
        ),
      );
    },
    overrides: <Type, Generator>{
      Usage: () => testUsage,
      Analytics: () => fakeAnalytics,
      FileSystem: () => fileSystem,
      ProcessManager: () => processManager,
      FeatureFlags: enableExplicitPackageDependencies,
      Pub: FakePubWithPrimedDeps.new,
    },
  );

  testUsingContext(
    'Can hot restart after attaching',
    () async {
      final BufferLogger logger = BufferLogger.test();
      final ResidentRunner residentWebRunner = setUpResidentRunner(
        flutterDevice,
        logger: logger,
        systemClock: SystemClock.fixed(DateTime(2001)),
      );
      fakeVmServiceHost = FakeVmServiceHost(
        requests: <VmServiceExpectation>[
          ...kAttachExpectations,
          const FakeVmServiceRequest(
            method: kHotRestartServiceName,
            jsonResponse: <String, Object>{'type': 'Success'},
          ),
        ],
      );
      setupMocks();
      final TestChromiumLauncher chromiumLauncher = TestChromiumLauncher();
      final FakeProcess process = FakeProcess();
      final Chromium chrome = Chromium(
        1,
        chromeConnection,
        chromiumLauncher: chromiumLauncher,
        process: process,
        logger: logger,
      );
      chromiumLauncher.setInstance(chrome);

      flutterDevice.device = GoogleChromeDevice(
        fileSystem: fileSystem,
        chromiumLauncher: chromiumLauncher,
        logger: BufferLogger.test(),
        platform: FakePlatform(),
        processManager: FakeProcessManager.any(),
      );
      webDevFS.report = UpdateFSReport(success: true);

      final Completer<DebugConnectionInfo> connectionInfoCompleter =
          Completer<DebugConnectionInfo>();
      unawaited(residentWebRunner.run(connectionInfoCompleter: connectionInfoCompleter));
      await connectionInfoCompleter.future;
      final OperationResult result = await residentWebRunner.restart(fullRestart: true);

      // Ensure that generated entrypoint is generated correctly.
      expect(webDevFS.mainUri, isNotNull);
      final String entrypointContents = fileSystem.file(webDevFS.mainUri).readAsStringSync();
      expect(entrypointContents, contains('// Flutter web bootstrap script'));
      expect(entrypointContents, contains("import 'dart:ui_web' as ui_web;"));
      expect(entrypointContents, contains('await ui_web.bootstrapEngine('));

      expect(logger.statusText, contains('Restarted application in'));
      expect(result.code, 0);

      // ensure that analytics are sent.
      expect(testUsage.events, <TestUsageEvent>[
        TestUsageEvent(
          'hot',
          'restart',
          parameters: CustomDimensions.fromMap(<String, String>{
            'cd27': 'web-javascript',
            'cd28': '',
            'cd29': 'false',
            'cd30': 'true',
            'cd13': '0',
            'cd48': 'false',
          }),
        ),
      ]);
      expect(
        fakeAnalytics.sentEvents,
        contains(
          Event.hotRunnerInfo(
            label: 'restart',
            targetPlatform: 'web-javascript',
            sdkName: '',
            emulator: false,
            fullRestart: true,
            overallTimeInMs: 0,
          ),
        ),
      );
      expect(testUsage.timings, const <TestTimingEvent>[
        TestTimingEvent('hot', 'web-incremental-restart', Duration.zero),
      ]);
      expect(
        fakeAnalytics.sentEvents,
        contains(
          Event.timing(
            workflow: 'hot',
            variableName: 'web-incremental-restart',
            elapsedMilliseconds: 0,
          ),
        ),
      );
    },
    overrides: <Type, Generator>{
      Usage: () => testUsage,
      Analytics: () => fakeAnalytics,
      FileSystem: () => fileSystem,
      ProcessManager: () => processManager,
      FeatureFlags: enableExplicitPackageDependencies,
      Pub: FakePubWithPrimedDeps.new,
    },
  );

  testUsingContext(
    'Can hot restart after attaching with web-server device',
    () async {
      final BufferLogger logger = BufferLogger.test();
      final ResidentRunner residentWebRunner = setUpResidentRunner(
        flutterDevice,
        logger: logger,
        systemClock: SystemClock.fixed(DateTime(2001)),
      );
      fakeVmServiceHost = FakeVmServiceHost(requests: kAttachExpectations);
      setupMocks();
      flutterDevice.device = webServerDevice;
      webDevFS.report = UpdateFSReport(success: true);

      final Completer<DebugConnectionInfo> connectionInfoCompleter =
          Completer<DebugConnectionInfo>();
      unawaited(residentWebRunner.run(connectionInfoCompleter: connectionInfoCompleter));
      await connectionInfoCompleter.future;
      final OperationResult result = await residentWebRunner.restart(fullRestart: true);

      expect(logger.statusText, contains('Restarted application in'));
      expect(result.code, 0);

      // web-server device does not send restart analytics
      expect(testUsage.events, isEmpty);
      expect(fakeAnalytics.sentEvents, isEmpty);
      expect(testUsage.timings, isEmpty);
    },
    overrides: <Type, Generator>{
      Usage: () => testUsage,
      Analytics: () => fakeAnalytics,
      FileSystem: () => fileSystem,
      ProcessManager: () => processManager,
      FeatureFlags: enableExplicitPackageDependencies,
      Pub: FakePubWithPrimedDeps.new,
    },
  );

  testUsingContext(
    'web resident runner is debuggable',
    () {
      final ResidentRunner residentWebRunner = setUpResidentRunner(flutterDevice);
      fakeVmServiceHost = FakeVmServiceHost(requests: kAttachExpectations.toList());

      expect(residentWebRunner.debuggingEnabled, true);
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => processManager,
    },
  );

  testUsingContext(
    'Exits when initial compile fails',
    () async {
      final ResidentRunner residentWebRunner = setUpResidentRunner(flutterDevice);
      fakeVmServiceHost = FakeVmServiceHost(requests: <VmServiceExpectation>[]);
      setupMocks();
      webDevFS.report = UpdateFSReport();

      final Completer<DebugConnectionInfo> connectionInfoCompleter =
          Completer<DebugConnectionInfo>();
      unawaited(residentWebRunner.run(connectionInfoCompleter: connectionInfoCompleter));

      expect(await residentWebRunner.run(), 1);
      expect(testUsage.events, isEmpty);
      expect(fakeAnalytics.sentEvents, isEmpty);
      expect(testUsage.timings, isEmpty);
    },
    overrides: <Type, Generator>{
      Usage: () => testUsage,
      Analytics: () => fakeAnalytics,
      FileSystem: () => fileSystem,
      ProcessManager: () => processManager,
      FeatureFlags: enableExplicitPackageDependencies,
      Pub: FakePubWithPrimedDeps.new,
    },
  );

  testUsingContext(
    'Faithfully displays stdout messages with leading/trailing spaces',
    () async {
      final BufferLogger logger = BufferLogger.test();
      final ResidentRunner residentWebRunner = setUpResidentRunner(flutterDevice, logger: logger);
      fakeVmServiceHost = FakeVmServiceHost(
        requests: <VmServiceExpectation>[
          ...kAttachLogExpectations,
          FakeVmServiceStreamResponse(
            streamId: 'Stdout',
            event: vm_service.Event(
              timestamp: 0,
              kind: vm_service.EventStreams.kStdout,
              bytes: base64.encode(
                utf8.encode('    This is a message with 4 leading and trailing spaces    '),
              ),
            ),
          ),
          ...kAttachIsolateExpectations,
        ],
      );
      setupMocks();
      final Completer<DebugConnectionInfo> connectionInfoCompleter =
          Completer<DebugConnectionInfo>();
      unawaited(residentWebRunner.run(connectionInfoCompleter: connectionInfoCompleter));
      await connectionInfoCompleter.future;

      expect(
        logger.statusText,
        contains('    This is a message with 4 leading and trailing spaces    '),
      );
      expect(fakeVmServiceHost.hasRemainingExpectations, false);
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => processManager,
      FeatureFlags: enableExplicitPackageDependencies,
      Pub: FakePubWithPrimedDeps.new,
    },
  );

  testUsingContext(
    'Fails on compilation errors in hot restart',
    () async {
      final ResidentRunner residentWebRunner = setUpResidentRunner(flutterDevice);
      fakeVmServiceHost = FakeVmServiceHost(requests: kAttachExpectations.toList());
      setupMocks();
      final Completer<DebugConnectionInfo> connectionInfoCompleter =
          Completer<DebugConnectionInfo>();
      unawaited(residentWebRunner.run(connectionInfoCompleter: connectionInfoCompleter));
      await connectionInfoCompleter.future;
      webDevFS.report = UpdateFSReport();

      final OperationResult result = await residentWebRunner.restart(fullRestart: true);

      expect(result.code, 1);
      expect(result.message, contains('Failed to recompile application.'));
      expect(testUsage.events, isEmpty);
      expect(fakeAnalytics.sentEvents, isEmpty);
      expect(testUsage.timings, isEmpty);
    },
    overrides: <Type, Generator>{
      Usage: () => testUsage,
      Analytics: () => fakeAnalytics,
      FileSystem: () => fileSystem,
      ProcessManager: () => processManager,
      FeatureFlags: enableExplicitPackageDependencies,
      Pub: FakePubWithPrimedDeps.new,
    },
  );

  testUsingContext(
    'Fails non-fatally on vmservice response error for hot restart',
    () async {
      final ResidentRunner residentWebRunner = setUpResidentRunner(flutterDevice);
      fakeVmServiceHost = FakeVmServiceHost(
        requests: <VmServiceExpectation>[
          ...kAttachExpectations,
          const FakeVmServiceRequest(
            method: kHotRestartServiceName,
            jsonResponse: <String, Object>{'type': 'Failed'},
          ),
        ],
      );
      setupMocks();
      final Completer<DebugConnectionInfo> connectionInfoCompleter =
          Completer<DebugConnectionInfo>();
      unawaited(residentWebRunner.run(connectionInfoCompleter: connectionInfoCompleter));
      await connectionInfoCompleter.future;

      final OperationResult result = await residentWebRunner.restart();

      expect(result.code, 0);
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => processManager,
      FeatureFlags: enableExplicitPackageDependencies,
      Pub: FakePubWithPrimedDeps.new,
    },
  );

  testUsingContext(
    'Fails fatally on Vm Service error response',
    () async {
      final ResidentRunner residentWebRunner = setUpResidentRunner(flutterDevice);
      fakeVmServiceHost = FakeVmServiceHost(
        requests: <VmServiceExpectation>[
          ...kAttachExpectations,
          FakeVmServiceRequest(
            method: kHotRestartServiceName,
            // Failed response,
            error: FakeRPCError(code: vm_service.RPCErrorKind.kInternalError.code),
          ),
        ],
      );
      setupMocks();
      final Completer<DebugConnectionInfo> connectionInfoCompleter =
          Completer<DebugConnectionInfo>();
      unawaited(residentWebRunner.run(connectionInfoCompleter: connectionInfoCompleter));
      await connectionInfoCompleter.future;
      final OperationResult result = await residentWebRunner.restart();

      expect(result.code, 1);
      expect(result.message, contains(vm_service.RPCErrorKind.kInternalError.code.toString()));
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => processManager,
      FeatureFlags: enableExplicitPackageDependencies,
      Pub: FakePubWithPrimedDeps.new,
    },
  );

  testUsingContext(
    'printHelp without details shows hot restart help message',
    () async {
      final BufferLogger logger = BufferLogger.test();
      final ResidentRunner residentWebRunner = setUpResidentRunner(flutterDevice, logger: logger);
      fakeVmServiceHost = FakeVmServiceHost(requests: <VmServiceExpectation>[]);
      residentWebRunner.printHelp(details: false);

      expect(logger.statusText, contains('To hot restart changes'));
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => processManager,
    },
  );

  testUsingContext(
    'cleanup of resources is safe to call multiple times',
    () async {
      final ResidentRunner residentWebRunner = setUpResidentRunner(flutterDevice);
      mockDevice.dds = DartDevelopmentService(logger: test_fakes.FakeLogger());
      fakeVmServiceHost = FakeVmServiceHost(
        requests: <VmServiceExpectation>[...kAttachExpectations],
      );
      setupMocks();
      final Completer<DebugConnectionInfo> connectionInfoCompleter =
          Completer<DebugConnectionInfo>();
      unawaited(residentWebRunner.run(connectionInfoCompleter: connectionInfoCompleter));
      await connectionInfoCompleter.future;

      await residentWebRunner.exit();
      await residentWebRunner.exit();

      expect(debugConnection.didClose, false);
      expect(fakeVmServiceHost.hasRemainingExpectations, false);
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => processManager,
      FeatureFlags: enableExplicitPackageDependencies,
      Pub: FakePubWithPrimedDeps.new,
    },
  );

  testUsingContext(
    'cleans up Chrome if tab is closed',
    () async {
      final ResidentRunner residentWebRunner = setUpResidentRunner(flutterDevice);
      fakeVmServiceHost = FakeVmServiceHost(
        requests: <VmServiceExpectation>[...kAttachExpectations],
      );
      setupMocks();
      final Completer<DebugConnectionInfo> connectionInfoCompleter =
          Completer<DebugConnectionInfo>();
      final Future<int?> result = residentWebRunner.run(
        connectionInfoCompleter: connectionInfoCompleter,
      );
      await connectionInfoCompleter.future;
      debugConnection.completer.complete();

      await result;
      expect(fakeVmServiceHost.hasRemainingExpectations, false);
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => processManager,
      FeatureFlags: enableExplicitPackageDependencies,
      Pub: FakePubWithPrimedDeps.new,
    },
  );

  testUsingContext(
    'Prints target and device name on run',
    () async {
      final BufferLogger logger = BufferLogger.test();
      final ResidentRunner residentWebRunner = setUpResidentRunner(flutterDevice, logger: logger);
      fakeVmServiceHost = FakeVmServiceHost(
        requests: <VmServiceExpectation>[...kAttachExpectations],
      );
      setupMocks();
      mockDevice.name = 'Chromez';
      final Completer<DebugConnectionInfo> connectionInfoCompleter =
          Completer<DebugConnectionInfo>();
      unawaited(residentWebRunner.run(connectionInfoCompleter: connectionInfoCompleter));
      await connectionInfoCompleter.future;

      expect(
        logger.statusText,
        contains(
          'Launching ${fileSystem.path.join('lib', 'main.dart')} on '
          'Chromez in debug mode',
        ),
      );
      expect(fakeVmServiceHost.hasRemainingExpectations, false);
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => processManager,
      FeatureFlags: enableExplicitPackageDependencies,
      Pub: FakePubWithPrimedDeps.new,
    },
  );

  testUsingContext(
    'Sends launched app.webLaunchUrl event for Chrome device',
    () async {
      final BufferLogger logger = BufferLogger.test();
      fakeVmServiceHost = FakeVmServiceHost(
        requests: <VmServiceExpectation>[...kAttachLogExpectations, ...kAttachIsolateExpectations],
      );
      setupMocks();
      final FakeChromeConnection chromeConnection = FakeChromeConnection();
      final TestChromiumLauncher chromiumLauncher = TestChromiumLauncher();
      final FakeProcess process = FakeProcess();
      final Chromium chrome = Chromium(
        1,
        chromeConnection,
        chromiumLauncher: chromiumLauncher,
        process: process,
        logger: logger,
      );
      chromiumLauncher.setInstance(chrome);

      flutterDevice.device = GoogleChromeDevice(
        fileSystem: fileSystem,
        chromiumLauncher: chromiumLauncher,
        logger: logger,
        platform: FakePlatform(),
        processManager: FakeProcessManager.any(),
      );
      webDevFS.baseUri = Uri.parse('http://localhost:8765/app/');

      final FakeChromeTab chromeTab = FakeChromeTab('index.html');
      chromeConnection.tabs.add(chromeTab);

      final ResidentWebRunner runner = ResidentWebRunner(
        flutterDevice,
        flutterProject: FlutterProject.fromDirectoryTest(fileSystem.currentDirectory),
        debuggingOptions: DebuggingOptions.enabled(BuildInfo.debug),
        fileSystem: fileSystem,
        logger: logger,
        usage: globals.flutterUsage,
        analytics: globals.analytics,
        systemClock: globals.systemClock,
        devtoolsHandler: createNoOpHandler,
      );

      final Completer<DebugConnectionInfo> connectionInfoCompleter =
          Completer<DebugConnectionInfo>();
      unawaited(runner.run(connectionInfoCompleter: connectionInfoCompleter));
      await connectionInfoCompleter.future;

      // Ensure we got the URL and that it was already launched.
      expect(
        logger.eventText,
        contains(
          json.encode(<String, Object>{
            'name': 'app.webLaunchUrl',
            'args': <String, Object>{'url': 'http://localhost:8765/app/', 'launched': true},
          }),
        ),
      );
      expect(fakeVmServiceHost.hasRemainingExpectations, false);
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => processManager,
      FeatureFlags: enableExplicitPackageDependencies,
      Pub: FakePubWithPrimedDeps.new,
    },
  );

  testUsingContext(
    'Sends unlaunched app.webLaunchUrl event for Web Server device',
    () async {
      final BufferLogger logger = BufferLogger.test();
      fakeVmServiceHost = FakeVmServiceHost(requests: <VmServiceExpectation>[]);
      setupMocks();
      flutterDevice.device = WebServerDevice(logger: logger);
      webDevFS.baseUri = Uri.parse('http://localhost:8765/app/');

      final ResidentWebRunner runner = ResidentWebRunner(
        flutterDevice,
        flutterProject: FlutterProject.fromDirectoryTest(fileSystem.currentDirectory),
        debuggingOptions: DebuggingOptions.enabled(BuildInfo.debug),
        fileSystem: fileSystem,
        logger: logger,
        usage: globals.flutterUsage,
        analytics: globals.analytics,
        systemClock: globals.systemClock,
        devtoolsHandler: createNoOpHandler,
      );

      final Completer<DebugConnectionInfo> connectionInfoCompleter =
          Completer<DebugConnectionInfo>();
      unawaited(runner.run(connectionInfoCompleter: connectionInfoCompleter));
      await connectionInfoCompleter.future;

      // Ensure we got the URL and that it was not already launched.
      expect(
        logger.eventText,
        contains(
          json.encode(<String, Object>{
            'name': 'app.webLaunchUrl',
            'args': <String, Object>{'url': 'http://localhost:8765/app/', 'launched': false},
          }),
        ),
      );
      expect(fakeVmServiceHost.hasRemainingExpectations, false);
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => processManager,
      FeatureFlags: enableExplicitPackageDependencies,
      Pub: FakePubWithPrimedDeps.new,
    },
  );

  testUsingContext(
    'ResidentWebRunner generates files when l10n.yaml exists',
    () async {
      fakeVmServiceHost = FakeVmServiceHost(requests: kAttachExpectations.toList());
      setupMocks();
      final ResidentRunner residentWebRunner = ResidentWebRunner(
        flutterDevice,
        flutterProject: FlutterProject.fromDirectoryTest(fileSystem.currentDirectory),
        debuggingOptions: DebuggingOptions.enabled(BuildInfo.debug),
        stayResident: false,
        fileSystem: fileSystem,
        logger: BufferLogger.test(),
        usage: globals.flutterUsage,
        analytics: globals.analytics,
        systemClock: globals.systemClock,
        devtoolsHandler: createNoOpHandler,
      );

      // Create necessary files.
      globals.fs.file(globals.fs.path.join('lib', 'main.dart')).createSync(recursive: true);
      globals.fs.file(globals.fs.path.join('lib', 'l10n', 'app_en.arb'))
        ..createSync(recursive: true)
        ..writeAsStringSync('''
{
  "helloWorld": "Hello, World!",
  "@helloWorld": {
    "description": "Sample description"
  }
}''');
      globals.fs.file('l10n.yaml').createSync();
      globals.fs.file('pubspec.yaml').writeAsStringSync('''
flutter:
  generate: true
''');
      globals.fs.directory('.dart_tool').childFile('package_config.json')
        ..createSync(recursive: true)
        ..writeAsStringSync('''
{
  "configVersion": 2,
  "packages": [
    {
      "name": "path_provider_linux",
      "rootUri": "../../../path_provider_linux",
      "packageUri": "lib/",
      "languageVersion": "2.12"
    }
  ]
}
''');
      expect(await residentWebRunner.run(), 0);
      final File generatedLocalizationsFile = globals.fs
          .directory('.dart_tool')
          .childDirectory('flutter_gen')
          .childDirectory('gen_l10n')
          .childFile('app_localizations.dart');
      expect(generatedLocalizationsFile.existsSync(), isTrue);
      // Completing this future ensures that the daemon can exit correctly.
      expect(fakeVmServiceHost.hasRemainingExpectations, false);
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => processManager,
      FeatureFlags: enableExplicitPackageDependencies,
      Pub: FakePubWithPrimedDeps.new,
    },
  );

  // While this file should be ignored on web, generating it here will cause a
  // perf regression in hot restart.
  testUsingContext(
    'Does not generate dart_plugin_registrant.dart',
    () async {
      // Create necessary files for [DartPluginRegistrantTarget]
      final File packageConfig = globals.fs
          .directory('.dart_tool')
          .childFile('package_config.json');
      packageConfig.createSync(recursive: true);
      packageConfig.writeAsStringSync('''
{
  "configVersion": 2,
  "packages": [
    {
      "name": "path_provider_linux",
      "rootUri": "../../../path_provider_linux",
      "packageUri": "lib/",
      "languageVersion": "2.12"
    }
  ]
}
''');
      // Start with a dart_plugin_registrant.dart file.
      globals.fs
          .directory('.dart_tool')
          .childDirectory('flutter_build')
          .childFile('dart_plugin_registrant.dart')
          .createSync(recursive: true);

      final FlutterProject project = FlutterProject.fromDirectoryTest(fileSystem.currentDirectory);

      final ResidentRunner residentWebRunner = setUpResidentRunner(flutterDevice);
      await residentWebRunner.runSourceGenerators();

      // dart_plugin_registrant.dart should be untouched, indicating that its
      // generation didn't run. If it had run, the file would have been removed as
      // there are no plugins in the project.
      expect(project.dartPluginRegistrant.existsSync(), true);
      expect(project.dartPluginRegistrant.readAsStringSync(), '');
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => processManager,
    },
  );

  testUsingContext(
    'Successfully turns WebSocketException into ToolExit',
    () async {
      final BufferLogger logger = BufferLogger.test();
      final ResidentRunner residentWebRunner = setUpResidentRunner(flutterDevice, logger: logger);
      fakeVmServiceHost = FakeVmServiceHost(requests: <VmServiceExpectation>[]);
      setupMocks();
      webDevFS.exception = const WebSocketException();

      await expectLater(residentWebRunner.run, throwsToolExit());
      expect(logger.errorText, contains('WebSocketException'));
      expect(fakeVmServiceHost.hasRemainingExpectations, false);
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => processManager,
      FeatureFlags: enableExplicitPackageDependencies,
      Pub: FakePubWithPrimedDeps.new,
    },
  );

  testUsingContext(
    'Turns HttpException from ChromeTab::connect into ToolExit',
    () async {
      final BufferLogger logger = BufferLogger.test();
      final ResidentRunner residentWebRunner = setUpResidentRunner(flutterDevice, logger: logger);
      fakeVmServiceHost = FakeVmServiceHost(requests: <VmServiceExpectation>[]);
      setupMocks();
      final FakeChromeConnection chromeConnection = FakeChromeConnection();
      final TestChromiumLauncher chromiumLauncher = TestChromiumLauncher();
      final FakeProcess process = FakeProcess();
      final Chromium chrome = Chromium(
        1,
        chromeConnection,
        chromiumLauncher: chromiumLauncher,
        process: process,
        logger: logger,
      );
      chromiumLauncher.setInstance(chrome);

      flutterDevice.device = GoogleChromeDevice(
        fileSystem: fileSystem,
        chromiumLauncher: chromiumLauncher,
        logger: logger,
        platform: FakePlatform(),
        processManager: FakeProcessManager.any(),
      );
      webDevFS.baseUri = Uri.parse('http://localhost:8765/app/');

      final FakeChromeTab chromeTab = FakeChromeTab(
        'index.html',
        connectException: HttpException(
          'Connection closed before full header was received',
          uri: Uri(path: 'http://localhost:50094/devtools/page/3036A94908353E86E183B6A40F54104B'),
        ),
      );
      chromeConnection.tabs.add(chromeTab);

      await expectLater(
        residentWebRunner.run,
        throwsToolExit(
          message: 'Failed to establish connection with the application instance in Chrome.',
        ),
      );
      expect(logger.errorText, contains('HttpException'));
      expect(fakeVmServiceHost.hasRemainingExpectations, isFalse);
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => processManager,
      FeatureFlags: enableExplicitPackageDependencies,
      Pub: FakePubWithPrimedDeps.new,
    },
  );

  testUsingContext(
    'Successfully turns AppConnectionException into ToolExit',
    () async {
      final ResidentRunner residentWebRunner = setUpResidentRunner(flutterDevice);
      fakeVmServiceHost = FakeVmServiceHost(requests: <VmServiceExpectation>[]);
      setupMocks();
      webDevFS.exception = AppConnectionException('');

      await expectLater(residentWebRunner.run, throwsToolExit());
      expect(fakeVmServiceHost.hasRemainingExpectations, false);
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => processManager,
      FeatureFlags: enableExplicitPackageDependencies,
      Pub: FakePubWithPrimedDeps.new,
    },
  );

  testUsingContext(
    'Successfully turns ChromeDebugError into ToolExit',
    () async {
      final ResidentRunner residentWebRunner = setUpResidentRunner(flutterDevice);
      fakeVmServiceHost = FakeVmServiceHost(requests: <VmServiceExpectation>[]);
      setupMocks();

      webDevFS.exception = ChromeDebugException(<String, Object?>{'text': 'error'});

      await expectLater(residentWebRunner.run, throwsToolExit());
      expect(fakeVmServiceHost.hasRemainingExpectations, false);
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => processManager,
      FeatureFlags: enableExplicitPackageDependencies,
      Pub: FakePubWithPrimedDeps.new,
    },
  );

  testUsingContext(
    'Rethrows unknown Exception type from dwds',
    () async {
      final ResidentRunner residentWebRunner = setUpResidentRunner(flutterDevice);
      fakeVmServiceHost = FakeVmServiceHost(requests: <VmServiceExpectation>[]);
      setupMocks();
      webDevFS.exception = Exception();

      await expectLater(residentWebRunner.run, throwsException);
      expect(fakeVmServiceHost.hasRemainingExpectations, false);
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => processManager,
      FeatureFlags: enableExplicitPackageDependencies,
      Pub: FakePubWithPrimedDeps.new,
    },
  );

  testUsingContext(
    'Rethrows unknown Error type from dwds tooling',
    () async {
      final BufferLogger logger = BufferLogger.test();
      final ResidentRunner residentWebRunner = setUpResidentRunner(flutterDevice, logger: logger);
      fakeVmServiceHost = FakeVmServiceHost(requests: <VmServiceExpectation>[]);
      setupMocks();
      webDevFS.exception = StateError('');

      await expectLater(residentWebRunner.run, throwsStateError);
      expect(fakeVmServiceHost.hasRemainingExpectations, false);
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => processManager,
      FeatureFlags: enableExplicitPackageDependencies,
      Pub: FakePubWithPrimedDeps.new,
    },
  );

  testUsingContext(
    'throws when port is an integer outside the valid TCP range',
    () async {
      final BufferLogger logger = BufferLogger.test();

      DebuggingOptions debuggingOptions = DebuggingOptions.enabled(BuildInfo.debug, port: '65536');
      ResidentRunner residentWebRunner = setUpResidentRunner(
        flutterDevice,
        logger: logger,
        debuggingOptions: debuggingOptions,
      );
      await expectToolExitLater(residentWebRunner.run(), matches('Invalid port: 65536.*'));

      debuggingOptions = DebuggingOptions.enabled(BuildInfo.debug, port: '-1');
      residentWebRunner = setUpResidentRunner(
        flutterDevice,
        logger: logger,
        debuggingOptions: debuggingOptions,
      );
      await expectToolExitLater(residentWebRunner.run(), matches('Invalid port: -1.*'));
    },
    overrides: <Type, Generator>{
      FileSystem: () => fileSystem,
      ProcessManager: () => processManager,
    },
  );
}

ResidentRunner setUpResidentRunner(
  FlutterDevice flutterDevice, {
  Logger? logger,
  SystemClock? systemClock,
  DebuggingOptions? debuggingOptions,
}) {
  return ResidentWebRunner(
    flutterDevice,
    flutterProject: FlutterProject.fromDirectoryTest(globals.fs.currentDirectory),
    debuggingOptions: debuggingOptions ?? DebuggingOptions.enabled(BuildInfo.debug),
    usage: globals.flutterUsage,
    analytics: globals.analytics,
    systemClock: systemClock ?? SystemClock.fixed(DateTime.now()),
    fileSystem: globals.fs,
    logger: logger ?? BufferLogger.test(),
    devtoolsHandler: createNoOpHandler,
  );
}

class FakeWebServerDevice extends FakeDevice implements WebServerDevice {}

class FakeDevice extends Fake implements Device {
  @override
  String name = 'FakeDevice';

  @override
  String get displayName => name;

  int count = 0;

  @override
  Future<String> get sdkNameAndVersion async => 'SDK Name and Version';

  @override
  late DartDevelopmentService dds;

  @override
  Future<LaunchResult> startApp(
    ApplicationPackage? package, {
    String? mainPath,
    String? route,
    DebuggingOptions? debuggingOptions,
    Map<String, dynamic>? platformArgs,
    bool prebuiltApplication = false,
    bool ipv6 = false,
    String? userIdentifier,
  }) async {
    return LaunchResult.succeeded();
  }

  @override
  Future<bool> stopApp(ApplicationPackage? app, {String? userIdentifier}) async {
    if (count > 0) {
      throw StateError('stopApp called more than once.');
    }
    count += 1;
    return true;
  }
}

class FakeDebugConnection extends Fake implements DebugConnection {
  late FakeVmServiceHost Function() fakeVmServiceHost;

  @override
  vm_service.VmService get vmService => fakeVmServiceHost.call().vmService.service;

  @override
  late String uri;

  final Completer<void> completer = Completer<void>();
  bool didClose = false;

  @override
  Future<void> get onDone => completer.future;

  @override
  Future<void> close() async {
    didClose = true;
  }
}

class FakeAppConnection extends Fake implements AppConnection {
  bool ranMain = false;

  @override
  void runMain() {
    ranMain = true;
  }
}

class FakeChromeDevice extends Fake implements ChromiumDevice {}

class FakeWipDebugger extends Fake implements WipDebugger {}

class FakeResidentCompiler extends Fake implements ResidentCompiler {
  @override
  Future<CompilerOutput> recompile(
    Uri mainUri,
    List<Uri>? invalidatedFiles, {
    required String outputPath,
    required PackageConfig packageConfig,
    required FileSystem fs,
    String? projectRootPath,
    bool suppressErrors = false,
    bool checkDartPluginRegistry = false,
    File? dartPluginRegistrant,
    Uri? nativeAssetsYaml,
  }) async {
    return const CompilerOutput('foo.dill', 0, <Uri>[]);
  }

  @override
  void accept() {}

  @override
  void reset() {}

  @override
  Future<CompilerOutput> reject() async {
    return const CompilerOutput('foo.dill', 0, <Uri>[]);
  }

  @override
  void addFileSystemRoot(String root) {}
}

class FakeWebDevFS extends Fake implements WebDevFS {
  Object? exception;
  ConnectionResult? result;
  late UpdateFSReport report;

  Uri? mainUri;

  @override
  List<Uri> sources = <Uri>[];

  @override
  Uri baseUri = Uri.parse('http://localhost:12345');

  @override
  DateTime? lastCompiled = DateTime.now();

  @override
  PackageConfig? lastPackageConfig = PackageConfig.empty;

  @override
  Future<Uri> create() async {
    return baseUri;
  }

  @override
  Future<UpdateFSReport> update({
    required Uri mainUri,
    required ResidentCompiler generator,
    required bool trackWidgetCreation,
    required String pathToReload,
    required List<Uri> invalidatedFiles,
    required PackageConfig packageConfig,
    required String dillOutputPath,
    required DevelopmentShaderCompiler shaderCompiler,
    DevelopmentSceneImporter? sceneImporter,
    DevFSWriter? devFSWriter,
    String? target,
    AssetBundle? bundle,
    bool bundleFirstUpload = false,
    bool fullRestart = false,
    String? projectRootPath,
    File? dartPluginRegistrant,
  }) async {
    this.mainUri = mainUri;
    return report;
  }

  @override
  Future<ConnectionResult?> connect(
    bool useDebugExtension, {
    VmServiceFactory vmServiceFactory = createVmServiceDelegate,
  }) async {
    if (exception != null) {
      assert(exception is Exception || exception is Error);
      // ignore: only_throw_errors, exception is either Error or Exception here.
      throw exception!;
    }
    return result;
  }
}

class FakeChromeConnection extends Fake implements ChromeConnection {
  final List<ChromeTab> tabs = <ChromeTab>[];

  @override
  Future<ChromeTab> getTab(bool Function(ChromeTab tab) accept, {Duration? retryFor}) async {
    return tabs.firstWhere(accept);
  }

  @override
  Future<List<ChromeTab>> getTabs({Duration? retryFor}) async {
    return tabs;
  }
}

class FakeChromeTab extends Fake implements ChromeTab {
  FakeChromeTab(this.url, {Exception? connectException}) : _connectException = connectException;

  @override
  final String url;

  final Exception? _connectException;
  final FakeWipConnection connection = FakeWipConnection();

  @override
  Future<WipConnection> connect({Function? onError}) async {
    if (_connectException != null) {
      throw _connectException;
    }
    return connection;
  }
}

class FakeWipConnection extends Fake implements WipConnection {
  @override
  final WipDebugger debugger = FakeWipDebugger();
}

/// A test implementation of the [ChromiumLauncher] that launches a fixed instance.
class TestChromiumLauncher implements ChromiumLauncher {
  TestChromiumLauncher();

  bool _hasInstance = false;
  void setInstance(Chromium chromium) {
    _hasInstance = true;
    currentCompleter.complete(chromium);
  }

  @override
  Completer<Chromium> currentCompleter = Completer<Chromium>();

  @override
  bool canFindExecutable() {
    return true;
  }

  @override
  Future<Chromium> get connectedInstance => currentCompleter.future;

  @override
  String findExecutable() {
    return 'chrome';
  }

  @override
  bool get hasChromeInstance => _hasInstance;

  @override
  Future<Chromium> launch(
    String url, {
    bool headless = false,
    int? debugPort,
    bool skipCheck = false,
    Directory? cacheDir,
    List<String> webBrowserFlags = const <String>[],
  }) async {
    return currentCompleter.future;
  }

  @override
  Future<Chromium> connect(Chromium chrome, bool skipCheck) {
    return currentCompleter.future;
  }
}

class FakeFlutterDevice extends Fake implements FlutterDevice {
  Uri? testUri;
  UpdateFSReport report = UpdateFSReport(success: true, invalidatedSourcesCount: 1);
  Exception? reportError;

  @override
  ResidentCompiler? generator;

  @override
  Stream<Uri?> get vmServiceUris => Stream<Uri?>.value(testUri);

  @override
  DevelopmentShaderCompiler get developmentShaderCompiler => const FakeShaderCompiler();

  @override
  FlutterVmService? vmService;

  DevFS? _devFS;

  @override
  DevFS? get devFS => _devFS;

  @override
  set devFS(DevFS? value) {}

  @override
  Device? device;

  @override
  Future<void> stopEchoingDeviceLog() async {}

  @override
  Future<Uri?> setupDevFS(String fsName, Directory rootDirectory) async {
    return testUri;
  }

  @override
  Future<void> exitApps({Duration timeoutDelay = const Duration(seconds: 10)}) async {}

  @override
  Future<void> connect({
    ReloadSources? reloadSources,
    Restart? restart,
    CompileExpression? compileExpression,
    GetSkSLMethod? getSkSLMethod,
    FlutterProject? flutterProject,
    PrintStructuredErrorLogMethod? printStructuredErrorLogMethod,
    required DebuggingOptions debuggingOptions,
    int? hostVmServicePort,
    bool? ipv6 = false,
    bool enableDevTools = false,
    bool allowExistingDdsInstance = false,
  }) async {}

  @override
  Future<UpdateFSReport> updateDevFS({
    Uri? mainUri,
    String? target,
    AssetBundle? bundle,
    bool bundleFirstUpload = false,
    bool bundleDirty = false,
    bool fullRestart = false,
    String? projectRootPath,
    String? pathToReload,
    String? dillOutputPath,
    List<Uri>? invalidatedFiles,
    PackageConfig? packageConfig,
    File? dartPluginRegistrant,
  }) async {
    if (reportError != null) {
      throw reportError!;
    }
    return report;
  }

  @override
  Future<void> updateReloadStatus(bool wasReloadSuccessful) async {}
}

class FakeShaderCompiler implements DevelopmentShaderCompiler {
  const FakeShaderCompiler();

  @override
  void configureCompiler(TargetPlatform? platform) {}

  @override
  Future<DevFSContent> recompileShader(DevFSContent inputShader) {
    throw UnimplementedError();
  }
}
