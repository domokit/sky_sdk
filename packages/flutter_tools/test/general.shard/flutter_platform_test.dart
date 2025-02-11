// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:io' as io;

import 'package:file/memory.dart';
import 'package:flutter_tools/src/application_package.dart';
import 'package:flutter_tools/src/artifacts.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/io.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/process.dart';
import 'package:flutter_tools/src/build_info.dart';
import 'package:flutter_tools/src/device.dart';
import 'package:flutter_tools/src/flutter_manifest.dart';
import 'package:flutter_tools/src/globals.dart' as globals;
import 'package:flutter_tools/src/project.dart';
import 'package:flutter_tools/src/test/flutter_platform.dart';
import 'package:flutter_tools/src/test/test_compiler.dart';
import 'package:flutter_tools/src/vmservice.dart';
import 'package:stream_channel/stream_channel.dart';
import 'package:test/fake.dart';
import 'package:test_core/backend.dart';
import 'package:vm_service/src/vm_service.dart';

import '../src/common.dart';
import '../src/context.dart';

void main() {
  late FileSystem fileSystem;

  setUp(() {
    fileSystem = MemoryFileSystem.test();
    fileSystem.file('.dart_tool/package_config.json')
      ..createSync(recursive: true)
      ..writeAsStringSync('{"configVersion":2,"packages":[]}');
  });

  group('FlutterPlatform', () {
    late SuitePlatform fakeSuitePlatform;
    setUp(() {
      fakeSuitePlatform = SuitePlatform(Runtime.vm);
    });

    testUsingContext(
      'ensureConfiguration throws an error if an '
      'explicitVmServicePort is specified and more than one test file',
      () async {
        final FlutterPlatform flutterPlatform = FlutterPlatform(
          flutterTesterBinPath: '/',
          debuggingOptions: DebuggingOptions.enabled(BuildInfo.debug, hostVmServicePort: 1234),
          enableVmService: false,
          buildInfo: BuildInfo.debug,
          fileSystem: fileSystem,
          processManager: FakeProcessManager.empty(),
          logger: BufferLogger.test(),
        );
        flutterPlatform.loadChannel('test1.dart', fakeSuitePlatform);

        expect(
          () => flutterPlatform.loadChannel('test2.dart', fakeSuitePlatform),
          throwsToolExit(),
        );
      },
      overrides: <Type, Generator>{
        FileSystem: () => fileSystem,
        ProcessManager: () => FakeProcessManager.any(),
      },
    );

    testUsingContext(
      'ensureConfiguration throws an error if a precompiled '
      'entrypoint is specified and more that one test file',
      () {
        final FlutterPlatform flutterPlatform = FlutterPlatform(
          debuggingOptions: DebuggingOptions.enabled(BuildInfo.debug),
          flutterTesterBinPath: '/',
          precompiledDillPath: 'example.dill',
          enableVmService: false,
          buildInfo: BuildInfo.debug,
          fileSystem: fileSystem,
          processManager: FakeProcessManager.empty(),
          logger: BufferLogger.test(),
        );
        flutterPlatform.loadChannel('test1.dart', fakeSuitePlatform);

        expect(
          () => flutterPlatform.loadChannel('test2.dart', fakeSuitePlatform),
          throwsToolExit(),
        );
      },
      overrides: <Type, Generator>{
        FileSystem: () => fileSystem,
        ProcessManager: () => FakeProcessManager.any(),
      },
    );

    testUsingContext(
      'an exception from the app not starting bubbles up to the test runner',
      () async {
        final _UnstartableDevice testDevice = _UnstartableDevice();
        final FlutterPlatform flutterPlatform = FlutterPlatform(
          debuggingOptions: DebuggingOptions.enabled(BuildInfo.debug),
          flutterTesterBinPath: '/',
          enableVmService: false,
          integrationTestDevice: testDevice,
          flutterProject: _FakeFlutterProject(),
          host: InternetAddress.anyIPv4,
          updateGoldens: false,
          buildInfo: BuildInfo.debug,
          fileSystem: fileSystem,
          processManager: FakeProcessManager.empty(),
          logger: BufferLogger.test(),
        );

        await expectLater(
          () => flutterPlatform.loadChannel('test1.dart', fakeSuitePlatform).stream.drain<void>(),
          // we intercept the actual exception and throw a string for the test runner to catch
          throwsA(
            isA<String>().having(
              (String msg) => msg,
              'string',
              'Unable to start the app on the device.',
            ),
          ),
        );
        expect(
          (globals.logger as BufferLogger).traceText,
          contains('test 0: error caught during test;'),
        );
      },
      overrides: <Type, Generator>{
        FileSystem: () => fileSystem,
        ProcessManager: () => FakeProcessManager.any(),
        ApplicationPackageFactory: () => _FakeApplicationPackageFactory(),
      },
    );

    testUsingContext(
      'a shutdown signal terminates the test device',
      () async {
        final _WorkingDevice testDevice = _WorkingDevice();

        final ShutdownHooks shutdownHooks = ShutdownHooks();
        final FlutterPlatform flutterPlatform = FlutterPlatform(
          debuggingOptions: DebuggingOptions.disabled(BuildInfo.debug),
          flutterTesterBinPath: '/',
          enableVmService: false,
          integrationTestDevice: testDevice,
          flutterProject: _FakeFlutterProject(),
          host: InternetAddress.anyIPv4,
          updateGoldens: false,
          shutdownHooks: shutdownHooks,
          buildInfo: BuildInfo.debug,
          fileSystem: fileSystem,
          processManager: FakeProcessManager.empty(),
          logger: BufferLogger.test(),
        );

        await expectLater(
          () => flutterPlatform.loadChannel('test1.dart', fakeSuitePlatform).stream.drain<void>(),
          returnsNormally,
        );

        final BufferLogger logger = globals.logger as BufferLogger;
        await shutdownHooks.runShutdownHooks(logger);
        expect(logger.traceText, contains('test 0: ensuring test device is terminated.'));
      },
      overrides: <Type, Generator>{
        FileSystem: () => fileSystem,
        ProcessManager: () => FakeProcessManager.any(),
        ApplicationPackageFactory: () => _FakeApplicationPackageFactory(),
      },
    );

    testUsingContext('installHook creates a FlutterPlatform', () {
      expect(
        () => installHook(
          flutterTesterBinPath: 'abc',
          debuggingOptions: DebuggingOptions.enabled(BuildInfo.debug, startPaused: true),
          buildInfo: BuildInfo.debug,
          fileSystem: fileSystem,
          processManager: FakeProcessManager.empty(),
          logger: BufferLogger.test(),
        ),
        throwsAssertionError,
      );

      expect(
        () => installHook(
          flutterTesterBinPath: 'abc',
          debuggingOptions: DebuggingOptions.enabled(
            BuildInfo.debug,
            startPaused: true,
            hostVmServicePort: 123,
          ),
          buildInfo: BuildInfo.debug,
          fileSystem: fileSystem,
          processManager: FakeProcessManager.empty(),
          logger: BufferLogger.test(),
        ),
        throwsAssertionError,
      );

      FlutterPlatform? capturedPlatform;
      final Map<String, String> expectedPrecompiledDillFiles = <String, String>{'Key': 'Value'};
      final FlutterPlatform flutterPlatform = installHook(
        flutterTesterBinPath: 'abc',
        debuggingOptions: DebuggingOptions.enabled(
          BuildInfo.debug,
          startPaused: true,
          disableServiceAuthCodes: true,
          hostVmServicePort: 200,
        ),
        enableVmService: true,
        machine: true,
        precompiledDillPath: 'def',
        precompiledDillFiles: expectedPrecompiledDillFiles,
        updateGoldens: true,
        testAssetDirectory: '/build/test',
        serverType: InternetAddressType.IPv6,
        icudtlPath: 'ghi',
        platformPluginRegistration: (FlutterPlatform platform) {
          capturedPlatform = platform;
        },
        buildInfo: BuildInfo.debug,
        fileSystem: fileSystem,
        processManager: FakeProcessManager.empty(),
        logger: BufferLogger.test(),
      );

      expect(identical(capturedPlatform, flutterPlatform), equals(true));
      expect(flutterPlatform.flutterTesterBinPath, equals('abc'));
      expect(flutterPlatform.debuggingOptions.buildInfo, equals(BuildInfo.debug));
      expect(flutterPlatform.debuggingOptions.startPaused, equals(true));
      expect(flutterPlatform.debuggingOptions.disableServiceAuthCodes, equals(true));
      expect(flutterPlatform.debuggingOptions.hostVmServicePort, equals(200));
      expect(flutterPlatform.enableVmService, equals(true));
      expect(flutterPlatform.machine, equals(true));
      expect(flutterPlatform.host, InternetAddress.loopbackIPv6);
      expect(flutterPlatform.precompiledDillPath, equals('def'));
      expect(flutterPlatform.precompiledDillFiles, expectedPrecompiledDillFiles);
      expect(flutterPlatform.updateGoldens, equals(true));
      expect(flutterPlatform.testAssetDirectory, '/build/test');
      expect(flutterPlatform.icudtlPath, equals('ghi'));
    });
  });

  group('generateTestBootstrap', () {
    group('writes a "const packageConfigLocation" string', () {
      test('with null packageConfigUri', () {
        final String contents = generateTestBootstrap(
          testUrl: Uri.parse('file:///Users/me/some_package/test/some_test.dart'),
          host: InternetAddress('127.0.0.1', type: InternetAddressType.IPv4),
        );
        // IMPORTANT: DO NOT RENAME, REMOVE, OR MODIFY THE
        // 'const packageConfigLocation' VARIABLE.
        // Dash tooling like Dart DevTools performs an evaluation on this variable
        // at runtime to get the package config location for Flutter test targets.
        expect(contents, contains("const packageConfigLocation = 'null';"));
      });

      test('with non-null packageConfigUri', () {
        final String contents = generateTestBootstrap(
          testUrl: Uri.parse('file:///Users/me/some_package/test/some_test.dart'),
          host: InternetAddress('127.0.0.1', type: InternetAddressType.IPv4),
          packageConfigUri: Uri.parse(
            'file:///Users/me/some_package/.dart_tool/package_config.json',
          ),
        );
        // IMPORTANT: DO NOT RENAME, REMOVE, OR MODIFY THE
        // 'const packageConfigLocation' VARIABLE.
        // Dash tooling like Dart DevTools performs an evaluation on this variable
        // at runtime to get the package config location for Flutter test targets.
        expect(
          contents,
          contains(
            "const packageConfigLocation = 'file:///Users/me/some_package/.dart_tool/package_config.json';",
          ),
        );
      });
    });
  });

  group('proxies goldenFileComparator using the VM service driver', () {
    late SuitePlatform fakeSuitePlatform;
    late MemoryFileSystem fileSystem;
    late Artifacts artifacts;
    late FlutterProject flutterProject;
    late FakeProcessManager processManager;
    late BufferLogger logger;
    late TestCompiler testCompiler;
    late _FakeFlutterVmService flutterVmService;
    late Completer<void> testCompleter;

    setUp(() {
      fakeSuitePlatform = SuitePlatform(Runtime.vm);
      fileSystem = MemoryFileSystem.test();
      fileSystem.file('.dart_tool/package_config.json')
        ..createSync(recursive: true)
        ..writeAsStringSync('{"configVersion":2,"packages":[]}');
      artifacts = Artifacts.test(fileSystem: fileSystem);

      flutterProject = FlutterProject.fromDirectoryTest(fileSystem.systemTempDirectory);
      testCompleter = Completer<void>();
      processManager = FakeProcessManager.empty();
      logger = BufferLogger.test();
      testCompiler = _FakeTestCompiler();
      flutterVmService = _FakeFlutterVmService();
    });

    tearDown(() {
      printOnFailure(logger.errorText);
    });

    void addFlutterTesterDeviceExpectation() {
      processManager.addCommand(
        FakeCommand(
          command: const <String>[
            'flutter_tester',
            '--disable-vm-service',
            '--enable-checked-mode',
            '--verify-entry-points',
            '--enable-software-rendering',
            '--skia-deterministic-rendering',
            '--enable-dart-profiling',
            '--non-interactive',
            '--use-test-fonts',
            '--disable-asset-fonts',
            '--packages=.dart_tool/package_config.json',
            'path_to_output.dill',
          ],
          exitCode: -9,
          completer: testCompleter,
        ),
      );
    }

    testUsingContext(
      'should not listen in a non-integration test',
      () async {
        addFlutterTesterDeviceExpectation();

        const Device? notAnIntegrationTest = null;
        final FlutterPlatform flutterPlatform = FlutterPlatform(
          debuggingOptions: DebuggingOptions.disabled(BuildInfo.debug),
          flutterTesterBinPath: 'flutter_tester',
          enableVmService: false,
          // ignore: avoid_redundant_argument_values
          integrationTestDevice: notAnIntegrationTest,
          flutterProject: flutterProject,
          host: InternetAddress.anyIPv4,
          updateGoldens: false,
          buildInfo: BuildInfo.debug,
          fileSystem: fileSystem,
          processManager: processManager,
          logger: BufferLogger.test(),
        );
        flutterPlatform.compiler = testCompiler;

        // Simulate the test immediately completing.
        testCompleter.complete();

        final StreamChannel<Object?> channel = flutterPlatform.loadChannel(
          'test1.dart',
          fakeSuitePlatform,
        );

        // Without draining, the sink will never complete.
        unawaited(channel.stream.drain<void>());

        await expectLater(channel.sink.done, completes);
      },
      overrides: <Type, Generator>{
        FileSystem: () => fileSystem,
        ProcessManager: () => processManager,
        Logger: () => logger,
        VMServiceConnector: () => (_) => throw UnimplementedError(),
      },
    );

    // This is not a complete test of all the possible cases supported by the
    // golden-file integration, which is a complex multi-process implementation
    // that lives across multiple packages and files.
    //
    // Instead, this is a unit-test based smoke test that the overall flow works
    // as expected to give a quicker turn-around signal that either the test
    // should be updated, or the process was broken; run an integration_test on
    // an Android or iOS device or emulator/simulator that takes screenshots
    // and compares them with matchesGoldenFile for a full e2e-test of the
    // entire workflow.
    testUsingContext(
      'should listen in an integration test',
      () async {
        processManager.addCommand(
          const FakeCommand(
            command: <String>[
              'flutter_tester',
              '--disable-vm-service',
              '--non-interactive',
              'path_to_output.dill',
            ],
            stdout: '{"success": true}\n',
          ),
        );
        addFlutterTesterDeviceExpectation();

        final FlutterPlatform flutterPlatform = FlutterPlatform(
          debuggingOptions: DebuggingOptions.disabled(BuildInfo.debug),
          flutterTesterBinPath: 'flutter_tester',
          enableVmService: false,
          flutterProject: flutterProject,
          integrationTestDevice: _WorkingDevice(),
          host: InternetAddress.anyIPv4,
          updateGoldens: false,
          buildInfo: BuildInfo.debug,
          fileSystem: fileSystem,
          processManager: processManager,
          logger: BufferLogger.test(),
        );
        flutterPlatform.compiler = testCompiler;

        final StreamChannel<Object?> channel = flutterPlatform.loadChannel(
          'test1.dart',
          fakeSuitePlatform,
        );

        // Responds to update events.
        flutterVmService.service.onExtensionEventController.add(
          Event(
            extensionData: ExtensionData.parse(<String, Object?>{
              'id': 1,
              'path': 'foo',
              'bytes': '',
            }),
            extensionKind: 'update',
          ),
        );

        // Wait for tiny async tasks to complete.
        await pumpEventQueue();
        await flutterVmService.service.onExtensionEventController.close();

        final (String event, String? isolateId, Map<String, Object?>? data) =
            flutterVmService.callMethodWrapperInvocation!;
        expect(event, 'ext.integration_test.VmServiceProxyGoldenFileComparator');
        expect(isolateId, null);
        expect(data, <String, Object?>{'id': 1, 'result': true});

        // Without draining, the sink will never complete.
        unawaited(channel.stream.drain<void>());

        // Allow the test to finish.
        testCompleter.complete();

        await expectLater(channel.sink.done, completes);
      },
      overrides: <Type, Generator>{
        FileSystem: () => fileSystem,
        ProcessManager: () => processManager,
        Logger: () => logger,
        VMServiceConnector:
            () =>
                (
                  Uri httpUri, {
                  ReloadSources? reloadSources,
                  Restart? restart,
                  CompileExpression? compileExpression,
                  FlutterProject? flutterProject,
                  PrintStructuredErrorLogMethod? printStructuredErrorLogMethod,
                  io.CompressionOptions? compression,
                  Device? device,
                  Logger? logger,
                }) async => flutterVmService,
        ApplicationPackageFactory: _FakeApplicationPackageFactory.new,
        Artifacts: () => artifacts,
      },
    );
  });
}

class _FakeFlutterVmService extends Fake implements FlutterVmService {
  @override
  Future<IsolateRef> findExtensionIsolate(String extensionName) async {
    return IsolateRef();
  }

  @override
  final _FakeVmService service = _FakeVmService();

  (String, String?, Map<String, Object?>?)? callMethodWrapperInvocation;

  @override
  Future<Response?> callMethodWrapper(
    String method, {
    String? isolateId,
    Map<String, Object?>? args,
  }) async {
    callMethodWrapperInvocation = (method, isolateId, args);
    return Response();
  }
}

class _FakeVmService extends Fake implements VmService {
  String? lastStreamListenId;

  @override
  Future<Success> streamListen(String streamId) async {
    lastStreamListenId = streamId;
    return Success();
  }

  final StreamController<Event> onExtensionEventController = StreamController<Event>();

  @override
  Stream<Event> get onExtensionEvent => const Stream<Event>.empty();

  @override
  Stream<Event> onEvent(String streamId) {
    return onExtensionEventController.stream;
  }

  @override
  Future<void> get onDone => onExtensionEventController.done;
}

class _FakeTestCompiler extends Fake implements TestCompiler {
  @override
  Future<TestCompilerResult> compile(Uri mainUri) async {
    return TestCompilerComplete(outputPath: 'path_to_output.dill', mainUri: mainUri);
  }
}

class _UnstartableDevice extends Fake implements Device {
  @override
  Future<void> dispose() => Future<void>.value();

  @override
  Future<TargetPlatform> get targetPlatform => Future<TargetPlatform>.value(TargetPlatform.android);

  @override
  Future<bool> stopApp(ApplicationPackage? app, {String? userIdentifier}) async {
    return true;
  }

  @override
  Future<bool> uninstallApp(ApplicationPackage app, {String? userIdentifier}) async => true;

  @override
  Future<LaunchResult> startApp(
    covariant ApplicationPackage? package, {
    String? mainPath,
    String? route,
    required DebuggingOptions debuggingOptions,
    Map<String, Object?> platformArgs = const <String, Object>{},
    bool prebuiltApplication = false,
    String? userIdentifier,
  }) async {
    return LaunchResult.failed();
  }
}

class _WorkingDevice extends Fake implements Device {
  @override
  Future<void> dispose() async {}

  @override
  Future<TargetPlatform> get targetPlatform => Future<TargetPlatform>.value(TargetPlatform.android);

  @override
  Future<bool> stopApp(ApplicationPackage? app, {String? userIdentifier}) async => true;

  @override
  Future<bool> uninstallApp(ApplicationPackage app, {String? userIdentifier}) async => true;

  @override
  Future<LaunchResult> startApp(
    covariant ApplicationPackage? package, {
    String? mainPath,
    String? route,
    required DebuggingOptions debuggingOptions,
    Map<String, Object?> platformArgs = const <String, Object>{},
    bool prebuiltApplication = false,
    String? userIdentifier,
  }) async {
    return LaunchResult.succeeded(vmServiceUri: Uri.parse('http://127.0.0.1:12345/vmService'));
  }
}

class _FakeFlutterProject extends Fake implements FlutterProject {
  @override
  FlutterManifest get manifest => FlutterManifest.empty(logger: BufferLogger.test());
}

class _FakeApplicationPackageFactory implements ApplicationPackageFactory {
  TargetPlatform? platformRequested;
  File? applicationBinaryRequested;
  ApplicationPackage applicationPackage = _FakeApplicationPackage();

  @override
  Future<ApplicationPackage?> getPackageForPlatform(
    TargetPlatform platform, {
    BuildInfo? buildInfo,
    File? applicationBinary,
  }) async {
    platformRequested = platform;
    applicationBinaryRequested = applicationBinary;
    return applicationPackage;
  }
}

class _FakeApplicationPackage extends Fake implements ApplicationPackage {}
