// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/memory.dart';
import 'package:flutter_tools/src/application_package.dart';
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
import 'package:test/fake.dart';
import 'package:test_core/backend.dart';

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
          shellPath: '/',
          debuggingOptions: DebuggingOptions.enabled(BuildInfo.debug, hostVmServicePort: 1234),
          enableVmService: false,
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
          shellPath: '/',
          precompiledDillPath: 'example.dill',
          enableVmService: false,
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
          shellPath: '/',
          enableVmService: false,
          integrationTestDevice: testDevice,
          flutterProject: _FakeFlutterProject(),
          host: InternetAddress.anyIPv4,
          updateGoldens: false,
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
          shellPath: '/',
          enableVmService: false,
          integrationTestDevice: testDevice,
          flutterProject: _FakeFlutterProject(),
          host: InternetAddress.anyIPv4,
          updateGoldens: false,
          shutdownHooks: shutdownHooks,
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
          shellPath: 'abc',
          debuggingOptions: DebuggingOptions.enabled(BuildInfo.debug, startPaused: true),
        ),
        throwsAssertionError,
      );

      expect(
        () => installHook(
          shellPath: 'abc',
          debuggingOptions: DebuggingOptions.enabled(
            BuildInfo.debug,
            startPaused: true,
            hostVmServicePort: 123,
          ),
        ),
        throwsAssertionError,
      );

      FlutterPlatform? capturedPlatform;
      final Map<String, String> expectedPrecompiledDillFiles = <String, String>{'Key': 'Value'};
      final FlutterPlatform flutterPlatform = installHook(
        shellPath: 'abc',
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
      );

      expect(identical(capturedPlatform, flutterPlatform), equals(true));
      expect(flutterPlatform.shellPath, equals('abc'));
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
