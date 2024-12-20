// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:file/file.dart';
import 'package:file/memory.dart';
import 'package:flutter_tools/src/application_package.dart';
import 'package:flutter_tools/src/artifacts.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/platform.dart';
import 'package:flutter_tools/src/build_info.dart';
import 'package:flutter_tools/src/build_system/build_system.dart';
import 'package:flutter_tools/src/device.dart';
import 'package:flutter_tools/src/tester/flutter_tester.dart';
import 'package:test/fake.dart';

import '../../src/common.dart';
import '../../src/context.dart';
import '../../src/fake_process_manager.dart';
import '../../src/fakes.dart';
import '../../src/test_build_system.dart';

void main() {
  late MemoryFileSystem fileSystem;

  setUp(() {
    fileSystem = MemoryFileSystem.test();
  });

  testWithoutContext('FlutterTesterApp can be created from the current directory', () async {
    const String projectPath = '/home/my/projects/my_project';
    await fileSystem.directory(projectPath).create(recursive: true);
    fileSystem.currentDirectory = projectPath;

    final FlutterTesterApp app = FlutterTesterApp.fromCurrentDirectory(fileSystem);

    expect(app.name, 'my_project');
  });

  group('FlutterTesterDevices', () {
    tearDown(() {
      FlutterTesterDevices.showFlutterTesterDevice = false;
    });

    testWithoutContext('no device', () async {
      final FlutterTesterDevices discoverer = setUpFlutterTesterDevices();

      final List<Device> devices = await discoverer.devices();
      expect(devices, isEmpty);
    });

    testWithoutContext('has device', () async {
      FlutterTesterDevices.showFlutterTesterDevice = true;
      final FlutterTesterDevices discoverer = setUpFlutterTesterDevices();

      final List<Device> devices = await discoverer.devices();
      expect(devices, hasLength(1));

      final Device device = devices.single;
      expect(device, isA<FlutterTesterDevice>());
      expect(device.id, 'flutter-tester');
    });

    testWithoutContext('discoverDevices', () async {
      FlutterTesterDevices.showFlutterTesterDevice = true;
      final FlutterTesterDevices discoverer = setUpFlutterTesterDevices();

      // Timeout ignored.
      final List<Device> devices = await discoverer.discoverDevices(
        timeout: const Duration(seconds: 10),
      );
      expect(devices, hasLength(1));
    });
  });

  group('startApp', () {
    late FlutterTesterDevice device;
    late List<String> logLines;
    String? mainPath;

    late FakeProcessManager fakeProcessManager;
    late TestBuildSystem buildSystem;

    final Map<Type, Generator> startOverrides = <Type, Generator>{
      Platform: () => FakePlatform(),
      FileSystem: () => fileSystem,
      ProcessManager: () => fakeProcessManager,
      Artifacts: () => Artifacts.test(),
      BuildSystem: () => buildSystem,
    };

    setUp(() {
      buildSystem = TestBuildSystem.all(BuildResult(success: true));
      fakeProcessManager = FakeProcessManager.empty();
      device = FlutterTesterDevice(
        'flutter-tester',
        fileSystem: fileSystem,
        processManager: fakeProcessManager,
        artifacts: Artifacts.test(),
        logger: BufferLogger.test(),
        flutterVersion: FakeFlutterVersion(),
      );
      logLines = <String>[];
      device.getLogReader().logLines.listen(logLines.add);
    });

    testWithoutContext('default settings', () async {
      expect(device.id, 'flutter-tester');
      expect(await device.isLocalEmulator, isFalse);
      expect(device.name, 'Flutter test device');
      expect(device.portForwarder, isNot(isNull));
      expect(await device.targetPlatform, TargetPlatform.tester);

      expect(await device.installApp(FakeApplicationPackage()), isTrue);
      expect(await device.isAppInstalled(FakeApplicationPackage()), isFalse);
      expect(await device.isLatestBuildInstalled(FakeApplicationPackage()), isFalse);
      expect(await device.uninstallApp(FakeApplicationPackage()), isTrue);

      expect(device.isSupported(), isTrue);
    });

    testWithoutContext('does not accept profile, release, or jit-release builds', () async {
      final LaunchResult releaseResult = await device.startApp(
        FakeApplicationPackage(),
        mainPath: mainPath,
        debuggingOptions: DebuggingOptions.disabled(BuildInfo.release),
      );
      final LaunchResult profileResult = await device.startApp(
        FakeApplicationPackage(),
        mainPath: mainPath,
        debuggingOptions: DebuggingOptions.disabled(BuildInfo.profile),
      );
      final LaunchResult jitReleaseResult = await device.startApp(
        FakeApplicationPackage(),
        mainPath: mainPath,
        debuggingOptions: DebuggingOptions.disabled(BuildInfo.jitRelease),
      );

      expect(releaseResult.started, isFalse);
      expect(profileResult.started, isFalse);
      expect(jitReleaseResult.started, isFalse);
    });

    testUsingContext('performs a build and starts in debug mode', () async {
      final FlutterTesterApp app = FlutterTesterApp.fromCurrentDirectory(fileSystem);
      final Uri vmServiceUri = Uri.parse('http://127.0.0.1:6666/');
      final Completer<void> completer = Completer<void>();
      fakeProcessManager.addCommand(
        FakeCommand(
          command: const <String>[
            'Artifact.flutterTester',
            '--run-forever',
            '--non-interactive',
            '--enable-dart-profiling',
            '--packages=.dart_tool/package_config.json',
            '--flutter-assets-dir=/.tmp_rand0/flutter_tester.rand0',
            '/.tmp_rand0/flutter_tester.rand0/flutter-tester-app.dill',
          ],
          completer: completer,
          stdout: '''
The Dart VM service is listening on $vmServiceUri
Hello!
''',
        ),
      );

      final LaunchResult result = await device.startApp(
        app,
        mainPath: mainPath,
        debuggingOptions: DebuggingOptions.enabled(
          const BuildInfo(
            BuildMode.debug,
            null,
            treeShakeIcons: false,
            packageConfigPath: '.dart_tool/package_config.json',
          ),
        ),
      );
      expect(result.started, isTrue);
      expect(result.vmServiceUri, vmServiceUri);
      expect(logLines.last, 'Hello!');
      expect(fakeProcessManager, hasNoRemainingExpectations);
    }, overrides: startOverrides);

    testUsingContext(
      'performs a build and starts in debug mode with track-widget-creation',
      () async {
        final FlutterTesterApp app = FlutterTesterApp.fromCurrentDirectory(fileSystem);
        final Uri vmServiceUri = Uri.parse('http://127.0.0.1:6666/');
        final Completer<void> completer = Completer<void>();
        fakeProcessManager.addCommand(
          FakeCommand(
            command: const <String>[
              'Artifact.flutterTester',
              '--run-forever',
              '--non-interactive',
              '--enable-dart-profiling',
              '--packages=.dart_tool/package_config.json',
              '--flutter-assets-dir=/.tmp_rand0/flutter_tester.rand0',
              '/.tmp_rand0/flutter_tester.rand0/flutter-tester-app.dill.track.dill',
            ],
            completer: completer,
            stdout: '''
The Dart VM service is listening on $vmServiceUri
Hello!
''',
          ),
        );

        final LaunchResult result = await device.startApp(
          app,
          mainPath: mainPath,
          debuggingOptions: DebuggingOptions.enabled(BuildInfo.debug),
        );

        expect(result.started, isTrue);
        expect(result.vmServiceUri, vmServiceUri);
        expect(logLines.last, 'Hello!');
        expect(fakeProcessManager, hasNoRemainingExpectations);
      },
      overrides: startOverrides,
    );
  });
}

FlutterTesterDevices setUpFlutterTesterDevices() {
  return FlutterTesterDevices(
    logger: BufferLogger.test(),
    artifacts: Artifacts.test(),
    processManager: FakeProcessManager.any(),
    fileSystem: MemoryFileSystem.test(),
    flutterVersion: FakeFlutterVersion(),
  );
}

class FakeApplicationPackage extends Fake implements ApplicationPackage {}
