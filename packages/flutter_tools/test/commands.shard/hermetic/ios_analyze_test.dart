// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:convert';

import 'package:args/command_runner.dart';
import 'package:file/memory.dart';
import 'package:flutter_tools/src/artifacts.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/platform.dart';
import 'package:flutter_tools/src/base/terminal.dart';
import 'package:flutter_tools/src/cache.dart';
import 'package:flutter_tools/src/commands/analyze.dart';
import 'package:flutter_tools/src/commands/ios_analyze.dart';
import 'package:flutter_tools/src/ios/xcodeproj.dart';
import 'package:flutter_tools/src/project.dart';
import 'package:flutter_tools/src/project_validator.dart';
import 'package:test/fake.dart';

import '../../src/common.dart';
import '../../src/context.dart';
import '../../src/test_flutter_command_runner.dart';

void main() {
  group('ios analyze command', () {
    late BufferLogger logger;
    late FileSystem fileSystem;
    late Platform platform;
    late FakeProcessManager processManager;
    late Terminal terminal;
    late AnalyzeCommand command;
    late CommandRunner<void> runner;

    setUpAll(() {
      Cache.disableLocking();
    });

    setUp(() {
      logger = BufferLogger.test();
      fileSystem = MemoryFileSystem.test();
      platform = FakePlatform();
      processManager = FakeProcessManager.empty();
      terminal = Terminal.test();
      command = AnalyzeCommand(
        artifacts: Artifacts.test(),
        fileSystem: fileSystem,
        logger: logger,
        platform: platform,
        processManager: processManager,
        terminal: terminal,
        allProjectValidators: <ProjectValidator>[],
        suppressAnalytics: true,
      );
      runner = createTestCommandRunner(command);

      // Setup repo roots
      const String homePath = '/home/user/flutter';
      Cache.flutterRoot = homePath;
      for (final String dir in <String>['dev', 'examples', 'packages']) {
        fileSystem.directory(homePath).childDirectory(dir).createSync(recursive: true);
      }
    });

    testWithoutContext('can output json file', () async {
      final MockIosProject ios = MockIosProject();
      final MockFlutterProject project = MockFlutterProject(ios);
      const String expectedConfig = 'someConfig';
      const String expectedTarget = 'someTarget';
      const String expectedOutputFile = '/someFile';
      ios.outputFileLocation = expectedOutputFile;
      await IOSAnalyze(
        project: project,
        option: IOSAnalyzeOption.outputUniversalLinkSettings,
        configuration: expectedConfig,
        target: expectedTarget,
        logger: logger,
      ).analyze();
      expect(logger.statusText, contains(expectedOutputFile));
      expect(ios.outputConfiguration, expectedConfig);
      expect(ios.outputTarget, expectedTarget);
    });

    testWithoutContext('can list build options', () async {
      final MockIosProject ios = MockIosProject();
      final MockFlutterProject project = MockFlutterProject(ios);
      const List<String> targets = <String>['target1', 'target2'];
      const List<String> configs = <String>['config1', 'config2'];
      ios.expectedProjectInfo = XcodeProjectInfo(targets, configs, const <String>[], logger);
      await IOSAnalyze(
        project: project,
        option: IOSAnalyzeOption.listBuildOptions,
        logger: logger,
      ).analyze();
      final Map<String, Object?> jsonOutput = jsonDecode(logger.statusText) as Map<String, Object?>;
      expect(jsonOutput['targets'], unorderedEquals(targets));
      expect(jsonOutput['configurations'], unorderedEquals(configs));
    });

    testUsingContext('throws if provide multiple path', () async {
      final Directory tempDir = fileSystem.systemTempDirectory.createTempSync('someTemp');
      final Directory anotherTempDir = fileSystem.systemTempDirectory.createTempSync('another');
      await expectLater(
        runner.run(<String>[
          'analyze',
          '--ios',
          '--list-build-options',
          tempDir.path,
          anotherTempDir.path,
        ]),
        throwsA(
          isA<Exception>().having(
            (Exception e) => e.toString(),
            'description',
            contains('The iOS analyze can process only one directory path'),
          ),
        ),
      );
    });

    testUsingContext('throws if not enough parameters', () async {
      final Directory tempDir = fileSystem.systemTempDirectory.createTempSync('someTemp');
      await expectLater(
        runner.run(<String>['analyze', '--ios', '--output-universal-link-settings', tempDir.path]),
        throwsA(
          isA<Exception>().having(
            (Exception e) => e.toString(),
            'description',
            contains('"--configuration" must be provided'),
          ),
        ),
      );
    });
  });
}

class MockFlutterProject extends Fake implements FlutterProject {
  MockFlutterProject(this.ios);

  @override
  final IosProject ios;
}

class MockIosProject extends Fake implements IosProject {
  String? outputConfiguration;
  String? outputTarget;
  late String outputFileLocation;
  late XcodeProjectInfo expectedProjectInfo;

  @override
  Future<String> outputsUniversalLinkSettings({
    required String configuration,
    required String target,
  }) async {
    outputConfiguration = configuration;
    outputTarget = target;
    return outputFileLocation;
  }

  @override
  Future<XcodeProjectInfo> projectInfo() async => expectedProjectInfo;
}
