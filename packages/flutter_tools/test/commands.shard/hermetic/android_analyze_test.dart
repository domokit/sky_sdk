// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:args/command_runner.dart';
import 'package:file/file.dart';
import 'package:file/memory.dart';
import 'package:flutter_tools/src/android/android_builder.dart';
import 'package:flutter_tools/src/artifacts.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/platform.dart';
import 'package:flutter_tools/src/base/terminal.dart';
import 'package:flutter_tools/src/cache.dart';
import 'package:flutter_tools/src/commands/analyze.dart';
import 'package:flutter_tools/src/project.dart';
import 'package:flutter_tools/src/project_validator.dart';
import 'package:test/fake.dart';

import '../../src/context.dart';
import '../../src/test_flutter_command_runner.dart';

void main() {
  testUsingContext('Android analyze command should run pub', () async {
    final FileSystem fileSystem = MemoryFileSystem.test();
    final Platform platform = FakePlatform();
    final BufferLogger logger = BufferLogger.test();
    final FakeProcessManager processManager = FakeProcessManager.empty();
    final Terminal terminal = Terminal.test();
    final AnalyzeCommand command = FakeAndroidAnalyzeCommand(
      artifacts: Artifacts.test(),
      fileSystem: fileSystem,
      logger: logger,
      platform: platform,
      processManager: processManager,
      terminal: terminal,
      allProjectValidators: <ProjectValidator>[],
      suppressAnalytics: true,
    );
    fileSystem.currentDirectory.childFile('pubspec.yaml').createSync();
    expect(command.shouldRunPub, isTrue);
  });

  group('Android analyze command', () {
    late FileSystem fileSystem;
    late Platform platform;
    late BufferLogger logger;
    late FakeProcessManager processManager;
    late Terminal terminal;
    late AnalyzeCommand command;
    late CommandRunner<void> runner;
    late Directory tempDir;
    late FakeAndroidBuilder builder;

    setUpAll(() {
      Cache.disableLocking();
    });

    setUp(() async {
      fileSystem = MemoryFileSystem.test();
      platform = FakePlatform();
      logger = BufferLogger.test();
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
      tempDir = fileSystem.systemTempDirectory.createTempSync('flutter_tools_packages_test.');
      tempDir.childDirectory('android').createSync();

      // Setup repo roots
      const String homePath = '/home/user/flutter';
      Cache.flutterRoot = homePath;
      for (final String dir in <String>['dev', 'examples', 'packages']) {
        fileSystem.directory(homePath).childDirectory(dir).createSync(recursive: true);
      }
      builder = FakeAndroidBuilder();
    });

    testUsingContext('can list build variants', () async {
      builder.variants = <String>['debug', 'release'];
      await runner.run(<String>['analyze', '--android', '--list-build-variants', tempDir.path]);
      expect(logger.statusText, contains('["debug","release"]'));
    }, overrides: <Type, Generator>{AndroidBuilder: () => builder});

    testUsingContext('throw if provide multiple path', () async {
      final Directory anotherTempDir = fileSystem.systemTempDirectory.createTempSync('another');
      await expectLater(
        runner.run(<String>[
          'analyze',
          '--android',
          '--list-build-variants',
          tempDir.path,
          anotherTempDir.path,
        ]),
        throwsA(
          isA<Exception>().having(
            (Exception e) => e.toString(),
            'description',
            contains('The Android analyze can process only one directory path'),
          ),
        ),
      );
    });

    testUsingContext('can output app link settings', () async {
      const String buildVariant = 'release';
      await runner.run(<String>[
        'analyze',
        '--android',
        '--output-app-link-settings',
        '--build-variant=$buildVariant',
        tempDir.path,
      ]);
      expect(builder.outputVariant, buildVariant);
      expect(logger.statusText, contains(builder.outputPath));
    }, overrides: <Type, Generator>{AndroidBuilder: () => builder});

    testUsingContext('output app link settings throws if no build variant', () async {
      await expectLater(
        runner.run(<String>['analyze', '--android', '--output-app-link-settings', tempDir.path]),
        throwsA(
          isA<Exception>().having(
            (Exception e) => e.toString(),
            'description',
            contains('"--build-variant" must be provided'),
          ),
        ),
      );
    });
  });
}

class FakeAndroidBuilder extends Fake implements AndroidBuilder {
  List<String> variants = const <String>[];
  String? outputVariant;
  final String outputPath = '/';

  @override
  Future<List<String>> getBuildVariants({required FlutterProject project}) async {
    return variants;
  }

  @override
  Future<String> outputsAppLinkSettings(
    String buildVariant, {
    required FlutterProject project,
  }) async {
    outputVariant = buildVariant;
    return outputPath;
  }
}

class FakeAndroidAnalyzeCommand extends AnalyzeCommand {
  FakeAndroidAnalyzeCommand({
    required super.fileSystem,
    required super.platform,
    required super.terminal,
    required super.logger,
    required super.processManager,
    required super.artifacts,
    required super.allProjectValidators,
    required super.suppressAnalytics,
  });

  @override
  bool boolArg(String arg, {bool global = false}) {
    switch (arg) {
      case 'current-package':
        return true;
      case 'android':
        return true;
      case 'pub':
        return true;
      default:
        return false;
    }
  }
}
