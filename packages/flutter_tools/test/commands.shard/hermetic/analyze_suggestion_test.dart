// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:args/command_runner.dart';
import 'package:file/memory.dart';
import 'package:flutter_tools/src/artifacts.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/base/platform.dart';
import 'package:flutter_tools/src/base/terminal.dart';
import 'package:flutter_tools/src/commands/analyze.dart';
import 'package:flutter_tools/src/project.dart';
import 'package:flutter_tools/src/project_validator.dart';
import 'package:flutter_tools/src/project_validator_result.dart';

import '../../src/common.dart';
import '../../src/context.dart';
import '../../src/test_flutter_command_runner.dart';

class ProjectValidatorDummy extends ProjectValidator {
  @override
  Future<List<ProjectValidatorResult>> start(
    FlutterProject project, {
    Logger? logger,
    FileSystem? fileSystem,
  }) async {
    return <ProjectValidatorResult>[
      const ProjectValidatorResult(
        name: 'pass',
        value: 'value',
        status: StatusProjectValidator.success,
      ),
      const ProjectValidatorResult(
        name: 'fail',
        value: 'my error',
        status: StatusProjectValidator.error,
      ),
      const ProjectValidatorResult(
        name: 'pass two',
        value: 'pass',
        warning: 'my warning',
        status: StatusProjectValidator.warning,
      ),
    ];
  }

  @override
  bool supportsProject(FlutterProject project) {
    return true;
  }

  @override
  String get title => 'First Dummy';
}

class ProjectValidatorSecondDummy extends ProjectValidator {
  @override
  Future<List<ProjectValidatorResult>> start(
    FlutterProject project, {
    Logger? logger,
    FileSystem? fileSystem,
  }) async {
    return <ProjectValidatorResult>[
      const ProjectValidatorResult(
        name: 'second',
        value: 'pass',
        status: StatusProjectValidator.success,
      ),
      const ProjectValidatorResult(
        name: 'other fail',
        value: 'second fail',
        status: StatusProjectValidator.error,
      ),
    ];
  }

  @override
  bool supportsProject(FlutterProject project) {
    return true;
  }

  @override
  String get title => 'Second Dummy';
}

class ProjectValidatorCrash extends ProjectValidator {
  @override
  Future<List<ProjectValidatorResult>> start(
    FlutterProject project, {
    Logger? logger,
    FileSystem? fileSystem,
  }) async {
    throw Exception('my exception');
  }

  @override
  bool supportsProject(FlutterProject project) {
    return true;
  }

  @override
  String get title => 'Crash';
}

void main() {
  late FileSystem fileSystem;
  late Terminal terminal;
  late ProcessManager processManager;
  late Platform platform;

  group('analyze --suggestions command', () {
    setUp(() {
      fileSystem = MemoryFileSystem.test();
      terminal = Terminal.test();
      processManager = FakeProcessManager.empty();
      platform = FakePlatform();
    });

    testUsingContext('success, error and warning', () async {
      final BufferLogger loggerTest = BufferLogger.test();
      final AnalyzeCommand command = AnalyzeCommand(
        artifacts: Artifacts.test(),
        fileSystem: fileSystem,
        logger: loggerTest,
        platform: platform,
        terminal: terminal,
        processManager: processManager,
        allProjectValidators: <ProjectValidator>[
          ProjectValidatorDummy(),
          ProjectValidatorSecondDummy(),
        ],
        suppressAnalytics: true,
      );
      final CommandRunner<void> runner = createTestCommandRunner(command);

      await runner.run(<String>['analyze', '--suggestions', './']);

      const String expected =
          '\n'
          '┌──────────────────────────────────────────┐\n'
          '│ First Dummy                              │\n'
          '│ [✓] pass: value                          │\n'
          '│ [✗] fail: my error                       │\n'
          '│ [!] pass two: pass (warning: my warning) │\n'
          '│ Second Dummy                             │\n'
          '│ [✓] second: pass                         │\n'
          '│ [✗] other fail: second fail              │\n'
          '└──────────────────────────────────────────┘\n';

      expect(loggerTest.statusText, contains(expected));
    });

    testUsingContext('crash', () async {
      final BufferLogger loggerTest = BufferLogger.test();
      final AnalyzeCommand command = AnalyzeCommand(
        artifacts: Artifacts.test(),
        fileSystem: fileSystem,
        logger: loggerTest,
        platform: platform,
        terminal: terminal,
        processManager: processManager,
        allProjectValidators: <ProjectValidator>[ProjectValidatorCrash()],
        suppressAnalytics: true,
      );
      final CommandRunner<void> runner = createTestCommandRunner(command);

      await runner.run(<String>['analyze', '--suggestions', './']);

      const String expected = '[☠] Exception: my exception: #0      ProjectValidatorCrash.start';

      expect(loggerTest.statusText, contains(expected));
    });

    testUsingContext('--watch and --suggestions not compatible together', () async {
      final BufferLogger loggerTest = BufferLogger.test();
      final AnalyzeCommand command = AnalyzeCommand(
        artifacts: Artifacts.test(),
        fileSystem: fileSystem,
        logger: loggerTest,
        platform: platform,
        terminal: terminal,
        processManager: processManager,
        allProjectValidators: <ProjectValidator>[],
        suppressAnalytics: true,
      );
      final CommandRunner<void> runner = createTestCommandRunner(command);
      Future<void> result() => runner.run(<String>['analyze', '--suggestions', '--watch']);

      expect(result, throwsToolExit(message: 'flag --watch is not compatible with --suggestions'));
    });
  });
}
