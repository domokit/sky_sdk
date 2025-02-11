// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:args/command_runner.dart';
import 'package:file/memory.dart';
import 'package:flutter_tools/src/base/file_system.dart';
import 'package:flutter_tools/src/base/logger.dart';
import 'package:flutter_tools/src/build_system/build_system.dart';
import 'package:flutter_tools/src/cache.dart';
import 'package:flutter_tools/src/commands/build.dart';
import 'package:flutter_tools/src/runner/flutter_command.dart';

import '../../src/common.dart';
import '../../src/context.dart';
import '../../src/fakes.dart';
import '../../src/test_build_system.dart';
import '../../src/test_flutter_command_runner.dart';

void main() {
  testUsingContext('obfuscate requires split-debug-info', () {
    final FakeBuildInfoCommand command = FakeBuildInfoCommand();
    final CommandRunner<void> commandRunner = createTestCommandRunner(command);

    expect(
      () => commandRunner.run(<String>['fake', '--obfuscate']),
      throwsToolExit(
        message:
            '"--${FlutterOptions.kDartObfuscationOption}" can only be used in '
            'combination with "--${FlutterOptions.kSplitDebugInfoOption}"',
      ),
    );
  });

  group('Fatal Logs', () {
    late FakeBuildCommand command;
    late MemoryFileSystem fs;
    late BufferLogger logger;
    late ProcessManager processManager;

    setUp(() {
      fs = MemoryFileSystem.test();
      fs.file('/package/pubspec.yaml').createSync(recursive: true);
      fs.currentDirectory = '/package';
      Cache.disableLocking();
      logger = BufferLogger.test();
      processManager = FakeProcessManager.empty();
    });

    testUsingContext(
      "doesn't fail if --fatal-warnings specified and no warnings occur",
      () async {
        command = FakeBuildCommand(
          androidSdk: FakeAndroidSdk(),
          buildSystem: TestBuildSystem.all(BuildResult(success: true)),
          fileSystem: fs,
          logger: logger,
          osUtils: FakeOperatingSystemUtils(),
        );
        try {
          await createTestCommandRunner(
            command,
          ).run(<String>['build', 'test', '--${FlutterOptions.kFatalWarnings}']);
        } on Exception {
          fail('Unexpected exception thrown');
        }
      },
      overrides: <Type, Generator>{FileSystem: () => fs, ProcessManager: () => processManager},
    );

    testUsingContext(
      "doesn't fail if --fatal-warnings not specified",
      () async {
        command = FakeBuildCommand(
          androidSdk: FakeAndroidSdk(),
          buildSystem: TestBuildSystem.all(BuildResult(success: true)),
          fileSystem: fs,
          logger: logger,
          osUtils: FakeOperatingSystemUtils(),
        );
        testLogger.printWarning('Warning: Mild annoyance Will Robinson!');
        try {
          await createTestCommandRunner(command).run(<String>['build', 'test']);
        } on Exception {
          fail('Unexpected exception thrown');
        }
      },
      overrides: <Type, Generator>{FileSystem: () => fs, ProcessManager: () => processManager},
    );

    testUsingContext(
      'fails if --fatal-warnings specified and warnings emitted',
      () async {
        command = FakeBuildCommand(
          androidSdk: FakeAndroidSdk(),
          buildSystem: TestBuildSystem.all(BuildResult(success: true)),
          fileSystem: fs,
          logger: logger,
          osUtils: FakeOperatingSystemUtils(),
        );
        testLogger.printWarning('Warning: Mild annoyance Will Robinson!');
        await expectLater(
          createTestCommandRunner(
            command,
          ).run(<String>['build', 'test', '--${FlutterOptions.kFatalWarnings}']),
          throwsToolExit(
            message:
                'Logger received warning output during the run, and "--${FlutterOptions.kFatalWarnings}" is enabled.',
          ),
        );
      },
      overrides: <Type, Generator>{FileSystem: () => fs, ProcessManager: () => processManager},
    );

    testUsingContext(
      'fails if --fatal-warnings specified and errors emitted',
      () async {
        command = FakeBuildCommand(
          androidSdk: FakeAndroidSdk(),
          buildSystem: TestBuildSystem.all(BuildResult(success: true)),
          fileSystem: fs,
          logger: logger,
          osUtils: FakeOperatingSystemUtils(),
        );
        testLogger.printError('Error: Danger Will Robinson!');
        await expectLater(
          createTestCommandRunner(
            command,
          ).run(<String>['build', 'test', '--${FlutterOptions.kFatalWarnings}']),
          throwsToolExit(
            message:
                'Logger received error output during the run, and "--${FlutterOptions.kFatalWarnings}" is enabled.',
          ),
        );
      },
      overrides: <Type, Generator>{FileSystem: () => fs, ProcessManager: () => processManager},
    );
  });
}

class FakeBuildInfoCommand extends FlutterCommand {
  FakeBuildInfoCommand() : super() {
    addSplitDebugInfoOption();
    addDartObfuscationOption();
  }

  @override
  String get description => '';

  @override
  String get name => 'fake';

  @override
  Future<FlutterCommandResult> runCommand() async {
    await getBuildInfo();
    return FlutterCommandResult.success();
  }
}

class FakeBuildCommand extends BuildCommand {
  FakeBuildCommand({
    required super.fileSystem,
    required super.buildSystem,
    required super.osUtils,
    required Logger logger,
    required super.androidSdk,
    bool verboseHelp = false,
  }) : super(logger: logger) {
    addSubcommand(FakeBuildSubcommand(logger: logger, verboseHelp: verboseHelp));
  }

  @override
  String get description => '';

  @override
  String get name => 'build';

  @override
  Future<FlutterCommandResult> runCommand() async {
    return FlutterCommandResult.success();
  }
}

class FakeBuildSubcommand extends BuildSubCommand {
  FakeBuildSubcommand({required super.logger, required super.verboseHelp});

  @override
  String get description => '';

  @override
  String get name => 'test';

  @override
  Future<FlutterCommandResult> runCommand() async {
    return FlutterCommandResult.success();
  }
}
