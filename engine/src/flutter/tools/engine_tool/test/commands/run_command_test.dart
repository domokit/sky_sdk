// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert';
import 'dart:ffi' show Abi;
import 'dart:io' as io;

import 'package:args/command_runner.dart';
import 'package:engine_repo_tools/engine_repo_tools.dart';
import 'package:engine_tool/src/commands/run_command.dart';
import 'package:engine_tool/src/environment.dart';
import 'package:engine_tool/src/flutter_tool_interop/device.dart';
import 'package:engine_tool/src/flutter_tool_interop/flutter_tool.dart';
import 'package:engine_tool/src/logger.dart';
import 'package:path/path.dart' as p;
import 'package:platform/platform.dart';
import 'package:process_fakes/process_fakes.dart';
import 'package:process_runner/process_runner.dart';
import 'package:test/test.dart';

import '../src/test_build_configs.dart';

void main() {
  late io.Directory tempRoot;
  late TestEngine testEngine;

  setUp(() {
    tempRoot = io.Directory.systemTemp.createTempSync('engine_tool_test');
    testEngine = TestEngine.createTemp(rootDir: tempRoot);
  });

  tearDown(() {
    tempRoot.deleteSync(recursive: true);
  });

  test('fails if flutter is not on your PATH', () async {
    final failsCanRun = FakeProcessManager(
      canRun: (executable, {workingDirectory}) {
        if (executable == 'flutter') {
          return false;
        }
        fail('Unexpected');
      },
    );

    final testEnvironment = Environment(
      abi: Abi.macosArm64,
      engine: testEngine,
      logger: Logger.test((_) {}),
      platform: _fakePlatform(Platform.linux),
      processRunner: ProcessRunner(defaultWorkingDirectory: tempRoot, processManager: failsCanRun),
    );

    final et = _engineTool(
      RunCommand(
        environment: testEnvironment,
        // Intentionally left blank, none of these builds make it far enough.
        configs: {},
      ),
    );

    expect(
      () => et.run(['run']),
      throwsA(
        isA<FatalError>().having(
          (a) => a.toString(),
          'toString()',
          contains('"flutter" command in your PATH'),
        ),
      ),
    );
  });

  group('configuration failures', () {
    final unusedProcessManager = FakeProcessManager(canRun: (_, {workingDirectory}) => true);

    late List<LogRecord> testLogs;
    late Environment testEnvironment;
    late _FakeFlutterTool flutterTool;

    setUp(() {
      testLogs = [];
      testEnvironment = Environment(
        abi: Abi.linuxX64,
        engine: testEngine,
        logger: Logger.test(testLogs.add),
        platform: _fakePlatform(Platform.linux),
        processRunner: ProcessRunner(
          defaultWorkingDirectory: tempRoot,
          processManager: unusedProcessManager,
        ),
      );
      flutterTool = _FakeFlutterTool();
    });

    test('fails if a host build could not be found', () async {
      final builders = TestBuilderConfig();
      builders.addBuild(
        name: 'linux/android_debug_arm64',
        dimension: TestDroneDimension.linux,
        targetDir: 'android_debug_arm64',
      );

      final et = _engineTool(
        RunCommand(
          environment: testEnvironment,
          configs: {
            'linux_test_config': builders.buildConfig(path: 'ci/builders/linux_test_config.json'),
          },
          flutterTool: flutterTool,
        ),
      );

      expect(
        () => et.run(['run', '--config=android_debug_arm64']),
        throwsA(
          isA<FatalError>().having(
            (a) => a.toString(),
            'toString()',
            contains('Could not find host build'),
          ),
        ),
      );
    });
  });

  group('builds and executes `flutter run`', () {
    late CommandRunner<int> et;
    late io.Directory rbeDir;

    var commandsRun = <List<String>>[];
    var testLogs = <LogRecord>[];
    var attachedDevices = <Device>[];
    var interceptCommands = <(String, FakeProcess? Function(List<String>))>[];

    setUp(() {
      // Builder configuration doesn't change for these tests.
      final builders = TestBuilderConfig();
      builders.addBuild(
        name: 'linux/android_debug_arm64',
        dimension: TestDroneDimension.linux,
        targetDir: 'android_debug_arm64',
        enableRbe: true,
      );
      builders.addBuild(
        name: 'linux/host_debug',
        dimension: TestDroneDimension.linux,
        targetDir: 'host_debug',
        enableRbe: true,
      );

      // Be very permissive on process execution, and check usage below instead.
      final permissiveProcessManager = FakeProcessManager(
        canRun: (_, {workingDirectory}) => true,
        onRun: (FakeCommandLogEntry entry) {
          commandsRun.add(entry.command);
          return io.ProcessResult(81, 0, '', '');
        },
        onStart: (FakeCommandLogEntry entry) {
          commandsRun.add(entry.command);
          for (final intercept in interceptCommands) {
            if (entry.command.first.endsWith(intercept.$1)) {
              final result = intercept.$2(entry.command);
              if (result != null) {
                return result;
              }
            }
          }
          switch (entry.command) {
            case ['flutter', 'devices', '--machine', ..._]:
              return FakeProcess(stdout: jsonEncode(attachedDevices));
            default:
              return FakeProcess();
          }
        },
      );

      // Create an RBE directory by default.
      rbeDir = io.Directory(p.join(testEngine.srcDir.path, 'flutter', 'build', 'rbe'));
      rbeDir.createSync(recursive: true);

      // Set up the environment for the test.
      final testEnvironment = Environment(
        abi: Abi.linuxX64,
        engine: testEngine,
        logger: Logger.test((log) {
          testLogs.add(log);
        }),
        platform: _fakePlatform(Platform.linux),
        processRunner: ProcessRunner(
          defaultWorkingDirectory: tempRoot,
          processManager: permissiveProcessManager,
        ),
      );

      // Set up the Flutter tool for the test.
      et = _engineTool(
        RunCommand(
          environment: testEnvironment,
          configs: {
            'linux_test_config': builders.buildConfig(path: 'ci/builders/linux_test_config.json'),
          },
        ),
      );

      // Reset logs.
      commandsRun = [];
      attachedDevices = [];
      testLogs = [];
      interceptCommands = [];
    });

    tearDown(() {
      printOnFailure('Commands run:\n${commandsRun.map((c) => c.join('\n'))}');
      printOnFailure('Logs:\n${testLogs.map((l) => l.message).join('\n')}');
    });

    test('build includes RBE flags when enabled implicitly', () async {
      await et.run(['run', '--config=android_debug_arm64']);

      expect(
        commandsRun,
        containsAllInOrder([
          // ./tools/gn --rbe
          containsAllInOrder([endsWith('tools/gn'), contains('--rbe')]),

          // ./reclient/bootstrap
          containsAllInOrder([endsWith('reclient/bootstrap')]),
        ]),
      );
    });

    test('build excludes RBE flags when disabled', () async {
      await et.run(['run', '--config=android_debug_arm64', '--no-rbe']);

      expect(
        commandsRun,
        containsAllInOrder([
          // ./tools/gn --no-rbe
          containsAllInOrder([endsWith('tools/gn'), contains('--no-rbe')]),

          // ./reclient/bootstrap
          isNot(containsAllInOrder([endsWith('reclient/bootstrap')])),
        ]),
      );
    });

    test('picks a default concurrency for RBE builds', () async {
      await et.run(['run', '--config=android_debug_arm64']);

      expect(
        commandsRun,
        containsAllInOrder([
          // ninja -C out/android_debug_arm64 -j 1000 (or whatever is picked)
          containsAllInOrder([
            endsWith('ninja/ninja'),
            contains('-j'),
            isA<String>().having(int.tryParse, 'concurrency', greaterThan(100)),
          ]),
        ]),
      );
    });

    test('does not define a default concurrency for non-RBE builds', () async {
      await et.run(['run', '--no-rbe', '--config=android_debug_arm64']);

      expect(
        commandsRun,
        containsAllInOrder([
          containsAllInOrder([endsWith('ninja/ninja'), isNot(contains('-j'))]),
        ]),
      );
    });

    test('define a user-specified concurrency for non-RBE builds', () async {
      await et.run(['run', '--concurrency=42', '--no-rbe', '--config=android_debug_arm64']);

      expect(
        commandsRun,
        containsAllInOrder([
          containsAllInOrder([endsWith('ninja/ninja'), contains('-j'), '42']),
        ]),
      );
    });

    test('handles host build failures', () async {
      interceptCommands.add((
        'ninja',
        (command) {
          print(command);
          if (command.any((c) => c.contains('host_debug'))) {
            return FakeProcess(exitCode: 1);
          }
          return null;
        },
      ));

      expect(
        () => et.run(['run', '--config=android_debug_arm64']),
        throwsA(
          isA<FatalError>().having(
            (a) => a.toString(),
            'toString()',
            contains('Failed to build host'),
          ),
        ),
      );
    });

    test('handles target build failures', () async {
      interceptCommands.add((
        'ninja',
        (command) {
          if (command.any((c) => c.contains('android_debug_arm64'))) {
            return FakeProcess(exitCode: 1);
          }
          return null;
        },
      ));

      expect(
        () => et.run(['run', '--config=android_debug_arm64']),
        throwsA(
          isA<FatalError>().having(
            (a) => a.toString(),
            'toString()',
            contains('Failed to build target'),
          ),
        ),
      );
    });

    test('builds only once if the target and host are the same', () async {
      await et.run(['run', '--config=host_debug']);

      expect(commandsRun, containsOnce(containsAllInOrder([endsWith('ninja')])));
    });

    test('builds both the target and host if they are different', () async {
      await et.run(['run', '--config=android_debug_arm64']);

      expect(
        commandsRun,
        containsAllInOrder([
          containsAllInOrder([endsWith('ninja'), contains('host_debug')]),
          containsAllInOrder([endsWith('ninja'), contains('android_debug_arm64')]),
        ]),
      );
    });

    test('delegates to `flutter run` with --local-engine flags', () async {
      await et.run(['run', '--config=android_debug_arm64']);

      expect(
        commandsRun,
        containsAllInOrder([
          containsAllInOrder([
            endsWith('flutter'),
            contains('run'),
            '--local-engine-src-path',
            testEngine.srcDir.path,
            '--local-engine',
            'android_debug_arm64',
            '--local-engine-host',
            'host_debug',
          ]),
        ]),
      );
    });

    group('delegates to `flutter run` in mode', () {
      for (final mode in const ['debug', 'profile', 'release']) {
        test('$mode mode', () async {
          await et.run(['run', '--config=android_debug_arm64', '--', '--$mode']);

          expect(
            commandsRun,
            containsAllInOrder([
              containsAllInOrder([endsWith('flutter'), contains('run'), contains('--$mode')]),
            ]),
          );
        });
      }
    });
  });

  // FIXME: Add positive tests.
  // ^^^ Both sets, calls flutter run as expected n stuff.
  // ... and check debug/profile/release
}

CommandRunner<int> _engineTool(RunCommand runCommand) {
  return CommandRunner<int>('et', 'Fake tool with a single instrumented command.')
    ..addCommand(runCommand);
}

Platform _fakePlatform(String os, {int numberOfProcessors = 32, String pathSeparator = '/'}) {
  return FakePlatform(
    operatingSystem: os,
    resolvedExecutable: io.Platform.resolvedExecutable,
    numberOfProcessors: numberOfProcessors,
    pathSeparator: pathSeparator,
  );
}

final class _FakeFlutterTool implements FlutterTool {
  List<Device> respondWithDevices = [];

  @override
  Future<List<Device>> devices() async {
    return respondWithDevices;
  }
}
