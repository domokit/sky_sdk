// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ffi' as ffi show Abi;
import 'dart:io' as io;

import 'package:engine_build_configs/engine_build_configs.dart';
import 'package:engine_repo_tools/engine_repo_tools.dart';
import 'package:engine_tool/src/commands/command_runner.dart';
import 'package:engine_tool/src/commands/flags.dart';
import 'package:engine_tool/src/environment.dart';
import 'package:engine_tool/src/logger.dart';
import 'package:logging/logging.dart' as log;
import 'package:platform/platform.dart';
import 'package:process_fakes/process_fakes.dart';
import 'package:process_runner/process_runner.dart';
import 'package:test/test.dart';

void main() {
  final Engine engine;
  try {
    engine = Engine.findWithin();
  } catch (e) {
    io.stderr.writeln(e);
    io.exitCode = 1;
    return;
  }

  Environment linuxEnv(Logger logger, FakeProcessManager processManager) {
    return Environment(
      abi: ffi.Abi.linuxX64,
      engine: engine,
      platform: FakePlatform(
        resolvedExecutable: '/dart',
        operatingSystem: Platform.linux,
        pathSeparator: '/',
      ),
      processRunner: ProcessRunner(processManager: processManager),
      logger: logger,
    );
  }

  List<String> stringsFromLogs(List<log.LogRecord> logs) {
    return logs.map((log.LogRecord r) => r.message).toList();
  }

  test('--fix is passed to ci/bin/format.dart by default', () async {
    final Logger logger = Logger.test((_) {});
    final FakeProcessManager manager = _formatProcessManager(expectedFlags: <String>['--fix']);
    final Environment env = linuxEnv(logger, manager);
    final ToolCommandRunner runner = ToolCommandRunner(
      environment: env,
      configs: <String, BuilderConfig>{},
    );
    final int result = await runner.run(<String>['format']);
    expect(result, equals(0));
  });

  test('--fix is not passed to ci/bin/format.dart with --dry-run', () async {
    final Logger logger = Logger.test((_) {});
    final FakeProcessManager manager = _formatProcessManager(expectedFlags: <String>[]);
    final Environment env = linuxEnv(logger, manager);
    final ToolCommandRunner runner = ToolCommandRunner(
      environment: env,
      configs: <String, BuilderConfig>{},
    );
    final int result = await runner.run(<String>['format', '--$dryRunFlag']);
    expect(result, equals(0));
  });

  test('exit code is non-zero when ci/bin/format.dart exit code was non zero', () async {
    final Logger logger = Logger.test((_) {});
    final FakeProcessManager manager = _formatProcessManager(
      expectedFlags: <String>['--fix'],
      exitCode: 1,
    );
    final Environment env = linuxEnv(logger, manager);
    final ToolCommandRunner runner = ToolCommandRunner(
      environment: env,
      configs: <String, BuilderConfig>{},
    );
    final int result = await runner.run(<String>['format']);
    expect(result, equals(1));
  });

  test('--all-files is passed to ci/bin/format.dart correctly', () async {
    final Logger logger = Logger.test((_) {});
    final FakeProcessManager manager = _formatProcessManager(
      expectedFlags: <String>['--fix', '--all-files'],
    );
    final Environment env = linuxEnv(logger, manager);
    final ToolCommandRunner runner = ToolCommandRunner(
      environment: env,
      configs: <String, BuilderConfig>{},
    );
    final int result = await runner.run(<String>['format', '--$allFlag']);
    expect(result, equals(0));
  });

  test('--verbose is passed to ci/bin/format.dart correctly', () async {
    final Logger logger = Logger.test((_) {});
    final FakeProcessManager manager = _formatProcessManager(
      expectedFlags: <String>['--fix', '--verbose'],
    );
    final Environment env = linuxEnv(logger, manager);
    final ToolCommandRunner runner = ToolCommandRunner(
      environment: env,
      configs: <String, BuilderConfig>{},
    );
    final int result = await runner.run(<String>['format', '--$verboseFlag']);
    expect(result, equals(0));
  });

  test('--quiet suppresses non-error output', () async {
    final List<LogRecord> testLogs = <LogRecord>[];
    final Logger logger = Logger.test(testLogs.add);
    final FakeProcessManager manager = _formatProcessManager(
      expectedFlags: <String>['--fix'],
      stdout: <String>['many', 'lines', 'of', 'output'].join('\n'),
      stderr: 'error\n',
    );
    final Environment env = linuxEnv(logger, manager);
    final ToolCommandRunner runner = ToolCommandRunner(
      environment: env,
      configs: <String, BuilderConfig>{},
    );
    final int result = await runner.run(<String>['format', '--$quietFlag']);
    expect(result, equals(0));
    expect(stringsFromLogs(testLogs), equals(<String>['error\n']));
  });

  test('Diffs are suppressed by default', () async {
    final List<LogRecord> testLogs = <LogRecord>[];
    final Logger logger = Logger.test(testLogs.add);
    final FakeProcessManager manager = _formatProcessManager(
      expectedFlags: <String>['--fix'],
      stdout: <String>[
        'To fix, run `et format --all` or:',
        'many',
        'lines',
        'of',
        'output',
        'DONE',
      ].join('\n'),
    );
    final Environment env = linuxEnv(logger, manager);
    final ToolCommandRunner runner = ToolCommandRunner(
      environment: env,
      configs: <String, BuilderConfig>{},
    );
    final int result = await runner.run(<String>['format']);
    expect(result, equals(0));
    expect(stringsFromLogs(testLogs), isEmpty);
  });

  test('--dry-run disables --fix and prints diffs', () async {
    final List<LogRecord> testLogs = <LogRecord>[];
    final Logger logger = Logger.test(testLogs.add);
    final FakeProcessManager manager = _formatProcessManager(
      expectedFlags: <String>[],
      stdout: <String>[
        'To fix, run `et format --all` or:',
        'many',
        'lines',
        'of',
        'output',
        'DONE',
      ].join('\n'),
    );
    final Environment env = linuxEnv(logger, manager);
    final ToolCommandRunner runner = ToolCommandRunner(
      environment: env,
      configs: <String, BuilderConfig>{},
    );
    final int result = await runner.run(<String>['format', '--$dryRunFlag']);
    expect(result, equals(0));
    expect(
      stringsFromLogs(testLogs),
      equals(<String>[
        'To fix, run `et format --all` or:\n',
        'many\n',
        'lines\n',
        'of\n',
        'output\n',
        'DONE\n',
      ]),
    );
  });

  test('progress lines are followed by a carriage return', () async {
    final List<LogRecord> testLogs = <LogRecord>[];
    final Logger logger = Logger.test(testLogs.add);
    const String progressLine =
        'diff Jobs:  46% done, 1528/3301 completed,  '
        '7 in progress, 1753 pending,  13 failed.';
    final FakeProcessManager manager = _formatProcessManager(
      expectedFlags: <String>['--fix'],
      stdout: progressLine,
    );
    final Environment env = linuxEnv(logger, manager);
    final ToolCommandRunner runner = ToolCommandRunner(
      environment: env,
      configs: <String, BuilderConfig>{},
    );
    final int result = await runner.run(<String>['format']);
    expect(result, equals(0));
    expect(stringsFromLogs(testLogs), equals(<String>['$progressLine\r']));
  });
}

FakeProcessManager _formatProcessManager({
  required List<String> expectedFlags,
  int exitCode = 0,
  String stdout = '',
  String stderr = '',
  bool Function(Object?, {String? workingDirectory})? canRun,
  bool failUnknown = true,
}) {
  final io.ProcessResult success = io.ProcessResult(1, 0, '', '');
  final FakeProcess formatProcess = FakeProcess(exitCode: exitCode, stdout: stdout, stderr: stderr);
  return FakeProcessManager(
    canRun: canRun ?? (Object? exe, {String? workingDirectory}) => true,
    onRun:
        (FakeCommandLogEntry entry) => switch (entry.command) {
          _ => failUnknown ? io.ProcessResult(1, 1, '', '') : success,
        },
    onStart:
        (FakeCommandLogEntry entry) => switch (entry.command) {
          [final String exe, final String fmt, ...final List<String> rest]
              when exe.endsWith('dart') &&
                  fmt.endsWith('ci/bin/format.dart') &&
                  rest.length == expectedFlags.length &&
                  expectedFlags.every(rest.contains) =>
            formatProcess,
          _ => failUnknown ? FakeProcess(exitCode: 1) : FakeProcess(),
        },
  );
}
