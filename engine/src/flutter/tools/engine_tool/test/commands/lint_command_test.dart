// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ffi' as ffi show Abi;
import 'dart:io' as io;

import 'package:engine_repo_tools/engine_repo_tools.dart';
import 'package:engine_tool/src/commands/command_runner.dart';
import 'package:engine_tool/src/environment.dart';
import 'package:engine_tool/src/logger.dart';
import 'package:platform/platform.dart';
import 'package:process_fakes/process_fakes.dart';
import 'package:process_runner/process_runner.dart';
import 'package:test/test.dart';

void main() {
  final engine = Engine.findWithin();

  (Environment, List<List<String>>) macEnv(Logger logger) {
    final runHistory = <List<String>>[];
    return (
      Environment(
        abi: ffi.Abi.macosArm64,
        engine: engine,
        platform: FakePlatform(
          operatingSystem: Platform.macOS,
          resolvedExecutable: io.Platform.resolvedExecutable,
          pathSeparator: '/',
        ),
        processRunner: ProcessRunner(
          processManager: FakeProcessManager(
            onStart: (command) {
              runHistory.add(command);
              return FakeProcess();
            },
            onRun: (command) {
              // Should not be executed.
              assert(false);
              return io.ProcessResult(81, 1, '', '');
            },
          ),
        ),
        logger: logger,
      ),
      runHistory,
    );
  }

  test('invoked linters', () async {
    final Logger logger = Logger.test((_) {});
    final (Environment env, List<List<String>> runHistory) = macEnv(logger);
    final ToolCommandRunner runner = ToolCommandRunner(environment: env, configs: {});
    final int result = await runner.run(<String>['lint']);
    expect(result, equals(0));
    expect(runHistory.length, greaterThanOrEqualTo(4));
    expect(runHistory[0].firstOrNull, contains('analyze.sh'));
  });
}
