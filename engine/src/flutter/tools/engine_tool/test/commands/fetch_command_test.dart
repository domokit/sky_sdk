// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ffi' as ffi show Abi;
import 'dart:io' as io;

import 'package:engine_build_configs/engine_build_configs.dart';
import 'package:engine_repo_tools/engine_repo_tools.dart';
import 'package:engine_tool/src/commands/command_runner.dart';
import 'package:engine_tool/src/environment.dart';
import 'package:engine_tool/src/logger.dart';
import 'package:platform/platform.dart';
import 'package:process_fakes/process_fakes.dart';
import 'package:process_runner/process_runner.dart';
import 'package:test/test.dart';

void main() {
  final Engine? engine = Engine.tryFindWithin();
  if (engine == null) {
    io.stderr.writeln('The current working directory is not a Flutter engine');
    io.exitCode = 1;
    return;
  }

  final Map<String, BuilderConfig> configs = <String, BuilderConfig>{};

  (Environment, List<FakeCommandLogEntry>) linuxEnv(Logger logger) {
    final runHistory = <FakeCommandLogEntry>[];
    return (
      Environment(
        abi: ffi.Abi.linuxX64,
        engine: engine,
        platform: FakePlatform(
          operatingSystem: Platform.linux,
          resolvedExecutable: io.Platform.resolvedExecutable,
          pathSeparator: '/',
        ),
        processRunner: ProcessRunner(
          processManager: FakeProcessManager(
            onStart: (FakeCommandLogEntry entry) {
              runHistory.add(entry);
              return FakeProcess();
            },
            onRun: (FakeCommandLogEntry entry) {
              runHistory.add(entry);
              return io.ProcessResult(81, 0, '', '');
            },
          ),
        ),
        logger: logger,
      ),
      runHistory,
    );
  }

  test('fetch command invokes gclient sync -D', () async {
    final Logger logger = Logger.test((_) {});
    final (Environment env, List<FakeCommandLogEntry> runHistory) = linuxEnv(logger);
    final ToolCommandRunner runner = ToolCommandRunner(environment: env, configs: configs);
    final int result = await runner.run(<String>['fetch']);
    expect(result, equals(0));
    expect(runHistory.length, greaterThanOrEqualTo(1));
    expect(runHistory[0].command, containsAllInOrder(<String>['gclient', 'sync', '-D']));
  });

  test('fetch command has sync alias', () async {
    final Logger logger = Logger.test((_) {});
    final (Environment env, List<FakeCommandLogEntry> runHistory) = linuxEnv(logger);
    final ToolCommandRunner runner = ToolCommandRunner(environment: env, configs: configs);
    final int result = await runner.run(<String>['sync']);
    expect(result, equals(0));
    expect(runHistory.length, greaterThanOrEqualTo(1));
    expect(runHistory[0].command, containsAllInOrder(<String>['gclient', 'sync', '-D']));
    expect(runHistory[0].workingDirectory, env.engine.flutterDir.parent.parent.parent.path);
  });
}
