// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ffi' as ffi show Abi;
import 'dart:io' as io;

import 'package:engine_repo_tools/engine_repo_tools.dart';
import 'package:engine_tool/src/environment.dart';
import 'package:engine_tool/src/logger.dart';
import 'package:engine_tool/src/proc_utils.dart';
import 'package:engine_tool/src/worker_pool.dart';
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

  (Environment, List<List<String>>) macEnv(Logger logger) {
    final List<List<String>> runHistory = <List<String>>[];
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
            onStart: (FakeCommandLogEntry entry) {
              runHistory.add(entry.command);
              switch (entry.command) {
                case ['success']:
                  return FakeProcess(stdout: 'stdout success');
                case ['failure']:
                  return FakeProcess(exitCode: 1, stdout: 'stdout failure');
                default:
                  return FakeProcess();
              }
            },
            onRun: (FakeCommandLogEntry entry) {
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

  test('process queue success', () async {
    final Logger logger = Logger.test((_) {});
    final (Environment env, _) = macEnv(logger);
    final WorkerPool wp = WorkerPool(env, NoopWorkerPoolProgressReporter());
    final ProcessTask task = ProcessTask('S', env, io.Directory.current, <String>['success']);
    final bool r = await wp.run(<WorkerTask>{task});
    expect(r, equals(true));
    expect(task.processArtifacts.exitCode, equals(0));
    final ProcessArtifacts loaded = ProcessArtifacts.fromFile(io.File(task.processArtifactsPath));
    expect(loaded.stdout, equals('stdout success'));
  });

  test('process queue failure', () async {
    final Logger logger = Logger.test((_) {});
    final (Environment env, _) = macEnv(logger);
    final WorkerPool wp = WorkerPool(env, NoopWorkerPoolProgressReporter());
    final ProcessTask task = ProcessTask('F', env, io.Directory.current, <String>['failure']);
    final bool r = await wp.run(<WorkerTask>{task});
    expect(r, equals(false));
    expect(task.processArtifacts.exitCode, isNot(0));
    final ProcessArtifacts loaded = ProcessArtifacts.fromFile(io.File(task.processArtifactsPath));
    expect(loaded.stdout, equals('stdout failure'));
  });
}
