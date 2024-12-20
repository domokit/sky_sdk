// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ffi' as ffi show Abi;
import 'dart:io' as io;

import 'package:engine_repo_tools/engine_repo_tools.dart';
import 'package:engine_tool/src/environment.dart';
import 'package:engine_tool/src/logger.dart';
import 'package:engine_tool/src/worker_pool.dart';
import 'package:platform/platform.dart';
import 'package:process_fakes/process_fakes.dart';
import 'package:process_runner/process_runner.dart';
import 'package:test/test.dart';

class TestWorkerPoolProgressReporter implements WorkerPoolProgressReporter {
  int _successCount = 0;
  int _errorCount = 0;
  final List<Object> _errors = <Object>[];

  @override
  void onRun(Set<WorkerTask> tasks) {}

  @override
  void onFinish() {}

  @override
  void onTaskStart(WorkerPool pool, WorkerTask task) {}

  @override
  void onTaskDone(WorkerPool pool, WorkerTask task, [Object? err]) {
    if (err == null) {
      _successCount++;
      return;
    }
    _errorCount++;
    _errors.add(err);
  }
}

class SuccessfulTask extends WorkerTask {
  SuccessfulTask() : super('s');

  @override
  Future<bool> run() async {
    return true;
  }
}

class FailureTask extends WorkerTask {
  FailureTask() : super('f');

  @override
  Future<bool> run() async {
    throw ArgumentError('test');
  }
}

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
        platform: FakePlatform(operatingSystem: Platform.macOS, pathSeparator: '/'),
        processRunner: ProcessRunner(
          processManager: FakeProcessManager(
            onStart: (List<String> command) {
              runHistory.add(command);
              switch (command) {
                case ['success']:
                  return FakeProcess(stdout: 'stdout success');
                case ['failure']:
                  return FakeProcess(exitCode: 1, stdout: 'stdout failure');
                default:
                  return FakeProcess();
              }
            },
            onRun: (List<String> command) {
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

  test('worker pool success', () async {
    final Logger logger = Logger.test((_) {});
    final (Environment env, _) = macEnv(logger);
    final TestWorkerPoolProgressReporter reporter = TestWorkerPoolProgressReporter();
    final WorkerPool wp = WorkerPool(env, reporter);
    final WorkerTask task = SuccessfulTask();
    final bool r = await wp.run(<WorkerTask>{task});
    expect(r, equals(true));
    expect(reporter._successCount, equals(1));
    expect(reporter._errorCount, equals(0));
    expect(reporter._errors.length, equals(0));
  });

  test('worker pool failure', () async {
    final Logger logger = Logger.test((_) {});
    final (Environment env, _) = macEnv(logger);
    final TestWorkerPoolProgressReporter reporter = TestWorkerPoolProgressReporter();
    final WorkerPool wp = WorkerPool(env, reporter);
    final WorkerTask task = FailureTask();
    final bool r = await wp.run(<WorkerTask>{task});
    expect(r, equals(false));
    expect(reporter._successCount, equals(0));
    expect(reporter._errorCount, equals(1));
    expect(reporter._errors.length, equals(1));
  });
}
