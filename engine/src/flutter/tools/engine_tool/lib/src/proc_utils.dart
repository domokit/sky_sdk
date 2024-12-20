// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:io';
import 'dart:math';

import 'package:path/path.dart' as p;
import 'package:process_runner/process_runner.dart';

import 'environment.dart';
import 'logger.dart';
import 'typed_json.dart';
import 'worker_pool.dart';

/// Artifacts from an exited sub-process.
final class ProcessArtifacts {
  /// Constructs an instance of ProcessArtifacts from raw values.
  ProcessArtifacts(this.cwd, this.commandLine, this.exitCode, this.stdout, this.stderr, {this.pid});

  /// Constructs an instance of ProcessArtifacts from a ProcessRunnerResult
  /// and the spawning context.
  factory ProcessArtifacts.fromResult(
    Directory cwd,
    List<String> commandLine,
    ProcessRunnerResult result,
  ) {
    return ProcessArtifacts(
      cwd,
      commandLine,
      result.exitCode,
      result.stdout,
      result.stderr,
      pid: result.pid,
    );
  }

  /// Constructs an instance of ProcessArtifacts from serialized JSON text.
  factory ProcessArtifacts.fromJson(String serialized) {
    final JsonObject artifact = JsonObject.parse(serialized);
    return artifact.map(
      (JsonObject json) => ProcessArtifacts(
        Directory(json.string('cwd')),
        json.stringList('commandLine'),
        json.integer('exitCode'),
        json.string('stdout'),
        json.string('stderr'),
        pid: json.integer('pid'),
      ),
      onError: (JsonObject source, JsonMapException e) {
        throw FormatException('Failed to parse ProcessArtifacts: $e', source.toPrettyString());
      },
    );
  }

  /// Constructs an instance of ProcessArtifacts from a file containing JSON.
  factory ProcessArtifacts.fromFile(File file) {
    return ProcessArtifacts.fromJson(file.readAsStringSync());
  }

  /// Saves ProcessArtifacts into file.
  void save(File file) {
    file.writeAsStringSync(
      JsonObject(<String, Object?>{
        if (pid != null) 'pid': pid,
        'exitCode': exitCode,
        'stdout': stdout,
        'stderr': stderr,
        'cwd': cwd.absolute.path,
        'commandLine': commandLine,
      }).toPrettyString(),
    );
  }

  /// Creates a temporary file and saves the artifacts into it.
  /// Returns the File.
  File saveTemp() {
    final Directory systemTemp = Directory.systemTemp;
    final String prefix = pid != null ? 'et$pid' : 'et';
    final Directory artifacts = systemTemp.createTempSync(prefix);
    final File resultFile = File(p.join(artifacts.path, 'process_artifacts.json'));
    save(resultFile);
    return resultFile;
  }

  /// Current working directory of process when it was spawned.
  final Directory cwd;

  /// Full command line of process.
  final List<String> commandLine;

  /// Exit code.
  final int exitCode;

  /// Stdout (may be empty).
  final String stdout;

  /// Stdout (may be empty).
  final String stderr;

  /// Pid (when available).
  final int? pid;
}

/// A WorkerTask that runs a process
class ProcessTask extends WorkerTask {
  /// Construct a new process task with name, cwd, and command line.
  ProcessTask(super.name, this._environment, this._cwd, this._commandLine);

  final Environment _environment;
  final Directory _cwd;
  final List<String> _commandLine;
  late ProcessArtifacts? _processArtifacts;
  late String? _processArtifactsPath;

  @override
  Future<bool> run() async {
    final ProcessRunnerResult result = await _environment.processRunner.runProcess(
      _commandLine,
      failOk: true,
      workingDirectory: _cwd,
    );
    _processArtifacts = ProcessArtifacts(
      _cwd,
      _commandLine,
      result.exitCode,
      result.stdout,
      result.stderr,
      pid: result.pid,
    );
    _processArtifactsPath = _processArtifacts!.saveTemp().path;
    return result.exitCode == 0;
  }

  /// Returns the ProcessArtifacts after run completes.
  ProcessArtifacts get processArtifacts {
    return _processArtifacts!;
  }

  /// Returns the path that the process artifacts were saved in.
  String get processArtifactsPath {
    return _processArtifactsPath!;
  }
}

/// A WorkerPoolProgressReporter designed to work with ProcessTasks.
class ProcessTaskProgressReporter implements WorkerPoolProgressReporter {
  /// Construct a new reporter.
  ProcessTaskProgressReporter(this._environment);

  final Environment _environment;
  Spinner? _spinner;
  bool _finished = false;
  int _longestName = 0;
  int _doneCount = 0;
  int _totalCount = 0;

  @override
  void onRun(Set<WorkerTask> tasks) {
    _totalCount = tasks.length;
    for (final WorkerTask task in tasks) {
      assert(task is ProcessTask);
      _longestName = max(_longestName, task.name.length);
    }
  }

  @override
  void onFinish() {
    _finished = true;
    _updateSpinner(<ProcessTask>{});
  }

  @override
  void onTaskStart(WorkerPool pool, WorkerTask task) {
    _updateSpinner(pool.running);
  }

  @override
  void onTaskDone(WorkerPool pool, WorkerTask task, [Object? err]) {
    _doneCount++;
    task as ProcessTask;
    final ProcessArtifacts pa = task.processArtifacts;
    final String dt = _formatDurationShort(task.runTime);
    if (pa.exitCode != 0) {
      final String paPath = task.processArtifactsPath;
      _environment.logger.clearLine();
      _environment.logger.status('FAIL: $dt ${task.name} [details in $paPath]');
    } else {
      _environment.logger.clearLine();
      _environment.logger.status('OKAY: $dt ${task.name}');
    }
    _updateSpinner(pool.running);
  }

  void _updateSpinner(Set<WorkerTask> tasks) {
    if (_spinner != null) {
      _spinner!.finish();
      _spinner = null;
    }
    if (_finished) {
      return;
    }
    _environment.logger.clearLine();
    final String taskName = tasks.isEmpty ? '' : tasks.first.name;
    final String etc = tasks.length > 1 ? '... [${tasks.length}]' : '';
    _environment.logger.status('Running $_doneCount/$_totalCount $taskName$etc ', newline: false);
    _spinner = _environment.logger.startSpinner();
  }

  String _formatDurationShort(Duration dur) {
    int micros = dur.inMicroseconds;
    String r = '';
    if (micros >= Duration.microsecondsPerMinute) {
      final int minutes = micros ~/ Duration.microsecondsPerMinute;
      micros -= minutes * Duration.microsecondsPerMinute;
      r += '${minutes}m';
    }
    if (micros >= Duration.microsecondsPerSecond) {
      final int seconds = micros ~/ Duration.microsecondsPerSecond;
      micros -= seconds * Duration.microsecondsPerSecond;
      if (r.isNotEmpty) {
        r += '.';
      }
      r += '${seconds}s';
    }
    if (micros >= Duration.microsecondsPerMillisecond) {
      final int millis = micros ~/ Duration.microsecondsPerMillisecond;
      micros -= millis * Duration.microsecondsPerMillisecond;
      if (r.isNotEmpty) {
        r += '.';
      }
      r += '${millis}ms';
    }
    return r.padLeft(15);
  }
}

/// If result.exitCode != 0, will call logger.fatal with relevant information
/// and terminate the program.
void fatalIfFailed(Environment environment, List<String> commandLine, ProcessRunnerResult result) {
  if (result.exitCode == 0) {
    return;
  }
  environment.logger.fatal(
    'Process "${commandLine.join(' ')}" failed exitCode=${result.exitCode}\n'
    'STDOUT:\n${result.stdout}'
    'STDERR:\n${result.stderr}',
  );
}

/// Ensures that pathToBinary includes a '.exe' suffix on relevant platforms.
String exePath(Environment environment, String pathToBinary) {
  String suffix = '';
  if (environment.platform.isWindows) {
    suffix = '.exe';
  }
  return '$pathToBinary$suffix';
}

/// Returns the path to the gn binary.
String gnBinPath(Environment environment) {
  return exePath(
    environment,
    p.join(environment.engine.srcDir.path, 'flutter', 'third_party', 'gn', 'gn'),
  );
}
