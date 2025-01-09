// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io';

import 'package:path/path.dart' as path;

import '../framework/task_result.dart';
import '../framework/utils.dart';

/// Run each benchmark this many times and compute average, min, max.
const int _kRunsPerBenchmark = 10;

Future<TaskResult> flutterToolStartupBenchmarkTask() async {
  final Directory projectParentDirectory = Directory.systemTemp.createTempSync(
    'flutter_tool_startup_benchmark',
  );
  final Directory projectDirectory = dir(path.join(projectParentDirectory.path, 'benchmark'));
  await inDirectory<void>(flutterDirectory, () async {
    await flutter('update-packages');
    await flutter('create', options: <String>[projectDirectory.path]);
    // Remove 'test' directory so we don't time the actual testing, but only the launching of the flutter tool
    rmTree(dir(path.join(projectDirectory.path, 'test')));
  });

  final Map<String, dynamic> data = <String, dynamic>{
    // `flutter test` in dir with no `test` folder.
    ...(await _Benchmark(projectDirectory, 'test startup', 'test').run()).asMap(
      'flutter_tool_startup_test',
    ),

    // `flutter test -d foo_device` in dir with no `test` folder.
    ...(await _Benchmark(
          projectDirectory,
          'test startup with specified device',
          'test',
          options: <String>['-d', 'foo_device'],
        ).run())
        .asMap('flutter_tool_startup_test_with_specified_device'),

    // `flutter test -v` where no android sdk will be found (at least currently).
    ...(await _Benchmark(
          projectDirectory,
          'test startup no android sdk',
          'test',
          options: <String>['-v'],
          environment: <String, String>{
            'ANDROID_HOME': 'dummy value',
            'ANDROID_SDK_ROOT': 'dummy value',
            'PATH': pathWithoutWhereHits(<String>['adb', 'aapt']),
          },
        ).run())
        .asMap('flutter_tool_startup_test_no_android_sdk'),

    // `flutter -h`.
    ...(await _Benchmark(projectDirectory, 'help startup', '-h').run()).asMap(
      'flutter_tool_startup_help',
    ),
  };

  // Cleanup.
  rmTree(projectParentDirectory);

  return TaskResult.success(data, benchmarkScoreKeys: data.keys.toList());
}

String pathWithoutWhereHits(List<String> whats) {
  final String pathEnvironment = Platform.environment['PATH'] ?? '';
  List<String> paths;
  if (Platform.isWindows) {
    paths = pathEnvironment.split(';');
  } else {
    paths = pathEnvironment.split(':');
  }
  // This isn't great but will probably work for our purposes.
  final List<String> extensions = <String>['', '.exe', '.bat', '.com'];

  final List<String> notFound = <String>[];
  for (final String path in paths) {
    bool found = false;
    for (final String extension in extensions) {
      for (final String what in whats) {
        final File f = File('$path${Platform.pathSeparator}$what$extension');
        if (f.existsSync()) {
          found = true;
          break;
        }
      }
      if (found) {
        break;
      }
    }
    if (!found) {
      notFound.add(path);
    }
  }

  if (Platform.isWindows) {
    return notFound.join(';');
  } else {
    return notFound.join(':');
  }
}

class _BenchmarkResult {
  const _BenchmarkResult(this.mean, this.min, this.max);

  final int mean; // Milliseconds

  final int min; // Milliseconds

  final int max; // Milliseconds

  Map<String, dynamic> asMap(String name) {
    return <String, dynamic>{name: mean, '${name}_minimum': min, '${name}_maximum': max};
  }
}

class _Benchmark {
  _Benchmark(
    this.directory,
    this.title,
    this.command, {
    this.options = const <String>[],
    this.environment,
  });

  final Directory directory;

  final String title;

  final String command;

  final List<String> options;

  final Map<String, String>? environment;

  Future<int> execute(int iteration, int targetIterations) async {
    section('Benchmark $title - ${iteration + 1} / $targetIterations');
    final Stopwatch stopwatch = Stopwatch();
    await inDirectory<void>(directory, () async {
      stopwatch.start();
      // canFail is set to true, as e.g. `flutter test` in a dir with no `test`
      // directory sets a non-zero return value.
      await flutter(command, options: options, canFail: true, environment: environment);
      stopwatch.stop();
    });
    return stopwatch.elapsedMilliseconds;
  }

  /// Runs `benchmark` several times and reports the results.
  Future<_BenchmarkResult> run() async {
    final List<int> results = <int>[];
    int sum = 0;
    for (int i = 0; i < _kRunsPerBenchmark; i++) {
      final int thisRuntime = await execute(i, _kRunsPerBenchmark);
      results.add(thisRuntime);
      sum += thisRuntime;
    }
    results.sort();
    return _BenchmarkResult(sum ~/ results.length, results.first, results.last);
  }
}
