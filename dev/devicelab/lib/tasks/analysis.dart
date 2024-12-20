// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io';

import 'package:path/path.dart' as path;

import '../framework/task_result.dart';
import '../framework/utils.dart';

/// Run each benchmark this many times and compute average, min, max.
///
/// This must be small enough that we can do all the work in 15 minutes, the
/// devicelab deadline. Since there's four different analysis tasks, on average,
/// each can have 4 minutes. The tasks currently average a little more than a
/// minute, so that allows three runs per task.
const int _kRunsPerBenchmark = 3;

/// Path to the generated "mega gallery" app.
Directory get _megaGalleryDirectory => dir(path.join(Directory.systemTemp.path, 'mega_gallery'));

Future<TaskResult> analyzerBenchmarkTask() async {
  await inDirectory<void>(flutterDirectory, () async {
    rmTree(_megaGalleryDirectory);
    mkdirs(_megaGalleryDirectory);
    await flutter('update-packages');
    await dart(<String>['dev/tools/mega_gallery.dart', '--out=${_megaGalleryDirectory.path}']);
  });

  final Map<String, dynamic> data = <String, dynamic>{
    ...(await _run(_FlutterRepoBenchmark())).asMap('flutter_repo', 'batch'),
    ...(await _run(_FlutterRepoBenchmark(watch: true))).asMap('flutter_repo', 'watch'),
    ...(await _run(_MegaGalleryBenchmark())).asMap('mega_gallery', 'batch'),
    ...(await _run(_MegaGalleryBenchmark(watch: true))).asMap('mega_gallery', 'watch'),
  };

  return TaskResult.success(data, benchmarkScoreKeys: data.keys.toList());
}

class _BenchmarkResult {
  const _BenchmarkResult(this.mean, this.min, this.max);

  final double mean; // seconds

  final double min; // seconds

  final double max; // seconds

  Map<String, dynamic> asMap(String benchmark, String mode) {
    return <String, dynamic>{
      '${benchmark}_$mode': mean,
      '${benchmark}_${mode}_minimum': min,
      '${benchmark}_${mode}_maximum': max,
    };
  }
}

abstract class _Benchmark {
  _Benchmark({this.watch = false});

  final bool watch;

  String get title;

  Directory get directory;

  List<String> get options => <String>['--benchmark', if (watch) '--watch'];

  Future<double> execute(int iteration, int targetIterations) async {
    section('Analyze $title ${watch ? 'with watcher' : ''} - ${iteration + 1} / $targetIterations');
    final Stopwatch stopwatch = Stopwatch();
    await inDirectory<void>(directory, () async {
      stopwatch.start();
      await flutter('analyze', options: options);
      stopwatch.stop();
    });
    return stopwatch.elapsedMicroseconds / (1000.0 * 1000.0);
  }
}

/// Times how long it takes to analyze the Flutter repository.
class _FlutterRepoBenchmark extends _Benchmark {
  _FlutterRepoBenchmark({super.watch});

  @override
  String get title => 'Flutter repo';

  @override
  Directory get directory => flutterDirectory;

  @override
  List<String> get options {
    return super.options..add('--flutter-repo');
  }
}

/// Times how long it takes to analyze the generated "mega_gallery" app.
class _MegaGalleryBenchmark extends _Benchmark {
  _MegaGalleryBenchmark({super.watch});

  @override
  String get title => 'mega gallery';

  @override
  Directory get directory => _megaGalleryDirectory;
}

/// Runs `benchmark` several times and reports the results.
Future<_BenchmarkResult> _run(_Benchmark benchmark) async {
  final List<double> results = <double>[];
  for (int i = 0; i < _kRunsPerBenchmark; i += 1) {
    // Delete cached analysis results.
    rmTree(dir('${Platform.environment['HOME']}/.dartServer'));
    results.add(await benchmark.execute(i, _kRunsPerBenchmark));
  }
  results.sort();
  final double sum = results.fold<double>(
    0.0,
    (double previousValue, double element) => previousValue + element,
  );
  return _BenchmarkResult(sum / results.length, results.first, results.last);
}
