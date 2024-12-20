// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:math' as math;

import 'task_result.dart';

const String kBenchmarkTypeKeyName = 'benchmark_type';
const String kBenchmarkVersionKeyName = 'version';
const String kLocalEngineKeyName = 'local_engine';
const String kLocalEngineHostKeyName = 'local_engine_host';
const String kTaskNameKeyName = 'task_name';
const String kRunStartKeyName = 'run_start';
const String kRunEndKeyName = 'run_end';
const String kAResultsKeyName = 'default_results';
const String kBResultsKeyName = 'local_engine_results';

const String kBenchmarkResultsType = 'A/B summaries';
const String kBenchmarkABVersion = '1.0';

enum FieldJustification { LEFT, RIGHT, CENTER }

/// Collects data from an A/B test and produces a summary for human evaluation.
///
/// See [printSummary] for more.
class ABTest {
  ABTest({required this.localEngine, required this.localEngineHost, required this.taskName})
    : runStart = DateTime.now(),
      _aResults = <String, List<double>>{},
      _bResults = <String, List<double>>{};

  ABTest.fromJsonMap(Map<String, dynamic> jsonResults)
    : localEngine = jsonResults[kLocalEngineKeyName] as String,
      localEngineHost = jsonResults[kLocalEngineHostKeyName] as String?,
      taskName = jsonResults[kTaskNameKeyName] as String,
      runStart = DateTime.parse(jsonResults[kRunStartKeyName] as String),
      _runEnd = DateTime.parse(jsonResults[kRunEndKeyName] as String),
      _aResults = _convertFrom(jsonResults[kAResultsKeyName] as Map<String, dynamic>),
      _bResults = _convertFrom(jsonResults[kBResultsKeyName] as Map<String, dynamic>);

  final String localEngine;
  final String? localEngineHost;
  final String taskName;
  final DateTime runStart;
  DateTime? _runEnd;
  DateTime? get runEnd => _runEnd;

  final Map<String, List<double>> _aResults;
  final Map<String, List<double>> _bResults;

  static Map<String, List<double>> _convertFrom(dynamic results) {
    final Map<String, dynamic> resultMap = results as Map<String, dynamic>;
    return <String, List<double>>{
      for (final String key in resultMap.keys)
        key: (resultMap[key] as List<dynamic>).cast<double>(),
    };
  }

  /// Adds the result of a single A run of the benchmark.
  ///
  /// The result may contain multiple score keys.
  ///
  /// [result] is expected to be a serialization of [TaskResult].
  void addAResult(TaskResult result) {
    if (_runEnd != null) {
      throw StateError('Cannot add results to ABTest after it is finalized');
    }
    _addResult(result, _aResults);
  }

  /// Adds the result of a single B run of the benchmark.
  ///
  /// The result may contain multiple score keys.
  ///
  /// [result] is expected to be a serialization of [TaskResult].
  void addBResult(TaskResult result) {
    if (_runEnd != null) {
      throw StateError('Cannot add results to ABTest after it is finalized');
    }
    _addResult(result, _bResults);
  }

  void finalize() {
    _runEnd = DateTime.now();
  }

  Map<String, dynamic> get jsonMap => <String, dynamic>{
    kBenchmarkTypeKeyName: kBenchmarkResultsType,
    kBenchmarkVersionKeyName: kBenchmarkABVersion,
    kLocalEngineKeyName: localEngine,
    if (localEngineHost != null) kLocalEngineHostKeyName: localEngineHost,
    kTaskNameKeyName: taskName,
    kRunStartKeyName: runStart.toIso8601String(),
    kRunEndKeyName: runEnd!.toIso8601String(),
    kAResultsKeyName: _aResults,
    kBResultsKeyName: _bResults,
  };

  static void updateColumnLengths(List<int> lengths, List<String?> results) {
    for (int column = 0; column < lengths.length; column++) {
      if (results[column] != null) {
        lengths[column] = math.max(lengths[column], results[column]?.length ?? 0);
      }
    }
  }

  static void formatResult(
    StringBuffer buffer,
    List<int> lengths,
    List<FieldJustification> aligns,
    List<String?> values,
  ) {
    for (int column = 0; column < lengths.length; column++) {
      final int len = lengths[column];
      String? value = values[column];
      if (value == null) {
        value = ''.padRight(len);
      } else {
        value = switch (aligns[column]) {
          FieldJustification.LEFT => value.padRight(len),
          FieldJustification.RIGHT => value.padLeft(len),
          FieldJustification.CENTER => value.padLeft((len + value.length) ~/ 2).padRight(len),
        };
      }
      if (column > 0) {
        value = value.padLeft(len + 1);
      }
      buffer.write(value);
    }
    buffer.writeln();
  }

  /// Returns the summary as a tab-separated spreadsheet.
  ///
  /// This value can be copied straight to a Google Spreadsheet for further analysis.
  String asciiSummary() {
    final Map<String, _ScoreSummary> summariesA = _summarize(_aResults);
    final Map<String, _ScoreSummary> summariesB = _summarize(_bResults);

    final List<List<String?>> tableRows = <List<String?>>[
      for (final String scoreKey in <String>{...summariesA.keys, ...summariesB.keys})
        <String?>[
          scoreKey,
          summariesA[scoreKey]?.averageString,
          summariesA[scoreKey]?.noiseString,
          summariesB[scoreKey]?.averageString,
          summariesB[scoreKey]?.noiseString,
          summariesA[scoreKey]?.improvementOver(summariesB[scoreKey]),
        ],
    ];

    final List<String> titles = <String>[
      'Score',
      'Average A',
      '(noise)',
      'Average B',
      '(noise)',
      'Speed-up',
    ];
    final List<FieldJustification> alignments = <FieldJustification>[
      FieldJustification.LEFT,
      FieldJustification.RIGHT,
      FieldJustification.LEFT,
      FieldJustification.RIGHT,
      FieldJustification.LEFT,
      FieldJustification.CENTER,
    ];

    final List<int> lengths = List<int>.filled(6, 0);
    updateColumnLengths(lengths, titles);
    for (final List<String?> row in tableRows) {
      updateColumnLengths(lengths, row);
    }

    final StringBuffer buffer = StringBuffer();
    formatResult(buffer, lengths, <FieldJustification>[
      FieldJustification.CENTER,
      ...alignments.skip(1),
    ], titles);
    for (final List<String?> row in tableRows) {
      formatResult(buffer, lengths, alignments, row);
    }

    return buffer.toString();
  }

  /// Returns unprocessed data collected by the A/B test formatted as
  /// a tab-separated spreadsheet.
  String rawResults() {
    final StringBuffer buffer = StringBuffer();
    for (final String scoreKey in _allScoreKeys) {
      buffer.writeln('$scoreKey:');
      buffer.write('  A:\t');
      if (_aResults.containsKey(scoreKey)) {
        for (final double score in _aResults[scoreKey]!) {
          buffer.write('${score.toStringAsFixed(2)}\t');
        }
      } else {
        buffer.write('N/A');
      }
      buffer.writeln();

      buffer.write('  B:\t');
      if (_bResults.containsKey(scoreKey)) {
        for (final double score in _bResults[scoreKey]!) {
          buffer.write('${score.toStringAsFixed(2)}\t');
        }
      } else {
        buffer.write('N/A');
      }
      buffer.writeln();
    }
    return buffer.toString();
  }

  Set<String> get _allScoreKeys => <String>{..._aResults.keys, ..._bResults.keys};

  /// Returns the summary as a tab-separated spreadsheet.
  ///
  /// This value can be copied straight to a Google Spreadsheet for further analysis.
  String printSummary() {
    final Map<String, _ScoreSummary> summariesA = _summarize(_aResults);
    final Map<String, _ScoreSummary> summariesB = _summarize(_bResults);

    final StringBuffer buffer = StringBuffer(
      'Score\tAverage A (noise)\tAverage B (noise)\tSpeed-up\n',
    );

    for (final String scoreKey in _allScoreKeys) {
      final _ScoreSummary? summaryA = summariesA[scoreKey];
      final _ScoreSummary? summaryB = summariesB[scoreKey];
      buffer.write('$scoreKey\t');

      if (summaryA != null) {
        buffer.write('${summaryA.averageString} ${summaryA.noiseString}\t');
      } else {
        buffer.write('\t');
      }

      if (summaryB != null) {
        buffer.write('${summaryB.averageString} ${summaryB.noiseString}\t');
      } else {
        buffer.write('\t');
      }

      if (summaryA != null && summaryB != null) {
        buffer.write('${summaryA.improvementOver(summaryB)}\t');
      }

      buffer.writeln();
    }

    return buffer.toString();
  }
}

class _ScoreSummary {
  _ScoreSummary({required this.average, required this.noise});

  /// Average (arithmetic mean) of a series of values collected by a benchmark.
  final double average;

  /// The noise (standard deviation divided by [average]) in the collected
  /// values.
  final double noise;

  String get averageString => average.toStringAsFixed(2);
  String get noiseString => '(${_ratioToPercent(noise)})';

  String improvementOver(_ScoreSummary? other) {
    return other == null ? '' : '${(average / other.average).toStringAsFixed(2)}x';
  }
}

void _addResult(TaskResult result, Map<String, List<double>> results) {
  for (final String scoreKey in result.benchmarkScoreKeys ?? <String>[]) {
    final double score = (result.data![scoreKey] as num).toDouble();
    results.putIfAbsent(scoreKey, () => <double>[]).add(score);
  }
}

Map<String, _ScoreSummary> _summarize(Map<String, List<double>> results) {
  return results.map<String, _ScoreSummary>((String scoreKey, List<double> values) {
    final double average = _computeAverage(values);
    return MapEntry<String, _ScoreSummary>(
      scoreKey,
      _ScoreSummary(
        average: average,
        // If the average is zero, the benchmark got the perfect score with no noise.
        noise: average > 0 ? _computeStandardDeviationForPopulation(values) / average : 0.0,
      ),
    );
  });
}

/// Computes the arithmetic mean (or average) of given [values].
double _computeAverage(Iterable<double> values) {
  final double sum = values.reduce((double a, double b) => a + b);
  return sum / values.length;
}

/// Computes population standard deviation.
///
/// Unlike sample standard deviation, which divides by N - 1, this divides by N.
///
/// See also:
///
/// * https://en.wikipedia.org/wiki/Standard_deviation
double _computeStandardDeviationForPopulation(Iterable<double> population) {
  final double mean = _computeAverage(population);
  final double sumOfSquaredDeltas = population.fold<double>(
    0.0,
    (double previous, num value) => previous += math.pow(value - mean, 2),
  );
  return math.sqrt(sumOfSquaredDeltas / population.length);
}

String _ratioToPercent(double value) {
  return '${(value * 100).toStringAsFixed(2)}%';
}
