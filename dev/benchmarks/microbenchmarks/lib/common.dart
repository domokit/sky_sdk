// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert' show json;
import 'dart:math' as math;

double _doNormal({required double mean, required double stddev, required double x}) {
  return (1.0 / (stddev * math.sqrt(2.0 * math.pi))) *
      math.pow(math.e, -0.5 * math.pow((x - mean) / stddev, 2.0));
}

double _doMean(List<double> values) => values.reduce((double x, double y) => x + y) / values.length;

double _doStddev(List<double> values, double mean) {
  double stddev = 0.0;
  for (final double value in values) {
    stddev += (value - mean) * (value - mean);
  }
  return math.sqrt(stddev / values.length);
}

double _doIntegral({
  required double Function(double) func,
  required double start,
  required double stop,
  required double resolution,
}) {
  double result = 0.0;
  while (start < stop) {
    final double value = func(start);
    result += resolution * value;
    start += resolution;
  }
  return result;
}

/// Probability is defined as the probability that the mean is within the
/// [margin] of the true value.
double _doProbability({required double mean, required double stddev, required double margin}) {
  return _doIntegral(
    func: (double x) => _doNormal(mean: mean, stddev: stddev, x: x),
    start: (1.0 - margin) * mean,
    stop: (1.0 + margin) * mean,
    resolution: 0.001,
  );
}

/// This class knows how to format benchmark results for machine and human
/// consumption.
///

/// Example:
///
///     BenchmarkResultPrinter printer = BenchmarkResultPrinter();
///     printer.add(
///       description: 'Average frame time',
///       value: averageFrameTime,
///       unit: 'ms',
///       name: 'average_frame_time',
///     );
///     printer.printToStdout();
///
class BenchmarkResultPrinter {
  final List<_BenchmarkResult> _results = <_BenchmarkResult>[];

  /// Adds a benchmark result to the list of results.
  ///
  /// [description] is a human-readable description of the result. [value] is a
  /// result value. [unit] is the unit of measurement, such as "ms", "km", "h".
  /// [name] is a computer-readable name of the result used as a key in the JSON
  /// serialization of the results.
  void addResult({
    required String description,
    required double value,
    required String unit,
    required String name,
  }) {
    _results.add(_BenchmarkResult(description, value, unit, name));
  }

  /// Adds a benchmark result to the list of results and a probability of that
  /// result.
  ///
  /// The probability is calculated as the probability that the mean is +- 5% of
  /// the true value.
  ///
  /// See also [addResult].
  void addResultStatistics({
    required String description,
    required List<double> values,
    required String unit,
    required String name,
  }) {
    final double mean = _doMean(values);
    final double stddev = _doStddev(values, mean);
    const double margin = 0.05;
    final double probability = _doProbability(mean: mean, stddev: stddev, margin: margin);
    _results.add(_BenchmarkResult(description, mean, unit, name));
    _results.add(
      _BenchmarkResult(
        '$description - probability margin of error $margin',
        probability,
        'percent',
        '${name}_probability_5pct',
      ),
    );
  }

  /// Prints the results added via [addResult] to standard output, once as JSON
  /// for computer consumption and once formatted as plain text for humans.
  void printToStdout() {
    // IMPORTANT: keep these values in sync with dev/devicelab/bin/tasks/microbenchmarks.dart
    const String jsonStart = '================ RESULTS ================';
    const String jsonEnd = '================ FORMATTED ==============';
    const String jsonPrefix = ':::JSON:::';

    print(jsonStart);
    print('$jsonPrefix ${_printJson()}');
    print(jsonEnd);
    print(_printPlainText());
  }

  String _printJson() {
    final Map<String, double> results = <String, double>{};
    for (final _BenchmarkResult result in _results) {
      results[result.name] = result.value;
    }
    return json.encode(results);
  }

  String _printPlainText() {
    final StringBuffer buf = StringBuffer();
    for (final _BenchmarkResult result in _results) {
      buf.writeln('${result.description}: ${result.value.toStringAsFixed(1)} ${result.unit}');
    }
    return buf.toString();
  }
}

class _BenchmarkResult {
  _BenchmarkResult(this.description, this.value, this.unit, this.name);

  /// Human-readable description of the result, e.g. "Average frame time".
  final String description;

  /// Result value that in agreement with [unit].
  final double value;

  /// Unit of measurement that is in agreement with [value].
  final String unit;

  /// Computer-readable name of the result.
  final String name;
}
