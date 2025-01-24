// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:ui/ui_web/src/ui_web.dart' as ui_web;

ui_web.BenchmarkValueCallback? engineBenchmarkValueCallback;

/// A function that computes a value of type [R].
///
/// Functions of this signature can be passed to [timeAction] for performance
/// profiling.
typedef Action<R> = R Function();

/// Uses the [Profiler] to time a synchronous [action] function and reports the
/// result under the give metric [name].
///
/// If profiling is disabled, simply calls [action] and returns the result.
///
/// Use this for situations when the cost of an extra closure is negligible.
/// This function reduces the boilerplate associated with checking if profiling
/// is enabled and exercising the stopwatch.
///
/// Example:
///
/// ```dart
/// final result = timeAction('expensive_operation', () {
///   // ... expensive work ...
///   return someValue;
/// });
/// ```
R timeAction<R>(String name, Action<R> action) {
  if (!Profiler.isBenchmarkMode) {
    return action();
  } else {
    final Stopwatch stopwatch = Stopwatch()..start();
    final R result = action();
    stopwatch.stop();
    Profiler.instance.benchmark(name, stopwatch.elapsedMicroseconds.toDouble());
    return result;
  }
}

/// The purpose of this class is to facilitate communication of
/// profiling/benchmark data to the outside world (e.g. a macrobenchmark that's
/// running a flutter app).
///
/// To use the [Profiler]:
///
/// 1. Set the environment variable `FLUTTER_WEB_ENABLE_PROFILING` to true.
///
/// 2. Set the [engineBenchmarkValueCallback] to a function that will receive
///   the benchmark data.
///
/// The listener function will be called every time a new benchmark number is
/// calculated. The signature is `Function(String name, num value)`.
class Profiler {
  Profiler._() {
    _checkBenchmarkMode();
  }

  static bool isBenchmarkMode = const bool.fromEnvironment('FLUTTER_WEB_ENABLE_PROFILING');

  static Profiler ensureInitialized() {
    _checkBenchmarkMode();
    return Profiler._instance ??= Profiler._();
  }

  static Profiler get instance {
    _checkBenchmarkMode();
    final Profiler? profiler = _instance;
    if (profiler == null) {
      throw Exception(
        'Profiler has not been properly initialized. '
        'Make sure Profiler.ensureInitialized() is being called before you '
        'access Profiler.instance',
      );
    }
    return profiler;
  }

  static Profiler? _instance;

  static void _checkBenchmarkMode() {
    if (!isBenchmarkMode) {
      throw Exception(
        'Cannot use Profiler unless benchmark mode is enabled. '
        'You can enable it by setting the `FLUTTER_WEB_ENABLE_PROFILING` '
        'environment variable to true.',
      );
    }
  }

  /// Used to send benchmark data to whoever is listening to them.
  void benchmark(String name, double value) {
    _checkBenchmarkMode();

    if (engineBenchmarkValueCallback != null) {
      engineBenchmarkValueCallback!(name, value);
    }
  }
}

/// Counts various events that take place while the app is running.
///
/// This class will slow down the app, and therefore should be disabled while
/// benchmarking. For example, avoid using it in conjunction with [Profiler].
class Instrumentation {
  Instrumentation._() {
    _checkInstrumentationEnabled();
  }

  /// Whether instrumentation is enabled.
  ///
  /// Check this value before calling any other methods in this class.
  static bool get enabled => _enabled;
  static set enabled(bool value) {
    if (_enabled == value) {
      return;
    }

    if (!value) {
      _instance._counters.clear();
      _instance._printTimer = null;
    }

    _enabled = value;
  }

  static bool _enabled = const bool.fromEnvironment('FLUTTER_WEB_ENABLE_INSTRUMENTATION');

  /// Returns the singleton that provides instrumentation API.
  static Instrumentation get instance {
    _checkInstrumentationEnabled();
    return _instance;
  }

  static final Instrumentation _instance = Instrumentation._();

  static void _checkInstrumentationEnabled() {
    if (!enabled) {
      throw StateError(
        'Cannot use Instrumentation unless it is enabled. '
        'You can enable it by setting the `FLUTTER_WEB_ENABLE_INSTRUMENTATION` '
        'environment variable to true, or by passing '
        '--dart-define=FLUTTER_WEB_ENABLE_INSTRUMENTATION=true to the flutter '
        'tool.',
      );
    }
  }

  Map<String, int> get debugCounters => _counters;
  final Map<String, int> _counters = <String, int>{};

  Timer? get debugPrintTimer => _printTimer;
  Timer? _printTimer;

  /// Increments the count of a particular event by one.
  void incrementCounter(String event) {
    _checkInstrumentationEnabled();
    final int currentCount = _counters[event] ?? 0;
    _counters[event] = currentCount + 1;
    _printTimer ??= Timer(const Duration(seconds: 2), () {
      if (_printTimer == null || !_enabled) {
        return;
      }
      final StringBuffer message = StringBuffer('Engine counters:\n');
      // Entries are sorted for readability and testability.
      final List<MapEntry<String, int>> entries =
          _counters.entries.toList()..sort((MapEntry<String, int> a, MapEntry<String, int> b) {
            return a.key.compareTo(b.key);
          });
      for (final MapEntry<String, int> entry in entries) {
        message.writeln('  ${entry.key}: ${entry.value}');
      }
      print(message);
      _printTimer = null;
    });
  }
}
