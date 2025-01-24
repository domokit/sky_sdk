// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';
import 'package:ui/ui_web/src/ui_web.dart' as ui_web;

import '../common/spy.dart';

void main() {
  internalBootstrapBrowserTest(() => testMain);
}

void testMain() {
  group('$Profiler', () {
    _profilerTests();
  });

  group('$Instrumentation', () {
    _instrumentationTests();
  });
}

void _profilerTests() {
  final List<String> warnings = <String>[];
  late void Function(String) oldPrintWarning;

  setUpAll(() {
    oldPrintWarning = printWarning;
    printWarning = (String warning) {
      warnings.add(warning);
    };
  });

  setUp(() {
    warnings.clear();
    Profiler.isBenchmarkMode = true;
    Profiler.ensureInitialized();
  });

  tearDownAll(() {
    printWarning = oldPrintWarning;
  });

  tearDown(() {
    ui_web.benchmarkValueCallback = null;
    Profiler.isBenchmarkMode = false;
  });

  test('works when there is no listener', () {
    expect(() => Profiler.instance.benchmark('foo', 123), returnsNormally);
  });

  test('can listen to benchmarks', () {
    final List<BenchmarkDatapoint> data = <BenchmarkDatapoint>[];
    ui_web.benchmarkValueCallback = (String name, double value) {
      data.add((name, value));
    };

    Profiler.instance.benchmark('foo', 123);
    expect(data, <BenchmarkDatapoint>[('foo', 123)]);
    data.clear();

    Profiler.instance.benchmark('bar', 0.0125);
    expect(data, <BenchmarkDatapoint>[('bar', 0.0125)]);
    data.clear();

    // Remove listener and make sure nothing breaks and the data isn't being
    // sent to the old callback anymore.
    ui_web.benchmarkValueCallback = null;
    expect(() => Profiler.instance.benchmark('baz', 99.999), returnsNormally);
    expect(data, isEmpty);
  });
}

void _instrumentationTests() {
  setUp(() {
    Instrumentation.enabled = false;
  });

  tearDown(() {
    Instrumentation.enabled = false;
  });

  test('when disabled throws instead of initializing', () {
    expect(() => Instrumentation.instance, throwsStateError);
  });

  test('when disabled throws instead of incrementing counter', () {
    Instrumentation.enabled = true;
    final Instrumentation instrumentation = Instrumentation.instance;
    Instrumentation.enabled = false;
    expect(() => instrumentation.incrementCounter('test'), throwsStateError);
  });

  test('when enabled increments counter', () {
    final ZoneSpy spy = ZoneSpy();
    spy.run(() {
      Instrumentation.enabled = true;
      final Instrumentation instrumentation = Instrumentation.instance;
      expect(instrumentation.debugPrintTimer, isNull);
      instrumentation.incrementCounter('foo');
      expect(instrumentation.debugPrintTimer, isNotNull);
      instrumentation.incrementCounter('foo');
      instrumentation.incrementCounter('bar');
      expect(spy.printLog, isEmpty);

      expect(instrumentation.debugPrintTimer, isNotNull);
      spy.fakeAsync.elapse(const Duration(seconds: 2));
      expect(instrumentation.debugPrintTimer, isNull);
      expect(spy.printLog, hasLength(1));
      expect(
        spy.printLog.single,
        'Engine counters:\n'
        '  bar: 1\n'
        '  foo: 2\n',
      );
    });
  });
}

typedef BenchmarkDatapoint = (String, double);
