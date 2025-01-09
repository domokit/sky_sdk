// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:js_interop';

import 'package:test/bootstrap/browser.dart';
import 'package:test/test.dart';
import 'package:ui/src/engine.dart';
import 'package:ui/ui_web/src/ui_web.dart' as ui_web;

import '../common/spy.dart';

@JS('window._flutter_internal_on_benchmark')
external set jsBenchmarkValueCallback(JSAny? object);

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
    jsBenchmarkValueCallback = null;
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

  // TODO(mdebbar): Remove this group once the JS API is removed.
  // https://github.com/flutter/flutter/issues/127395
  group('[JS API]', () {
    test('can listen to benchmarks', () {
      final List<BenchmarkDatapoint> data = <BenchmarkDatapoint>[];
      jsBenchmarkValueCallback =
          (String name, double value) {
            data.add((name, value));
          }.toJS;

      Profiler.instance.benchmark('foo', 123);
      expect(warnings, hasLength(1));
      expect(warnings.single, contains('deprecated'));
      expect(warnings.single, contains('benchmarkValueCallback'));
      expect(warnings.single, contains('dart:ui_web'));
      warnings.clear();

      expect(data, <BenchmarkDatapoint>[('foo', 123)]);
      data.clear();

      Profiler.instance.benchmark('bar', 0.0125);
      expect(data, <BenchmarkDatapoint>[('bar', 0.0125)]);
      data.clear();

      // Remove listener and make sure nothing breaks and the data isn't being
      // sent to the old callback anymore.
      jsBenchmarkValueCallback = null;
      expect(() => Profiler.instance.benchmark('baz', 99.999), returnsNormally);
      expect(data, isEmpty);
    });

    test('throws on wrong listener type', () {
      final List<BenchmarkDatapoint> data = <BenchmarkDatapoint>[];

      // Wrong callback signature.
      jsBenchmarkValueCallback =
          (double value) {
            data.add(('bad', value));
          }.toJS;
      expect(
        () => Profiler.instance.benchmark('foo', 123),

        // dart2js throws a NoSuchMethodError, dart2wasm throws a TypeError here.
        // Just make sure it throws an error in this case.
        throwsA(isA<Error>()),
      );
      expect(data, isEmpty);

      // Not even a callback.
      jsBenchmarkValueCallback = 'string'.toJS;
      expect(
        () => Profiler.instance.benchmark('foo', 123),
        // dart2js throws a TypeError, while dart2wasm throws an explicit
        // exception.
        throwsA(anything),
      );
    });

    test('can be combined with ui_web API', () {
      final List<BenchmarkDatapoint> uiWebData = <BenchmarkDatapoint>[];
      final List<BenchmarkDatapoint> jsData = <BenchmarkDatapoint>[];

      ui_web.benchmarkValueCallback = (String name, double value) {
        uiWebData.add((name, value));
      };
      jsBenchmarkValueCallback =
          (String name, double value) {
            jsData.add((name, value));
          }.toJS;

      Profiler.instance.benchmark('foo', 123);
      expect(warnings, hasLength(1));
      expect(warnings.single, contains('deprecated'));
      expect(warnings.single, contains('benchmarkValueCallback'));
      expect(warnings.single, contains('dart:ui_web'));
      warnings.clear();

      expect(uiWebData, <BenchmarkDatapoint>[('foo', 123)]);
      expect(jsData, <BenchmarkDatapoint>[('foo', 123)]);
      uiWebData.clear();
      jsData.clear();

      Profiler.instance.benchmark('bar', 0.0125);
      expect(uiWebData, <BenchmarkDatapoint>[('bar', 0.0125)]);
      expect(jsData, <BenchmarkDatapoint>[('bar', 0.0125)]);
      uiWebData.clear();
      jsData.clear();

      ui_web.benchmarkValueCallback = null;
      jsBenchmarkValueCallback = null;
      expect(() => Profiler.instance.benchmark('baz', 99.999), returnsNormally);
      expect(uiWebData, isEmpty);
      expect(jsData, isEmpty);
    });
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
