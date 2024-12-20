// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter_test/flutter_test.dart';

import 'package:flutter_test/flutter_test.dart' as flutter_test show expect;

import 'package:matcher/expect.dart' as matcher show expect;

// We have to use matcher's expect because the flutter_test expect() goes
// out of its way to check that we're not leaking APIs and the whole point
// of this test is to see how we handle leaking APIs.

class TestAPI {
  Future<Object?> testGuard1() {
    return TestAsyncUtils.guard<Object?>(() async {
      return null;
    });
  }

  Future<Object?> testGuard2() {
    return TestAsyncUtils.guard<Object?>(() async {
      return null;
    });
  }
}

class TestAPISubclass extends TestAPI {
  Future<Object?> testGuard3() {
    return TestAsyncUtils.guard<Object?>(() async {
      return null;
    });
  }
}

class RecognizableTestException implements Exception {
  const RecognizableTestException();
}

Future<Object> _guardedThrower() {
  return TestAsyncUtils.guard<Object>(() async {
    throw const RecognizableTestException();
  });
}

void main() {
  test('TestAsyncUtils - one class', () async {
    final TestAPI testAPI = TestAPI();
    Future<Object?>? f1, f2;
    f1 = testAPI.testGuard1();
    try {
      f2 = testAPI.testGuard2();
      fail('unexpectedly did not throw');
    } on FlutterError catch (e) {
      final List<String> lines = e.message.split('\n');
      matcher.expect(lines[0], 'Guarded function conflict.');
      matcher.expect(lines[1], 'You must use "await" with all Future-returning test APIs.');
      matcher.expect(
        lines[2],
        matches(
          r'The guarded method "testGuard1" from class TestAPI was called from .*test_async_utils_test.dart on line [0-9]+\.',
        ),
      );
      matcher.expect(
        lines[3],
        matches(
          r'Then, the "testGuard2" method \(also from class TestAPI\) was called from .*test_async_utils_test.dart on line [0-9]+\.',
        ),
      );
      matcher.expect(
        lines[4],
        'The first method (TestAPI.testGuard1) had not yet finished executing at the time that the second method (TestAPI.testGuard2) was called. Since both are guarded, and the second was not a nested call inside the first, the first must complete its execution before the second can be called. Typically, this is achieved by putting an "await" statement in front of the call to the first.',
      );
      matcher.expect(lines[5], '');
      matcher.expect(
        lines[6],
        'When the first method (TestAPI.testGuard1) was called, this was the stack:',
      );
      matcher.expect(lines.length, greaterThan(6));
    }
    expect(await f1, isNull);
    expect(f2, isNull);
  }, skip: kIsWeb); // [intended] depends on platform-specific stack traces.

  test('TestAsyncUtils - two classes, all callers in superclass', () async {
    final TestAPI testAPI = TestAPISubclass();
    Future<Object?>? f1, f2;
    f1 = testAPI.testGuard1();
    try {
      f2 = testAPI.testGuard2();
      fail('unexpectedly did not throw');
    } on FlutterError catch (e) {
      final List<String> lines = e.message.split('\n');
      matcher.expect(lines[0], 'Guarded function conflict.');
      matcher.expect(lines[1], 'You must use "await" with all Future-returning test APIs.');
      matcher.expect(
        lines[2],
        matches(
          r'^The guarded method "testGuard1" from class TestAPI was called from .*test_async_utils_test.dart on line [0-9]+\.$',
        ),
      );
      matcher.expect(
        lines[3],
        matches(
          r'^Then, the "testGuard2" method \(also from class TestAPI\) was called from .*test_async_utils_test.dart on line [0-9]+\.$',
        ),
      );
      matcher.expect(
        lines[4],
        'The first method (TestAPI.testGuard1) had not yet finished executing at the time that the second method (TestAPI.testGuard2) was called. Since both are guarded, and the second was not a nested call inside the first, the first must complete its execution before the second can be called. Typically, this is achieved by putting an "await" statement in front of the call to the first.',
      );
      matcher.expect(lines[5], '');
      matcher.expect(
        lines[6],
        'When the first method (TestAPI.testGuard1) was called, this was the stack:',
      );
      matcher.expect(lines.length, greaterThan(7));
    }
    expect(await f1, isNull);
    expect(f2, isNull);
  }, skip: kIsWeb); // [intended] depends on platform-specific stack traces.

  test('TestAsyncUtils - two classes, mixed callers', () async {
    final TestAPISubclass testAPI = TestAPISubclass();
    Future<Object?>? f1, f2;
    f1 = testAPI.testGuard1();
    try {
      f2 = testAPI.testGuard3();
      fail('unexpectedly did not throw');
    } on FlutterError catch (e) {
      final List<String> lines = e.message.split('\n');
      matcher.expect(lines[0], 'Guarded function conflict.');
      matcher.expect(lines[1], 'You must use "await" with all Future-returning test APIs.');
      matcher.expect(
        lines[2],
        matches(
          r'^The guarded method "testGuard1" from class TestAPI was called from .*test_async_utils_test.dart on line [0-9]+\.$',
        ),
      );
      matcher.expect(
        lines[3],
        matches(
          r'^Then, the "testGuard3" method from class TestAPISubclass was called from .*test_async_utils_test.dart on line [0-9]+\.$',
        ),
      );
      matcher.expect(
        lines[4],
        'The first method (TestAPI.testGuard1) had not yet finished executing at the time that the second method (TestAPISubclass.testGuard3) was called. Since both are guarded, and the second was not a nested call inside the first, the first must complete its execution before the second can be called. Typically, this is achieved by putting an "await" statement in front of the call to the first.',
      );
      matcher.expect(lines[5], '');
      matcher.expect(
        lines[6],
        'When the first method (TestAPI.testGuard1) was called, this was the stack:',
      );
      matcher.expect(lines.length, greaterThan(7));
    }
    expect(await f1, isNull);
    expect(f2, isNull);
  }, skip: kIsWeb); // [intended] depends on platform-specific stack traces.

  test('TestAsyncUtils - expect() catches pending async work', () async {
    final TestAPI testAPI = TestAPISubclass();
    Future<Object?>? f1;
    f1 = testAPI.testGuard1();
    try {
      flutter_test.expect(0, 0);
      fail('unexpectedly did not throw');
    } on FlutterError catch (e) {
      final List<String> lines = e.message.split('\n');
      matcher.expect(lines[0], 'Guarded function conflict.');
      matcher.expect(lines[1], 'You must use "await" with all Future-returning test APIs.');
      matcher.expect(
        lines[2],
        matches(
          r'^The guarded method "testGuard1" from class TestAPI was called from .*test_async_utils_test.dart on line [0-9]+\.$',
        ),
      );
      matcher.expect(
        lines[3],
        matches(
          r'^Then, the "expect" function was called from .*test_async_utils_test.dart on line [0-9]+\.$',
        ),
      );
      matcher.expect(
        lines[4],
        'The first method (TestAPI.testGuard1) had not yet finished executing at the time that the second function (expect) was called. Since both are guarded, and the second was not a nested call inside the first, the first must complete its execution before the second can be called. Typically, this is achieved by putting an "await" statement in front of the call to the first.',
      );
      matcher.expect(
        lines[5],
        'If you are confident that all test APIs are being called using "await", and this expect() call is not being called at the top level but is itself being called from some sort of callback registered before the testGuard1 method was called, then consider using expectSync() instead.',
      );
      matcher.expect(lines[6], '');
      matcher.expect(
        lines[7],
        'When the first method (TestAPI.testGuard1) was called, this was the stack:',
      );
      matcher.expect(lines.length, greaterThan(7));
    }
    expect(await f1, isNull);
  }, skip: kIsWeb); // [intended] depends on platform-specific stack traces.

  testWidgets('TestAsyncUtils - expect() catches pending async work', (WidgetTester tester) async {
    Future<Object?>? f1, f2;
    try {
      f1 = tester.pump();
      f2 = tester.pump();
      fail('unexpectedly did not throw');
    } on FlutterError catch (e) {
      final List<String> lines = e.message.split('\n');
      matcher.expect(lines[0], 'Guarded function conflict.');
      matcher.expect(lines[1], 'You must use "await" with all Future-returning test APIs.');
      matcher.expect(
        lines[2],
        matches(
          r'^The guarded method "pump" from class WidgetTester was called from .*test_async_utils_test.dart on line [0-9]+\.$',
        ),
      );
      matcher.expect(
        lines[3],
        matches(r'^Then, it was called from .*test_async_utils_test.dart on line [0-9]+\.$'),
      );
      matcher.expect(
        lines[4],
        'The first method had not yet finished executing at the time that the second method was called. Since both are guarded, and the second was not a nested call inside the first, the first must complete its execution before the second can be called. Typically, this is achieved by putting an "await" statement in front of the call to the first.',
      );
      matcher.expect(lines[5], '');
      matcher.expect(lines[6], 'When the first method was called, this was the stack:');
      matcher.expect(lines.length, greaterThan(7));
      // TODO(jacobr): add more tests like this if they are useful.

      final DiagnosticPropertiesBuilder propertiesBuilder = DiagnosticPropertiesBuilder();
      e.debugFillProperties(propertiesBuilder);
      final List<DiagnosticsNode> information = propertiesBuilder.properties;
      matcher.expect(information.length, 6);
      matcher.expect(information[0].level, DiagnosticLevel.summary);
      matcher.expect(information[1].level, DiagnosticLevel.hint);
      matcher.expect(information[2].level, DiagnosticLevel.info);
      matcher.expect(information[3].level, DiagnosticLevel.info);
      matcher.expect(information[4].level, DiagnosticLevel.info);
      matcher.expect(information[5].level, DiagnosticLevel.info);
      matcher.expect(information[0], isA<DiagnosticsProperty<void>>());
      matcher.expect(information[1], isA<DiagnosticsProperty<void>>());
      matcher.expect(information[2], isA<DiagnosticsProperty<void>>());
      matcher.expect(information[3], isA<DiagnosticsProperty<void>>());
      matcher.expect(information[4], isA<DiagnosticsProperty<void>>());
      matcher.expect(information[5], isA<DiagnosticsStackTrace>());
      final DiagnosticsStackTrace stackTraceProperty = information[5] as DiagnosticsStackTrace;
      matcher.expect(
        stackTraceProperty.name,
        '\nWhen the first method was called, this was the stack',
      );
      matcher.expect(stackTraceProperty.value, isA<StackTrace>());
    }
    await f1;
    await f2;
  }, skip: kIsWeb); // [intended] depends on platform-specific stack traces.

  testWidgets('TestAsyncUtils - expect() catches pending async work', (WidgetTester tester) async {
    Future<Object?>? f1;
    try {
      f1 = tester.pump();
      TestAsyncUtils.verifyAllScopesClosed();
      fail('unexpectedly did not throw');
    } on FlutterError catch (e) {
      final List<String> lines = e.message.split('\n');
      matcher.expect(lines[0], 'Asynchronous call to guarded function leaked.');
      matcher.expect(lines[1], 'You must use "await" with all Future-returning test APIs.');
      matcher.expect(
        lines[2],
        matches(
          r'^The guarded method "pump" from class WidgetTester was called from .*test_async_utils_test.dart on line [0-9]+, but never completed before its parent scope closed\.$',
        ),
      );
      matcher.expect(
        lines[3],
        matches(
          r'^The guarded method "pump" from class AutomatedTestWidgetsFlutterBinding was called from [^ ]+ on line [0-9]+, but never completed before its parent scope closed\.',
        ),
      );
      matcher.expect(lines.length, 4);
      final DiagnosticPropertiesBuilder propertiesBuilder = DiagnosticPropertiesBuilder();
      e.debugFillProperties(propertiesBuilder);
      final List<DiagnosticsNode> information = propertiesBuilder.properties;
      matcher.expect(information.length, 4);
      matcher.expect(information[0].level, DiagnosticLevel.summary);
      matcher.expect(information[1].level, DiagnosticLevel.hint);
      matcher.expect(information[2].level, DiagnosticLevel.info);
      matcher.expect(information[3].level, DiagnosticLevel.info);
    }
    await f1;
  }, skip: kIsWeb); // [intended] depends on platform-specific stack traces.

  testWidgets('TestAsyncUtils - expect() catches pending async work', (WidgetTester tester) async {
    Future<Object?>? f1;
    try {
      f1 = tester.pump();
      TestAsyncUtils.verifyAllScopesClosed();
      fail('unexpectedly did not throw');
    } on FlutterError catch (e) {
      final List<String> lines = e.message.split('\n');
      matcher.expect(lines[0], 'Asynchronous call to guarded function leaked.');
      matcher.expect(lines[1], 'You must use "await" with all Future-returning test APIs.');
      matcher.expect(
        lines[2],
        matches(
          r'^The guarded method "pump" from class WidgetTester was called from .*test_async_utils_test.dart on line [0-9]+, but never completed before its parent scope closed\.$',
        ),
      );
      matcher.expect(
        lines[3],
        matches(
          r'^The guarded method "pump" from class AutomatedTestWidgetsFlutterBinding was called from [^ ]+ on line [0-9]+, but never completed before its parent scope closed\.',
        ),
      );
      matcher.expect(lines.length, 4);
      final DiagnosticPropertiesBuilder propertiesBuilder = DiagnosticPropertiesBuilder();
      e.debugFillProperties(propertiesBuilder);
      final List<DiagnosticsNode> information = propertiesBuilder.properties;
      matcher.expect(information.length, 4);
      matcher.expect(information[0].level, DiagnosticLevel.summary);
      matcher.expect(information[1].level, DiagnosticLevel.hint);
      matcher.expect(information[2].level, DiagnosticLevel.info);
      matcher.expect(information[3].level, DiagnosticLevel.info);
    }
    await f1;
  }, skip: kIsWeb); // [intended] depends on platform-specific stack traces.

  testWidgets('TestAsyncUtils - guard body can throw', (WidgetTester tester) async {
    try {
      await _guardedThrower();
      expect(false, true); // _guardedThrower should throw and we shouldn't reach here
    } on RecognizableTestException catch (e) {
      expect(e, const RecognizableTestException());
    }
  });

  test('TestAsyncUtils - web', () async {
    final TestAPI testAPI = TestAPI();
    Future<Object?>? f1, f2;
    f1 = testAPI.testGuard1();
    try {
      f2 = testAPI.testGuard2();
      fail('unexpectedly did not throw');
    } on FlutterError catch (e) {
      final List<String> lines = e.message.split('\n');
      matcher.expect(lines[0], 'Guarded function conflict.');
      matcher.expect(lines[1], 'You must use "await" with all Future-returning test APIs.');
      matcher.expect(lines[2], '');
      matcher.expect(lines[3], 'When the first function was called, this was the stack:');
      matcher.expect(lines.length, greaterThan(3));
    }
    expect(await f1, isNull);
    expect(f2, isNull);
  }, skip: !kIsWeb); // [intended] depends on platform-specific stack traces.

  // see also dev/manual_tests/test_data which contains tests run by the flutter_tools tests for 'flutter test'
}
