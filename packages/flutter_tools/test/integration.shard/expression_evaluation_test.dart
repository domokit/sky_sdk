// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:file/file.dart';

import 'package:vm_service/vm_service.dart';

import '../src/common.dart';
import 'test_data/basic_project.dart';
import 'test_data/integration_tests_project.dart';
import 'test_data/tests_project.dart';
import 'test_driver.dart';
import 'test_utils.dart';

void batch1() {
  final BasicProject project = BasicProject();
  late Directory tempDir;
  late FlutterRunTestDriver flutter;

  setUp(() async {
    tempDir = createResolvedTempDirectorySync('run_expression_eval_test.');
    await project.setUpIn(tempDir);
    flutter = FlutterRunTestDriver(tempDir);
  });

  tearDown(() async {
    await flutter.stop();
    tryToDelete(tempDir);
  });

  Future<void> breakInBuildMethod(FlutterTestDriver flutter) async {
    await flutter.breakAt(project.buildMethodBreakpointUri, project.buildMethodBreakpointLine);
  }

  Future<void> breakInTopLevelFunction(FlutterTestDriver flutter) async {
    await flutter.breakAt(
      project.topLevelFunctionBreakpointUri,
      project.topLevelFunctionBreakpointLine,
    );
  }

  testWithoutContext('can evaluate trivial expressions in top level function', () async {
    await flutter.run(withDebugger: true);
    await breakInTopLevelFunction(flutter);
    await evaluateTrivialExpressions(flutter);
  });

  testWithoutContext('can evaluate trivial expressions in build method', () async {
    await flutter.run(withDebugger: true);
    await breakInBuildMethod(flutter);
    await evaluateTrivialExpressions(flutter);
  });

  testWithoutContext('can evaluate complex expressions in top level function', () async {
    await flutter.run(withDebugger: true);
    await breakInTopLevelFunction(flutter);
    await evaluateComplexExpressions(flutter);
  });

  testWithoutContext('can evaluate complex expressions in build method', () async {
    await flutter.run(withDebugger: true);
    await breakInBuildMethod(flutter);
    await evaluateComplexExpressions(flutter);
  });

  testWithoutContext(
    'can evaluate expressions returning complex objects in top level function',
    () async {
      await flutter.run(withDebugger: true);
      await breakInTopLevelFunction(flutter);
      await evaluateComplexReturningExpressions(flutter);
    },
  );

  testWithoutContext(
    'can evaluate expressions returning complex objects in build method',
    () async {
      await flutter.run(withDebugger: true);
      await breakInBuildMethod(flutter);
      await evaluateComplexReturningExpressions(flutter);
    },
  );

  testWithoutContext(
    'evaluating invalid expressions throws an exception with compilation error details',
    () async {
      await flutter.run(withDebugger: true);
      await breakInTopLevelFunction(flutter);
      await evaluateInvalidExpression(flutter);
    },
  );
}

void batch2() {
  final TestsProject project = TestsProject();
  late Directory tempDir;
  late FlutterTestTestDriver flutter;

  setUp(() async {
    tempDir = createResolvedTempDirectorySync('test_expression_eval_test.');
    await project.setUpIn(tempDir);
    flutter = FlutterTestTestDriver(tempDir);
  });

  tearDown(() async {
    await flutter.waitForCompletion();
    tryToDelete(tempDir);
  });

  testWithoutContext('can evaluate trivial expressions in a test', () async {
    await flutter.test(
      withDebugger: true,
      beforeStart: () => flutter.addBreakpoint(project.breakpointUri, project.breakpointLine),
    );
    await flutter.waitForPause();
    await evaluateTrivialExpressions(flutter);

    // Ensure we did not leave a dill file alongside the test.
    // https://github.com/Dart-Code/Dart-Code/issues/4243.
    final String dillFilename = '${project.testFilePath}.dill';
    expect(fileSystem.file(dillFilename).existsSync(), isFalse);
  });

  testWithoutContext('can evaluate complex expressions in a test', () async {
    await flutter.test(
      withDebugger: true,
      beforeStart: () => flutter.addBreakpoint(project.breakpointUri, project.breakpointLine),
    );
    await flutter.waitForPause();
    await evaluateComplexExpressions(flutter);
  });

  testWithoutContext('can evaluate expressions returning complex objects in a test', () async {
    await flutter.test(
      withDebugger: true,
      beforeStart: () => flutter.addBreakpoint(project.breakpointUri, project.breakpointLine),
    );
    await flutter.waitForPause();
    await evaluateComplexReturningExpressions(flutter);
  });

  testWithoutContext(
    'evaluating invalid expressions throws an exception with compilation error details',
    () async {
      await flutter.test(
        withDebugger: true,
        beforeStart: () => flutter.addBreakpoint(project.breakpointUri, project.breakpointLine),
      );
      await flutter.waitForPause();
      await evaluateInvalidExpression(flutter);
    },
  );
}

void batch3() {
  final IntegrationTestsProject project = IntegrationTestsProject();
  late Directory tempDir;
  late FlutterTestTestDriver flutter;

  setUp(() async {
    tempDir = createResolvedTempDirectorySync('integration_test_expression_eval_test.');
    await project.setUpIn(tempDir);
    flutter = FlutterTestTestDriver(tempDir);
  });

  tearDown(() async {
    await flutter.waitForCompletion();
    tryToDelete(tempDir);
  });

  testWithoutContext('can evaluate expressions in a test', () async {
    await flutter.test(
      deviceId: 'flutter-tester',
      testFile: project.testFilePath,
      withDebugger: true,
      beforeStart: () => flutter.addBreakpoint(project.breakpointUri, project.breakpointLine),
    );
    await flutter.waitForPause();
    await evaluateTrivialExpressions(flutter);

    // Ensure we did not leave a dill file alongside the test.
    // https://github.com/Dart-Code/Dart-Code/issues/4243.
    final String dillFilename = '${project.testFilePath}.dill';
    expect(fileSystem.file(dillFilename).existsSync(), isFalse);
  });

  testWithoutContext(
    'evaluating invalid expressions throws an exception with compilation error details',
    () async {
      await flutter.test(
        deviceId: 'flutter-tester',
        testFile: project.testFilePath,
        withDebugger: true,
        beforeStart: () => flutter.addBreakpoint(project.breakpointUri, project.breakpointLine),
      );
      await flutter.waitForPause();
      await evaluateInvalidExpression(flutter);

      // Ensure we did not leave a dill file alongside the test.
      // https://github.com/Dart-Code/Dart-Code/issues/4243.
      final String dillFilename = '${project.testFilePath}.dill';
      expect(fileSystem.file(dillFilename).existsSync(), isFalse);
    },
  );
}

Future<void> evaluateTrivialExpressions(FlutterTestDriver flutter) async {
  ObjRef res;

  res = await flutter.evaluateInFrame('"test"');
  expectValueOfType(res, InstanceKind.kString, 'test');

  res = await flutter.evaluateInFrame('1');
  expectValueOfType(res, InstanceKind.kInt, 1.toString());

  res = await flutter.evaluateInFrame('true');
  expectValueOfType(res, InstanceKind.kBool, true.toString());
}

Future<void> evaluateComplexExpressions(FlutterTestDriver flutter) async {
  final ObjRef res = await flutter.evaluateInFrame('new DateTime(2000).year');
  expectValueOfType(res, InstanceKind.kInt, '2000');
}

Future<void> evaluateComplexReturningExpressions(FlutterTestDriver flutter) async {
  final DateTime date = DateTime(2000);
  final ObjRef resp = await flutter.evaluateInFrame('new DateTime(2000)');
  expectInstanceOfClass(resp, 'DateTime');
  final ObjRef res = await flutter.evaluate(resp.id!, r'"$year-$month-$day"');
  expectValue(res, '${date.year}-${date.month}-${date.day}');
}

Future<void> evaluateInvalidExpression(FlutterTestDriver flutter) async {
  try {
    await flutter.evaluateInFrame('is Foo');
    fail("'is Foo' is not a valid expression");
  } on RPCError catch (e) {
    expect(e.code, RPCErrorKind.kExpressionCompilationError.code);
    expect(e.message, RPCErrorKind.kExpressionCompilationError.message);
    expect(
      e.details,
      "org-dartlang-debug:synthetic_debug_expression:1:1: Error: Expected an identifier, but got 'is'.\n"
      "Try inserting an identifier before 'is'.\n"
      'is Foo\n'
      '^^\n'
      "org-dartlang-debug:synthetic_debug_expression:1:4: Error: 'Foo' isn't a type.\n"
      'is Foo\n'
      '   ^^^\n',
    );
  }
}

void expectInstanceOfClass(ObjRef result, String name) {
  expect(
    result,
    const TypeMatcher<InstanceRef>().having(
      (InstanceRef instance) => instance.classRef!.name,
      'resp.classRef.name',
      name,
    ),
  );
}

void expectValueOfType(ObjRef result, String kind, String message) {
  expect(
    result,
    const TypeMatcher<InstanceRef>()
        .having((InstanceRef instance) => instance.kind, 'kind', kind)
        .having((InstanceRef instance) => instance.valueAsString, 'valueAsString', message),
  );
}

void expectValue(ObjRef result, String message) {
  expect(
    result,
    const TypeMatcher<InstanceRef>().having(
      (InstanceRef instance) => instance.valueAsString,
      'valueAsString',
      message,
    ),
  );
}

void main() {
  group('flutter run expression evaluation -', batch1);
  group('flutter test expression evaluation -', batch2);
  group('flutter integration test expression evaluation -', batch3);
}
