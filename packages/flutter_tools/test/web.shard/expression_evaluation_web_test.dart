// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

@Tags(<String>['flutter-test-driver'])
library;

import 'package:file/file.dart';
import 'package:vm_service/vm_service.dart';

import '../integration.shard/test_data/basic_project.dart';
import '../integration.shard/test_data/tests_project.dart';
import '../integration.shard/test_driver.dart';
import '../integration.shard/test_utils.dart';
import '../src/common.dart';

void main() {
  group('Flutter run for web', () {
    final BasicProject project = BasicProject();
    late Directory tempDir;
    late FlutterRunTestDriver flutter;

    setUp(() async {
      tempDir = createResolvedTempDirectorySync('run_expression_eval_test.');
      await project.setUpIn(tempDir);
      flutter = FlutterRunTestDriver(tempDir);
      flutter.stdout.listen((String line) {
        expect(line, isNot(contains('Unresolved uri:')));
        expect(line, isNot(contains('No module for')));
      });
    });

    tearDown(() async {
      await flutter.stop();
      tryToDelete(tempDir);
    });

    Future<void> start({required bool expressionEvaluation}) async {
      // The non-test project has a loop around its breakpoints.
      // No need to start paused as all breakpoint would be eventually reached.
      await flutter.run(
        withDebugger: true,
        chrome: true,
        expressionEvaluation: expressionEvaluation,
        additionalCommandArgs: <String>['--verbose'],
      );
    }

    Future<void> breakInBuildMethod(FlutterTestDriver flutter) async {
      await flutter.breakAt(project.buildMethodBreakpointUri, project.buildMethodBreakpointLine);
    }

    Future<void> breakInTopLevelFunction(FlutterTestDriver flutter) async {
      await flutter.breakAt(
        project.topLevelFunctionBreakpointUri,
        project.topLevelFunctionBreakpointLine,
      );
    }

    testWithoutContext('cannot evaluate expression if feature is disabled', () async {
      await start(expressionEvaluation: false);
      await breakInTopLevelFunction(flutter);
      await failToEvaluateExpression(flutter);
    });

    testWithoutContext('shows no native javascript objects in static scope', () async {
      await start(expressionEvaluation: true);
      await breakInTopLevelFunction(flutter);
      await checkStaticScope(flutter);
    });

    testWithoutContext('can handle compilation errors', () async {
      await start(expressionEvaluation: true);
      await breakInTopLevelFunction(flutter);
      await evaluateErrorExpressions(flutter);
    });

    testWithoutContext('can evaluate trivial expressions in top level function', () async {
      await start(expressionEvaluation: true);
      await breakInTopLevelFunction(flutter);
      await evaluateTrivialExpressions(flutter);
    });

    testWithoutContext('can evaluate trivial expressions in build method', () async {
      await start(expressionEvaluation: true);
      await breakInBuildMethod(flutter);
      await evaluateTrivialExpressions(flutter);
    });

    testWithoutContext('can evaluate complex expressions in top level function', () async {
      await start(expressionEvaluation: true);
      await breakInTopLevelFunction(flutter);
      await evaluateComplexExpressions(flutter);
    });

    testWithoutContext('can evaluate complex expressions in build method', () async {
      await start(expressionEvaluation: true);
      await breakInBuildMethod(flutter);
      await evaluateComplexExpressions(flutter);
    });

    testWithoutContext('can evaluate trivial expressions in library without pause', () async {
      await start(expressionEvaluation: true);
      await evaluateTrivialExpressionsInLibrary(flutter);
    });

    testWithoutContext('can evaluate complex expressions in library without pause', () async {
      await start(expressionEvaluation: true);
      await evaluateComplexExpressionsInLibrary(flutter);
    });

    testWithoutContext('evaluated expression includes web library environment defines', () async {
      await start(expressionEvaluation: true);
      await evaluateWebLibraryBooleanFromEnvironmentInLibrary(flutter);
    });
  });

  group('Flutter test for web', () {
    final TestsProject project = TestsProject();
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

    Future<Isolate?> breakInMethod(FlutterTestDriver flutter) async {
      await flutter.addBreakpoint(project.breakpointAppUri, project.breakpointLine);
      return flutter.resume(waitForNextPause: true);
    }

    Future<void> startPaused({required bool expressionEvaluation}) {
      // The test project does not have a loop around its breakpoints.
      // Start paused so we can set a breakpoint before passing it
      // in the execution.
      return flutter.run(
        withDebugger: true,
        chrome: true,
        expressionEvaluation: expressionEvaluation,
        startPaused: true,
        script: project.testFilePath,
        additionalCommandArgs: <String>['--verbose'],
      );
    }

    testWithoutContext('cannot evaluate expressions if feature is disabled', () async {
      await startPaused(expressionEvaluation: false);
      await breakInMethod(flutter);
      await failToEvaluateExpression(flutter);
    });

    testWithoutContext('can evaluate trivial expressions in a test', () async {
      await startPaused(expressionEvaluation: true);
      await breakInMethod(flutter);
      await evaluateTrivialExpressions(flutter);
    });

    testWithoutContext('can evaluate complex expressions in a test', () async {
      await startPaused(expressionEvaluation: true);
      await breakInMethod(flutter);
      await evaluateComplexExpressions(flutter);
    });

    testWithoutContext('can evaluate trivial expressions in library without pause', () async {
      await startPaused(expressionEvaluation: true);
      await evaluateTrivialExpressionsInLibrary(flutter);
    });

    testWithoutContext('can evaluate complex expressions in library without pause', () async {
      await startPaused(expressionEvaluation: true);
      await evaluateComplexExpressionsInLibrary(flutter);
    });
    testWithoutContext('evaluated expression includes web library environment defines', () async {
      await startPaused(expressionEvaluation: true);
      await evaluateWebLibraryBooleanFromEnvironmentInLibrary(flutter);
    });
  });
}

Future<void> failToEvaluateExpression(FlutterTestDriver flutter) async {
  await expectLater(
    flutter.evaluateInFrame('"test"'),
    throwsA(
      isA<RPCError>().having(
        (RPCError error) => error.message,
        'message',
        contains('Expression evaluation is not supported for this configuration'),
      ),
    ),
  );
}

Future<void> checkStaticScope(FlutterTestDriver flutter) async {
  final Frame res = await flutter.getTopStackFrame();
  expect(res.vars, equals(<BoundVariable>[]));
}

Future<void> evaluateErrorExpressions(FlutterTestDriver flutter) async {
  final ObjRef res = await flutter.evaluateInFrame('typo');
  expectError(res, 'CompilationError:');
}

Future<void> evaluateTrivialExpressions(FlutterTestDriver flutter) async {
  ObjRef res;

  res = await flutter.evaluateInFrame('"test"');
  expectInstance(res, InstanceKind.kString, 'test');

  res = await flutter.evaluateInFrame('1');
  expectInstance(res, InstanceKind.kDouble, 1.toString());

  res = await flutter.evaluateInFrame('true');
  expectInstance(res, InstanceKind.kBool, true.toString());
}

Future<void> evaluateComplexExpressions(FlutterTestDriver flutter) async {
  final ObjRef res = await flutter.evaluateInFrame('new DateTime.now().year');
  expectInstance(res, InstanceKind.kDouble, DateTime.now().year.toString());
}

Future<void> evaluateTrivialExpressionsInLibrary(FlutterTestDriver flutter) async {
  final LibraryRef library = await getRootLibrary(flutter);
  final ObjRef res = await flutter.evaluate(library.id!, '"test"');
  expectInstance(res, InstanceKind.kString, 'test');
}

Future<void> evaluateComplexExpressionsInLibrary(FlutterTestDriver flutter) async {
  final LibraryRef library = await getRootLibrary(flutter);
  final ObjRef res = await flutter.evaluate(library.id!, 'new DateTime.now().year');
  expectInstance(res, InstanceKind.kDouble, DateTime.now().year.toString());
}

Future<void> evaluateWebLibraryBooleanFromEnvironmentInLibrary(FlutterTestDriver flutter) async {
  final LibraryRef library = await getRootLibrary(flutter);
  final ObjRef res = await flutter.evaluate(
    library.id!,
    'const bool.fromEnvironment("dart.library.html")',
  );
  expectInstance(res, InstanceKind.kBool, true.toString());
}

Future<LibraryRef> getRootLibrary(FlutterTestDriver flutter) async {
  // `isolate.rootLib` returns incorrect library, so find the
  // entrypoint manually here instead.
  //
  // Issue: https://github.com/dart-lang/sdk/issues/44760
  final Isolate isolate = await flutter.getFlutterIsolate();
  return isolate.libraries!.firstWhere((LibraryRef l) => l.uri!.contains('org-dartlang-app'));
}

void expectInstance(ObjRef result, String kind, String message) {
  expect(
    result,
    const TypeMatcher<InstanceRef>()
        .having((InstanceRef instance) => instance.kind, 'kind', kind)
        .having((InstanceRef instance) => instance.valueAsString, 'valueAsString', message),
  );
}

void expectError(ObjRef result, String message) {
  expect(
    result,
    const TypeMatcher<ErrorRef>().having(
      (ErrorRef instance) => instance.message,
      'message',
      contains(message),
    ),
  );
}
