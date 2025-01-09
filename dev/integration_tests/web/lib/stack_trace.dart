// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:js_interop';

import 'package:collection/collection.dart';
import 'package:flutter/foundation.dart';
import 'package:meta/dart2js.dart';
import 'package:web/web.dart' as web;

/// Expected sequence of method calls.
const List<String> callChain = <String>['baz', 'bar', 'foo'];

final List<StackFrame> expectedProfileStackFrames =
    callChain.map<StackFrame>((String method) {
      return StackFrame(
        number: -1,
        packageScheme: '<unknown>',
        package: '<unknown>',
        packagePath: '<unknown>',
        line: -1,
        column: -1,
        className: 'Object',
        method: method,
        source: '',
      );
    }).toList();

// TODO(yjbanov): fix these stack traces when https://github.com/flutter/flutter/issues/50753 is fixed.
const List<StackFrame> expectedDebugStackFrames = <StackFrame>[
  StackFrame(
    number: -1,
    packageScheme: 'package',
    package: 'packages',
    packagePath: 'web_integration/stack_trace.dart',
    line: 119,
    column: 3,
    className: '<unknown>',
    method: 'baz',
    source: '',
  ),
  StackFrame(
    number: -1,
    packageScheme: 'package',
    package: 'packages',
    packagePath: 'web_integration/stack_trace.dart',
    line: 114,
    column: 3,
    className: '<unknown>',
    method: 'bar',
    source: '',
  ),
  StackFrame(
    number: -1,
    packageScheme: 'package',
    package: 'packages',
    packagePath: 'web_integration/stack_trace.dart',
    line: 109,
    column: 3,
    className: '<unknown>',
    method: 'foo',
    source: '',
  ),
];

/// Tests that we do not crash while parsing Web stack traces.
///
/// This test is run in debug, profile, and release modes.
void main() async {
  final StringBuffer output = StringBuffer();
  try {
    try {
      foo();
    } catch (expectedError, expectedStackTrace) {
      final List<StackFrame> parsedFrames = StackFrame.fromStackTrace(expectedStackTrace);
      if (parsedFrames.isEmpty) {
        throw Exception(
          'Failed to parse stack trace. Got empty list of stack frames.\n'
          'Stack trace:\n$expectedStackTrace',
        );
      }

      // Symbols in release mode are randomly obfuscated, so there's no good way to
      // validate the contents. However, profile mode can be checked.
      if (kProfileMode) {
        _checkStackFrameContents(parsedFrames, expectedProfileStackFrames, expectedStackTrace);
      }

      if (kDebugMode) {
        _checkStackFrameContents(parsedFrames, expectedDebugStackFrames, expectedStackTrace);
      }
    }
    output.writeln('--- TEST SUCCEEDED ---');
  } catch (unexpectedError, unexpectedStackTrace) {
    output.writeln('--- UNEXPECTED EXCEPTION ---');
    output.writeln(unexpectedError);
    output.writeln(unexpectedStackTrace);
    output.writeln('--- TEST FAILED ---');
  }
  await web.window
      .fetch('/test-result'.toJS, web.RequestInit(method: 'POST', body: '$output'.toJS))
      .toDart;
  print(output);
}

@noInline
void foo() {
  bar();
}

@noInline
void bar() {
  baz();
}

@noInline
void baz() {
  throw Exception('Test error message');
}

void _checkStackFrameContents(
  List<StackFrame> parsedFrames,
  List<StackFrame> expectedFrames,
  dynamic stackTrace,
) {
  // Filter out stack frames outside this library so this test is less brittle.
  final List<StackFrame> actual =
      parsedFrames.where((StackFrame frame) => callChain.contains(frame.method)).toList();
  final bool stackFramesAsExpected = ListEquality<StackFrame>(
    StackFrameEquality(),
  ).equals(actual, expectedFrames);
  if (!stackFramesAsExpected) {
    throw Exception(
      'Stack frames parsed incorrectly:\n'
      'Expected:\n${expectedFrames.join('\n')}\n'
      'Actual:\n${actual.join('\n')}\n'
      'Stack trace:\n$stackTrace',
    );
  }
}

/// Use custom equality to ignore [StackFrame.source], which is not important
/// for the purposes of this test.
class StackFrameEquality implements Equality<StackFrame> {
  @override
  bool equals(StackFrame e1, StackFrame e2) {
    return e1.number == e2.number &&
        e1.packageScheme == e2.packageScheme &&
        e1.package == e2.package &&
        e1.packagePath == e2.packagePath &&
        e1.line == e2.line &&
        e1.column == e2.column &&
        e1.className == e2.className &&
        e1.method == e2.method;
  }

  @override
  int hash(StackFrame e) {
    return Object.hash(
      e.number,
      e.packageScheme,
      e.package,
      e.packagePath,
      e.line,
      e.column,
      e.className,
      e.method,
    );
  }

  @override
  bool isValidKey(Object? o) => o is StackFrame;
}
