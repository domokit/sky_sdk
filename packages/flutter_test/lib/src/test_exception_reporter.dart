// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:stack_trace/stack_trace.dart' as stack_trace;
import 'package:test_api/scaffolding.dart' as test_package;

/// Signature for the [reportTestException] callback.
typedef TestExceptionReporter = void Function(FlutterErrorDetails details, String testDescription);

/// A function that is called by the test framework when an unexpected error
/// occurred during a test.
///
/// This function is responsible for reporting the error to the user such that
/// the user can easily diagnose what failed when inspecting the test results.
/// It is also responsible for reporting the error to the test framework itself
/// in order to cause the test to fail.
///
/// This function is pluggable to handle the cases where tests are run in
/// contexts _other_ than via `flutter test`.
TestExceptionReporter get reportTestException => _reportTestException;
TestExceptionReporter _reportTestException = _defaultTestExceptionReporter;
set reportTestException(TestExceptionReporter handler) {
  _reportTestException = handler;
}

void _defaultTestExceptionReporter(FlutterErrorDetails errorDetails, String testDescription) {
  FlutterError.dumpErrorToConsole(errorDetails, forceReport: true);
  // test_package.registerException actually just calls the current zone's error handler (that
  // is to say, _parentZone's handleUncaughtError function). FakeAsync doesn't add one of those,
  // but the test package does, that's how the test package tracks errors. So really we could
  // get the same effect here by calling that error handler directly or indeed just throwing.
  // However, we call registerException because that's the semantically correct thing...
  String additional = '';
  if (testDescription.isNotEmpty) {
    additional = '\nThe test description was: $testDescription';
  }
  test_package.registerException(
    'Test failed. See exception logs above.$additional',
    _emptyStackTrace,
  );
}

final StackTrace _emptyStackTrace = stack_trace.Chain(const <stack_trace.Trace>[]);
