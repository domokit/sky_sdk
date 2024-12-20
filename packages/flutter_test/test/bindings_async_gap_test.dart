// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:flutter/foundation.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:stack_trace/stack_trace.dart' as stack_trace;

Future<void> main() async {
  test('demangles stacks', () async {
    // Test that the tester bindings unmangle stacks that come in as
    // package:stack_trace types.
    // Uses runTest directly so that the test does not get hung up waiting for
    // the error reporter to be reset to the original one.

    final Completer<FlutterErrorDetails> errorCompleter = Completer<FlutterErrorDetails>();
    final TestExceptionReporter oldReporter = reportTestException;
    reportTestException = (FlutterErrorDetails details, String testDescription) {
      errorCompleter.complete(details);
      reportTestException = oldReporter;
    };

    final AutomatedTestWidgetsFlutterBinding binding = AutomatedTestWidgetsFlutterBinding();
    await binding.runTest(() async {
      final Completer<String> completer = Completer<String>();

      completer.future.then(
        (String value) {},
        onError: (Object error, StackTrace stack) {
          assert(stack is stack_trace.Chain);
          FlutterError.reportError(FlutterErrorDetails(exception: error, stack: stack));
        },
      );

      completer.completeError(const CustomException());
    }, () {});

    final FlutterErrorDetails details = await errorCompleter.future;
    expect(details, isNotNull);
    expect(details.exception, isA<CustomException>());
    reportTestException = oldReporter;
  });
}

class CustomException implements Exception {
  const CustomException();
}
