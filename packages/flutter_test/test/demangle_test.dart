// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:flutter/foundation.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:stack_trace/stack_trace.dart' as stack_trace;

Future<void> main() async {
  // We use AutomatedTestWidgetsFlutterBinding to allow the test binding to set
  // FlutterError.demangleStackTrace and FlutterError.onError without testWidgets.
  final AutomatedTestWidgetsFlutterBinding binding = AutomatedTestWidgetsFlutterBinding();

  test('FlutterErrorDetails demangles', () async {
    await binding.runTest(() async {
      // When we call toString on a FlutterErrorDetails, it attempts to parse and
      // filter the stack trace, which fails if demangleStackTrace returns a
      // mangled stack trace.
      FlutterErrorDetails(
        exception: const CustomException(),
        stack: await getMangledStack(),
      ).toString();

      // Additional logic is used to parse assertion stack traces.
      FlutterErrorDetails(
        exception: AssertionError('Some assertion'),
        stack: await getMangledStack(),
      ).toString();
    }, () {});
    binding.postTest();
  });

  test('debugPrintStack demangles', () async {
    await binding.runTest(() async {
      final DebugPrintCallback oldDebugPrint = debugPrint;
      try {
        debugPrint = (String? message, {int? wrapWidth}) {};
        debugPrintStack(stackTrace: await getMangledStack());
      } finally {
        debugPrint = oldDebugPrint;
      }
    }, () {});
    binding.postTest();
  });
}

Future<StackTrace> getMangledStack() {
  // package:test uses package:stack_trace to wrap tests in a Zone that overrides
  // errorCallback, the error callback transforms any StackTrace propagated
  // to futures into a Chain, which has a format different from the vm.
  final Completer<StackTrace> stackCompleter = Completer<StackTrace>();
  final Completer<void> completer = Completer<void>();
  completer.future.then(
    (void value) {
      assert(false);
    },
    onError: (Object error, StackTrace stack) {
      expect(error, isA<CustomException>());
      expect(stack, isA<stack_trace.Chain>());
      stackCompleter.complete(stack);
    },
  );
  completer.completeError(const CustomException());
  return stackCompleter.future;
}

class CustomException implements Exception {
  const CustomException();
}
