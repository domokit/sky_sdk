// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_devicelab/framework/runner.dart';
import 'package:vm_service/vm_service.dart';

import 'common.dart';

void main() {
  final Map<String, String> isolateParams = <String, String>{
    'runFlutterConfig': 'false',
    'timeoutInMinutes': '1',
  };
  late List<String> printLog;
  void print(String s) => printLog.add(s);

  group('run.dart script', () {
    test('Reruns - Test passes the first time.', () async {
      printLog = <String>[];
      await runTasks(
        <String>['smoke_test_success'],
        isolateParams: isolateParams,
        print: print,
        logs: printLog,
      );
      expect(printLog.length, 2);
      expect(printLog[0], 'Test passed on first attempt.');
      expect(printLog[1], 'flaky: false');
    });

    test('Reruns - Test fails all reruns.', () async {
      printLog = <String>[];
      await runTasks(
        <String>['smoke_test_failure'],
        isolateParams: isolateParams,
        print: print,
        logs: printLog,
      );
      expect(printLog.length, 2);
      expect(printLog[0], 'Consistently failed across all 3 executions.');
      expect(printLog[1], 'flaky: false');
    });

    test(
      'Ensures task results are received before task process shuts down.',
      () async {
        // Regression test for https://github.com/flutter/flutter/issues/155475
        //
        // Runs multiple concurrent instances of a short-lived task in an effort to
        // trigger the race between the VM service processing the response from
        // ext.cocoonRunTask and the VM shutting down, which will throw a RPCError
        // with a "Service connection disposed" message.
        //
        // Obviously this isn't foolproof, but this test becoming flaky or failing
        // consistently should signal that we're encountering a shutdown race
        // somewhere.
        const int runs = 30;
        try {
          await Future.wait(<Future<void>>[
            for (int i = 0; i < runs; ++i)
              runTasks(<String>['smoke_test_success'], isolateParams: isolateParams),
          ], eagerError: true);
        } on RPCError catch (e) {
          fail('Unexpected RPCError: $e');
        }
      },
      timeout: const Timeout.factor(2),
    );
  });
}
