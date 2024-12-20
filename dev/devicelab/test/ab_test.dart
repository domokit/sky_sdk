// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_devicelab/framework/ab.dart';
import 'package:flutter_devicelab/framework/task_result.dart';

import 'common.dart';

void main() {
  test('ABTest', () {
    final ABTest ab = ABTest(localEngine: 'engine', localEngineHost: 'engine', taskName: 'test');

    for (int i = 0; i < 5; i++) {
      final TaskResult aResult = TaskResult.fromJson(<String, dynamic>{
        'success': true,
        'data': <String, dynamic>{'i': i, 'j': 10 * i, 'not_a_metric': 'something'},
        'benchmarkScoreKeys': <String>['i', 'j'],
      });
      ab.addAResult(aResult);
      final TaskResult bResult = TaskResult.fromJson(<String, dynamic>{
        'success': true,
        'data': <String, dynamic>{'i': i + 1, 'k': 10 * i + 1},
        'benchmarkScoreKeys': <String>['i', 'k'],
      });
      ab.addBResult(bResult);
    }
    ab.finalize();

    expect(
      ab.rawResults(),
      'i:\n'
      '  A:\t0.00\t1.00\t2.00\t3.00\t4.00\t\n'
      '  B:\t1.00\t2.00\t3.00\t4.00\t5.00\t\n'
      'j:\n'
      '  A:\t0.00\t10.00\t20.00\t30.00\t40.00\t\n'
      '  B:\tN/A\n'
      'k:\n'
      '  A:\tN/A\n'
      '  B:\t1.00\t11.00\t21.00\t31.00\t41.00\t\n',
    );
    expect(
      ab.printSummary(),
      'Score\tAverage A (noise)\tAverage B (noise)\tSpeed-up\n'
      'i\t2.00 (70.71%)\t3.00 (47.14%)\t0.67x\t\n'
      'j\t20.00 (70.71%)\t\t\n'
      'k\t\t21.00 (67.34%)\t\n',
    );
  });
}
