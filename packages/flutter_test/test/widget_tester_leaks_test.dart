// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:leak_tracker_flutter_testing/leak_tracker_flutter_testing.dart';

import 'utils/memory_leak_tests.dart';

class _TestExecution {
  _TestExecution({required this.settings, required this.settingName, required this.test});

  final String settingName;
  final LeakTesting settings;
  final LeakTestCase test;

  String get name => '${test.name}, $settingName';
}

final List<_TestExecution> _testExecutions = <_TestExecution>[];

void main() {
  LeakTesting.collectedLeaksReporter = _verifyLeaks;
  LeakTesting.enable();

  LeakTesting.settings = LeakTesting.settings
      .withTrackedAll()
      .withTracked(allNotDisposed: true, experimentalAllNotGCed: true)
      .withIgnored(
        createdByTestHelpers: true,
        testHelperExceptions: <RegExp>[RegExp(RegExp.escape(memoryLeakTestsFilePath()))],
      );

  for (final LeakTestCase test in memoryLeakTests) {
    for (final MapEntry<String, LeakTesting Function(LeakTesting settings)> settingsCase
        in leakTestingSettingsCases.entries) {
      final LeakTesting settings = settingsCase.value(LeakTesting.settings);
      if (settings.leakDiagnosticConfig.collectRetainingPathForNotGCed) {
        // Retaining path requires vm to be started, so skipping.
        continue;
      }
      final _TestExecution execution = _TestExecution(
        settingName: settingsCase.key,
        test: test,
        settings: settings,
      );
      _testExecutions.add(execution);
      testWidgets(execution.name, experimentalLeakTesting: settings, (WidgetTester tester) async {
        await test.body(
          (Widget widget, [Duration? duration]) => tester.pumpWidget(widget, duration: duration),
          tester.runAsync,
        );
      });
    }
  }
}

void _verifyLeaks(Leaks leaks) {
  for (final _TestExecution execution in _testExecutions) {
    final Leaks testLeaks = leaks.byPhase[execution.name] ?? Leaks.empty();
    execution.test.verifyLeaks(testLeaks, execution.settings, testDescription: execution.name);
  }
}
