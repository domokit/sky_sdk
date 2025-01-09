// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:test/test.dart';

import 'src/utils.dart';

void main() async {
  final cannedProcesses = <CannedProcess>[
    CannedProcess((command) => command.contains('ulfuls'), stdout: 'Ashita ga aru sa'),
    CannedProcess((command) => command.contains('quruli'), stdout: 'Tokyo'),
    CannedProcess((command) => command.contains('elizaveta'), stdout: 'Moshimo ano toki'),
    CannedProcess((command) => command.contains('scott_murphy'), stdout: 'Donna toki mo'),
  ];

  test('containsCommand passes if command matched', () async {
    final testEnvironment = TestEnvironment.withTestEngine(cannedProcesses: cannedProcesses);
    addTearDown(testEnvironment.cleanup);

    await testEnvironment.environment.processRunner.runProcess(
      ['ulfuls', '--lyrics'],
      workingDirectory: testEnvironment.environment.engine.srcDir,
      failOk: true,
    );
    await testEnvironment.environment.processRunner.runProcess(
      ['quruli', '--lyrics'],
      workingDirectory: testEnvironment.environment.engine.srcDir,
      failOk: true,
    );
    final history = testEnvironment.processHistory;
    expect(
      history,
      containsCommand((command) {
        return command.isNotEmpty && command[0] == 'quruli';
      }),
    );
    expect(
      history,
      containsCommand((command) {
        return command.length > 1 && command[1] == '--lyrics';
      }),
    );
  });

  test('doesNotContainCommand passes if command not matched', () async {
    final testEnvironment = TestEnvironment.withTestEngine(cannedProcesses: cannedProcesses);
    addTearDown(testEnvironment.cleanup);

    await testEnvironment.environment.processRunner.runProcess(
      ['elizaveta', '--lyrics'],
      workingDirectory: testEnvironment.environment.engine.srcDir,
      failOk: true,
    );
    await testEnvironment.environment.processRunner.runProcess(
      ['scott_murphy', '--lyrics'],
      workingDirectory: testEnvironment.environment.engine.srcDir,
      failOk: true,
    );
    final history = testEnvironment.processHistory;
    expect(
      history,
      doesNotContainCommand((command) {
        return command.length > 1 && command[1] == '--not-an-option';
      }),
    );
  });
}
