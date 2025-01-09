// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io';

import 'package:engine_tool/src/commands/command_runner.dart';
import 'package:engine_tool/src/logger.dart';
import 'package:path/path.dart' as p;
import 'package:test/test.dart';

import '../src/matchers.dart';
import '../src/utils.dart';

void main() {
  test('prints instead of deleting when --dry-run', () async {
    final fakeNow = DateTime.now().add(const Duration(days: 100));
    final testEnv = TestEnvironment.withTestEngine(now: () => fakeNow);
    addTearDown(testEnv.cleanup);

    final targetDir = Directory(p.join(testEnv.environment.engine.outDir.path, 'host_old'));
    await targetDir.create(recursive: true);

    final runner = ToolCommandRunner(environment: testEnv.environment, configs: {});

    final result = await runner.run(['cleanup', '--dry-run']);
    expect(result, 0);

    expect(targetDir.existsSync(), true, reason: '--dry-run should not delete directories');

    expect(
      testEnv.testLogs,
      containsAllInOrder([
        logRecord(contains('Checking ${testEnv.environment.engine.outDir.path}')),
        logRecord(contains('The following directories were accessed later than')),
        logRecord(contains('host_old')),
      ]),
    );
  });

  test('uses 30 days if --untouched-since is omitted', () async {
    final fakeNow = DateTime.now().add(const Duration(days: 32));
    final testEnv = TestEnvironment.withTestEngine(now: () => fakeNow);
    addTearDown(testEnv.cleanup);

    final targetDir = Directory(p.join(testEnv.environment.engine.outDir.path, 'host_old'));
    await targetDir.create(recursive: true);

    final runner = ToolCommandRunner(environment: testEnv.environment, configs: {});

    final result = await runner.run(['cleanup']);
    expect(result, 0);

    expect(targetDir.existsSync(), false, reason: 'Should be > 30 days since accessed');

    expect(
      testEnv.testLogs,
      containsAllInOrder([
        logRecord(contains('Checking ${testEnv.environment.engine.outDir.path}')),
        logRecord(contains('Deleted 1 output directories')),
      ]),
    );
  });

  test('parses the provided YYYY-MM-DD', () async {
    final fakeNow = DateTime.now();
    final testEnv = TestEnvironment.withTestEngine(now: () => fakeNow);
    addTearDown(testEnv.cleanup);

    final targetDir = Directory(p.join(testEnv.environment.engine.outDir.path, 'host_old'));
    await targetDir.create(recursive: true);

    final runner = ToolCommandRunner(environment: testEnv.environment, configs: {});

    final result = await runner.run(['cleanup', '--untouched-since=${fakeNow.year + 1}-01-01']);
    expect(result, 0);

    expect(
      targetDir.existsSync(),
      false,
      reason: 'Due to --untouched-since being a year in the future',
    );

    expect(
      testEnv.testLogs,
      containsAllInOrder([
        logRecord(contains('Checking ${testEnv.environment.engine.outDir.path}')),
        logRecord(contains('Deleted 1 output directories')),
      ]),
    );
  });

  test('refuses an invalid YYYY-MM-DD date', () async {
    final testEnv = TestEnvironment.withTestEngine();
    addTearDown(testEnv.cleanup);

    final runner = ToolCommandRunner(environment: testEnv.environment, configs: {});

    expect(runner.run(['cleanup', '--untouched-since=02-14-2024']), throwsA(isA<FatalError>()));
  });
}
