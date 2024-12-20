// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io' hide Platform;

import 'package:collection/collection.dart';
import 'package:file/file.dart' as fs;
import 'package:file/memory.dart';
import 'package:path/path.dart' as path;
import 'package:process/process.dart';

import '../suite_runners/run_flutter_packages_tests.dart';
import '../utils.dart';
import 'common.dart';

/// Fails a test if the exit code of `result` is not the expected value. This
/// is favored over `expect(result.exitCode, expectedExitCode)` because this
/// will include the process result's stdio in the failure message.
void expectExitCode(ProcessResult result, int expectedExitCode) {
  if (result.exitCode != expectedExitCode) {
    fail(
      'Process ${result.pid} exited with the wrong exit code.\n'
      '\n'
      'EXPECTED: exit code $expectedExitCode\n'
      'ACTUAL: exit code ${result.exitCode}\n'
      '\n'
      'STDOUT:\n'
      '${result.stdout}\n'
      'STDERR:\n'
      '${result.stderr}',
    );
  }
}

void main() {
  group('verifyVersion()', () {
    late MemoryFileSystem fileSystem;

    setUp(() {
      fileSystem = MemoryFileSystem.test();
    });

    test('passes for valid version strings', () async {
      const List<String> valid_versions = <String>[
        '1.2.3',
        '12.34.56',
        '1.2.3.pre.1',
        '1.2.3-4.5.pre',
        '1.2.3-5.0.pre.12',
      ];
      for (final String version in valid_versions) {
        final File file = fileSystem.file('version');
        file.writeAsStringSync(version);

        expect(
          await verifyVersion(file),
          isNull,
          reason: '$version is valid but verifyVersionFile said it was bad',
        );
      }
    });

    test('fails for invalid version strings', () async {
      const List<String> invalid_versions = <String>[
        '1.2.3.4',
        '1.2.3.',
        '1.2.pre.1',
        '1.2.3-pre.1',
        '1.2.3-pre.1+hotfix.1',
        '  1.2.3',
        '1.2.3-hotfix.1',
      ];
      for (final String version in invalid_versions) {
        final File file = fileSystem.file('version');
        file.writeAsStringSync(version);

        expect(
          await verifyVersion(file),
          'The version logic generated an invalid version string: "$version".',
          reason: '$version is invalid but verifyVersionFile said it was fine',
        );
      }
    });
  });

  group('flutter/packages version', () {
    final MemoryFileSystem memoryFileSystem = MemoryFileSystem();
    final fs.File packagesVersionFile = memoryFileSystem.file(
      path.join('bin', 'internal', 'flutter_packages.version'),
    );
    const String kSampleHash = '592b5b27431689336fa4c721a099eedf787aeb56';
    setUpAll(() {
      packagesVersionFile.createSync(recursive: true);
    });

    test('commit hash', () async {
      packagesVersionFile.writeAsStringSync(kSampleHash);
      final String actualHash = await getFlutterPackagesVersion(
        flutterRoot: flutterRoot,
        fileSystem: memoryFileSystem,
        packagesVersionFile: packagesVersionFile.path,
      );
      expect(actualHash, kSampleHash);
    });

    test('commit hash with newlines', () async {
      packagesVersionFile.writeAsStringSync('\n$kSampleHash\n');
      final String actualHash = await getFlutterPackagesVersion(
        flutterRoot: flutterRoot,
        fileSystem: memoryFileSystem,
        packagesVersionFile: packagesVersionFile.path,
      );
      expect(actualHash, kSampleHash);
    });
  });

  group('test.dart script', () {
    const ProcessManager processManager = LocalProcessManager();

    Future<ProcessResult> runScript([
      Map<String, String>? environment,
      List<String> otherArgs = const <String>[],
    ]) async {
      final String dart = path.absolute(
        path.join('..', '..', 'bin', 'cache', 'dart-sdk', 'bin', 'dart'),
      );
      final ProcessResult scriptProcess = processManager.runSync(<String>[
        dart,
        'test.dart',
        ...otherArgs,
      ], environment: environment);
      return scriptProcess;
    }

    test('subshards tests correctly', () async {
      // When updating this test, try to pick shard numbers that ensure we're checking
      // that unequal test distributions don't miss tests.
      ProcessResult result = await runScript(<String, String>{
        'SHARD': kTestHarnessShardName,
        'SUBSHARD': '1_3',
      });
      expectExitCode(result, 0);
      expect(result.stdout, contains('Selecting subshard 1 of 3 (tests 1-3 of 9)'));

      result = await runScript(<String, String>{'SHARD': kTestHarnessShardName, 'SUBSHARD': '3_3'});
      expectExitCode(result, 0);
      expect(result.stdout, contains('Selecting subshard 3 of 3 (tests 7-9 of 9)'));
    });

    test('exits with code 1 when SUBSHARD index greater than total', () async {
      final ProcessResult result = await runScript(<String, String>{
        'SHARD': kTestHarnessShardName,
        'SUBSHARD': '100_99',
      });
      expectExitCode(result, 1);
      expect(result.stdout, contains('Invalid subshard name'));
    });

    test('exits with code 255 when invalid SUBSHARD name', () async {
      final ProcessResult result = await runScript(<String, String>{
        'SHARD': kTestHarnessShardName,
        'SUBSHARD': 'invalid_name',
      });
      expectExitCode(result, 255);
      expect(result.stdout, contains('Invalid subshard name'));
    });

    test('--dry-run prints every test that would run', () async {
      final ProcessResult result = await runScript(<String, String>{}, <String>['--dry-run']);
      expectExitCode(result, 0);
      expect(result.stdout, contains('|> bin/flutter'));
    }, testOn: 'posix');
  });

  test('selectTestsForSubShard distributes tests amongst subshards correctly', () async {
    List<int> makeTests(int count) => List<int>.generate(count, (int index) => index);

    void testSubsharding(int testCount, int subshardCount) {
      String failureReason(String reason) {
        return 'Subsharding test failed for testCount=$testCount, subshardCount=$subshardCount.\n'
            '$reason';
      }

      final List<int> tests = makeTests(testCount);
      final List<List<int>> subshards = List<List<int>>.generate(subshardCount, (int index) {
        final int subShardIndex = index + 1;
        final (int start, int end) = selectTestsForSubShard(
          testCount: tests.length,
          subShardIndex: subShardIndex,
          subShardCount: subshardCount,
        );
        return tests.sublist(start, end);
      });

      final List<int> testedTests = subshards.flattened.toList();
      final Set<int> deduped = Set<int>.from(subshards.flattened);
      expect(
        testedTests,
        hasLength(deduped.length),
        reason: failureReason('Subshards may have had duplicate tests.'),
      );
      expect(
        testedTests,
        unorderedEquals(tests),
        reason: failureReason('One or more tests were not assigned to a subshard.'),
      );

      final int minimumTestsPerShard = (testCount / subshardCount).floor();
      for (int i = 0; i < subshards.length; i++) {
        final int extraTestsInThisShard = subshards[i].length - minimumTestsPerShard;
        expect(
          extraTestsInThisShard,
          isNonNegative,
          reason: failureReason(
            'Subsharding uneven. Subshard ${i + 1} had too few tests: ${subshards[i].length}',
          ),
        );
        expect(
          extraTestsInThisShard,
          lessThanOrEqualTo(1),
          reason: failureReason(
            'Subsharding uneven. Subshard ${i + 1} had too many tests: ${subshards[i].length}',
          ),
        );
      }
    }

    testSubsharding(9, 3);
    testSubsharding(25, 8);
    testSubsharding(30, 15);
  });
}
