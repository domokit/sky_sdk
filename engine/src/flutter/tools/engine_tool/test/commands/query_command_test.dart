// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert';
import 'dart:ffi' show Abi;

import 'package:engine_tool/src/commands/command_runner.dart';
import 'package:logging/logging.dart' as log;
import 'package:test/test.dart';

import '../src/test_build_configs.dart';
import '../src/utils.dart';

void main() {
  List<String> stringsFromLogs(List<log.LogRecord> logs) {
    return logs.map((log.LogRecord r) => r.message).toList();
  }

  test('query command returns builds for the host platform.', () async {
    final testEnvironment = TestEnvironment.withTestEngine(
      // Intentionally use the default parameter to make it explicit.
      // ignore: avoid_redundant_argument_values
      abi: Abi.linuxX64,
    );
    addTearDown(testEnvironment.cleanup);

    final linuxBuilders1 = TestBuilderConfig();
    linuxBuilders1.addBuild(name: 'ci/build_name', dimension: TestDroneDimension.linux);
    linuxBuilders1.addBuild(name: 'linux/host_debug', dimension: TestDroneDimension.linux);
    linuxBuilders1.addBuild(name: 'linux/android_debug_arm64', dimension: TestDroneDimension.linux);
    linuxBuilders1.addBuild(
      name: 'ci/android_debug_rbe_arm64',
      dimension: TestDroneDimension.linux,
    );

    final linuxBuilders2 = TestBuilderConfig();
    linuxBuilders2.addBuild(name: 'ci/build_name2', dimension: TestDroneDimension.linux);
    linuxBuilders2.addBuild(name: 'linux/host_debug2', dimension: TestDroneDimension.linux);
    linuxBuilders2.addBuild(
      name: 'linux/android_debug2_arm64',
      dimension: TestDroneDimension.linux,
    );
    linuxBuilders2.addBuild(
      name: 'ci/android_debug2_rbe_arm64',
      dimension: TestDroneDimension.linux,
    );

    final macOSBuilders = TestBuilderConfig();
    macOSBuilders.addBuild(name: 'ci/build_name', dimension: TestDroneDimension.mac);
    macOSBuilders.addBuild(name: 'mac/host_debug', dimension: TestDroneDimension.mac);
    macOSBuilders.addBuild(name: 'mac/android_debug_arm64', dimension: TestDroneDimension.mac);
    macOSBuilders.addBuild(name: 'ci/android_debug_rbe_arm64', dimension: TestDroneDimension.mac);

    final runner = ToolCommandRunner(
      environment: testEnvironment.environment,
      configs: {
        'linux_test_config': linuxBuilders1.buildConfig(path: 'ci/builders/linux_test_config.json'),
        'linux_test_config2': linuxBuilders2.buildConfig(
          path: 'ci/builders/linux_test_config2.json',
        ),
        'mac_test_config': macOSBuilders.buildConfig(path: 'ci/builders/mac_test_config.json'),
      },
    );
    final result = await runner.run(<String>['query', 'builders']);

    printOnFailure(testEnvironment.testLogs.map((r) => r.message).join('\n'));
    expect(result, equals(0));
    expect(
      stringsFromLogs(testEnvironment.testLogs),
      equals(<String>[
        'Add --verbose to see detailed information about each builder\n',
        '\n',
        '"linux_test_config" builder:\n',
        '   "ci/build_name" config\n',
        '   "linux/host_debug" config\n',
        '   "linux/android_debug_arm64" config\n',
        '   "ci/android_debug_rbe_arm64" config\n',
        '"linux_test_config2" builder:\n',
        '   "ci/build_name2" config\n',
        '   "linux/host_debug2" config\n',
        '   "linux/android_debug2_arm64" config\n',
        '   "ci/android_debug2_rbe_arm64" config\n',
      ]),
    );
  });

  test('query command returns only from the named builder.', () async {
    final testEnvironment = TestEnvironment.withTestEngine(
      // Intentionally use the default parameter to make it explicit.
      // ignore: avoid_redundant_argument_values
      abi: Abi.linuxX64,
    );
    addTearDown(testEnvironment.cleanup);

    final linuxBuilders1 = TestBuilderConfig();
    linuxBuilders1.addBuild(name: 'ci/build_name', dimension: TestDroneDimension.linux);
    linuxBuilders1.addBuild(name: 'linux/host_debug', dimension: TestDroneDimension.linux);
    linuxBuilders1.addBuild(name: 'linux/android_debug_arm64', dimension: TestDroneDimension.linux);
    linuxBuilders1.addBuild(
      name: 'ci/android_debug_rbe_arm64',
      dimension: TestDroneDimension.linux,
    );

    final linuxBuilders2 = TestBuilderConfig();
    linuxBuilders2.addBuild(name: 'ci/build_name2', dimension: TestDroneDimension.linux);
    linuxBuilders2.addBuild(name: 'linux/host_debug2', dimension: TestDroneDimension.linux);
    linuxBuilders2.addBuild(
      name: 'linux/android_debug2_arm64',
      dimension: TestDroneDimension.linux,
    );
    linuxBuilders2.addBuild(
      name: 'ci/android_debug2_rbe_arm64',
      dimension: TestDroneDimension.linux,
    );

    final runner = ToolCommandRunner(
      environment: testEnvironment.environment,
      configs: {
        'linux_test_config': linuxBuilders1.buildConfig(path: 'ci/builders/linux_test_config.json'),
        'linux_test_config2': linuxBuilders2.buildConfig(
          path: 'ci/builders/linux_test_config2.json',
        ),
      },
    );
    final result = await runner.run(<String>[
      'query',
      'builders',
      '--builder',
      'linux_test_config',
    ]);

    printOnFailure(testEnvironment.testLogs.map((r) => r.message).join('\n'));
    expect(result, equals(0));
    expect(
      stringsFromLogs(testEnvironment.testLogs),
      equals(<String>[
        'Add --verbose to see detailed information about each builder\n',
        '\n',
        '"linux_test_config" builder:\n',
        '   "ci/build_name" config\n',
        '   "linux/host_debug" config\n',
        '   "linux/android_debug_arm64" config\n',
        '   "ci/android_debug_rbe_arm64" config\n',
      ]),
    );
  });

  test('query command with --all returns all builds.', () async {
    final testEnvironment = TestEnvironment.withTestEngine();
    addTearDown(testEnvironment.cleanup);

    final linuxBuilders1 = TestBuilderConfig();
    linuxBuilders1.addBuild(name: 'ci/build_name', dimension: TestDroneDimension.linux);
    linuxBuilders1.addBuild(name: 'linux/host_debug', dimension: TestDroneDimension.linux);
    linuxBuilders1.addBuild(name: 'linux/android_debug_arm64', dimension: TestDroneDimension.linux);
    linuxBuilders1.addBuild(
      name: 'ci/android_debug_rbe_arm64',
      dimension: TestDroneDimension.linux,
    );

    final linuxBuilders2 = TestBuilderConfig();
    linuxBuilders2.addBuild(name: 'ci/build_name2', dimension: TestDroneDimension.linux);
    linuxBuilders2.addBuild(name: 'linux/host_debug2', dimension: TestDroneDimension.linux);
    linuxBuilders2.addBuild(
      name: 'linux/android_debug2_arm64',
      dimension: TestDroneDimension.linux,
    );
    linuxBuilders2.addBuild(
      name: 'ci/android_debug2_rbe_arm64',
      dimension: TestDroneDimension.linux,
    );

    final macOSBuilders = TestBuilderConfig();
    macOSBuilders.addBuild(name: 'ci/build_name', dimension: TestDroneDimension.mac);
    macOSBuilders.addBuild(name: 'mac/host_debug', dimension: TestDroneDimension.mac);
    macOSBuilders.addBuild(name: 'mac/android_debug_arm64', dimension: TestDroneDimension.mac);
    macOSBuilders.addBuild(name: 'ci/android_debug_rbe_arm64', dimension: TestDroneDimension.mac);

    final runner = ToolCommandRunner(
      environment: testEnvironment.environment,
      configs: {
        'linux_test_config': linuxBuilders1.buildConfig(path: 'ci/builders/linux_test_config.json'),
        'linux_test_config2': linuxBuilders2.buildConfig(
          path: 'ci/builders/linux_test_config2.json',
        ),
        'mac_test_config': macOSBuilders.buildConfig(path: 'ci/builders/mac_test_config.json'),
      },
    );
    final result = await runner.run(<String>['query', 'builders', '--all']);

    printOnFailure(testEnvironment.testLogs.map((r) => r.message).join('\n'));
    expect(result, equals(0));
    expect(
      stringsFromLogs(testEnvironment.testLogs),
      equals(<String>[
        'Add --verbose to see detailed information about each builder\n',
        '\n',
        '"linux_test_config" builder:\n',
        '   "ci/build_name" config\n',
        '   "linux/host_debug" config\n',
        '   "linux/android_debug_arm64" config\n',
        '   "ci/android_debug_rbe_arm64" config\n',
        '"linux_test_config2" builder:\n',
        '   "ci/build_name2" config\n',
        '   "linux/host_debug2" config\n',
        '   "linux/android_debug2_arm64" config\n',
        '   "ci/android_debug2_rbe_arm64" config\n',
        '"mac_test_config" builder:\n',
        '   "ci/build_name" config\n',
        '   "mac/host_debug" config\n',
        '   "mac/android_debug_arm64" config\n',
        '   "ci/android_debug_rbe_arm64" config\n',
      ]),
    );
  });

  test('query targets', () async {
    final testEnvironment = TestEnvironment.withTestEngine(
      cannedProcesses: [
        CannedProcess(
          (command) => command.contains('desc'),
          stdout: jsonEncode({
            '//flutter/display_list:display_list_unittests': {
              'outputs': ['//out/host_debug/display_list_unittests'],
              'testonly': true,
              'type': 'executable',
            },
            '//flutter/flow:flow_unittests': {
              'outputs': ['//out/host_debug/flow_unittests'],
              'testonly': true,
              'type': 'executable',
            },
            '//flutter/fml:fml_unittests': {
              'outputs': ['//out/host_debug/fml_unittests'],
              'testonly': true,
              'type': 'executable',
            },
          }),
        ),
      ],
    );
    addTearDown(testEnvironment.cleanup);

    final linuxBuilders1 = TestBuilderConfig();
    linuxBuilders1.addBuild(
      name: 'ci/build_name',
      dimension: TestDroneDimension.linux,
      targetDir: 'host_debug',
    );
    linuxBuilders1.addBuild(
      name: 'linux/host_debug',
      dimension: TestDroneDimension.linux,
      targetDir: 'host_debug',
    );
    linuxBuilders1.addBuild(
      name: 'linux/android_debug_arm64',
      dimension: TestDroneDimension.linux,
      targetDir: 'android_debug_arm64',
    );
    linuxBuilders1.addBuild(
      name: 'ci/android_debug_rbe_arm64',
      dimension: TestDroneDimension.linux,
      targetDir: 'android_debug_arm64',
    );

    final runner = ToolCommandRunner(
      environment: testEnvironment.environment,
      configs: {
        'linux_test_config': linuxBuilders1.buildConfig(path: 'ci/builders/linux_test_config.json'),
      },
    );
    final result = await runner.run(<String>['query', 'targets']);

    printOnFailure(testEnvironment.testLogs.map((r) => r.message).join('\n'));
    expect(result, equals(0));

    final expected = <String>[
      '//flutter/display_list:display_list_unittests',
      '//flutter/flow:flow_unittest',
      '//flutter/fml:fml_unittests',
    ];

    final testLogs = stringsFromLogs(testEnvironment.testLogs);
    for (final testLog in testLogs) {
      // Expect one of the expected targets to be in the output.
      // Then remove it from the list of expected targets.
      for (final target in expected) {
        if (testLog.contains(target)) {
          expected.remove(target);
          break;
        }
      }
    }

    expect(expected.isEmpty, isTrue, reason: 'All expected targets were found');
  });
}
