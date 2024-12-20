// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert' as convert;
import 'dart:ffi';

import 'package:engine_tool/src/build_utils.dart';
import 'package:engine_tool/src/commands/command_runner.dart';
import 'package:engine_tool/src/logger.dart';
import 'package:path/path.dart' as path;
import 'package:test/test.dart';

import '../src/matchers.dart';
import '../src/test_build_configs.dart';
import '../src/utils.dart';

void main() {
  test('build command invokes gn', () async {
    final testEnv = TestEnvironment.withTestEngine(abi: Abi.macosArm64);
    addTearDown(testEnv.cleanup);

    final builder = TestBuilderConfig();
    builder.addBuild(name: 'macos/host_debug', dimension: TestDroneDimension.mac);

    final configs = {
      'mac_test_config': builder.buildConfig(path: 'ci/builders/mac_test_config.json'),
    };

    final runner = ToolCommandRunner(environment: testEnv.environment, configs: configs);
    final result = await runner.run(['build', '--config', 'host_debug']);

    printOnFailure(testEnv.testLogs.map((r) => r.message).join('\n'));
    expect(result, equals(0));
    expect(testEnv.processHistory.length, greaterThanOrEqualTo(1));
    expect(testEnv.processHistory[0].command[0], contains('gn'));
  });

  test('build command invokes ninja', () async {
    final testEnv = TestEnvironment.withTestEngine(abi: Abi.macosArm64);
    addTearDown(testEnv.cleanup);

    final builder = TestBuilderConfig();
    builder.addBuild(name: 'macos/host_debug', dimension: TestDroneDimension.mac);

    final configs = {
      'mac_test_config': builder.buildConfig(path: 'ci/builders/mac_test_config.json'),
    };
    final runner = ToolCommandRunner(environment: testEnv.environment, configs: configs);
    final result = await runner.run(['build', '--config', 'host_debug']);

    printOnFailure(testEnv.testLogs.map((r) => r.message).join('\n'));
    expect(result, equals(0));
    expect(testEnv.processHistory.length, greaterThanOrEqualTo(2));
    expect(testEnv.processHistory[1].command[0], contains('ninja'));
  });

  test('build command invokes generator', () async {
    final testEnv = TestEnvironment.withTestEngine(abi: Abi.macosArm64);
    addTearDown(testEnv.cleanup);

    final builder = TestBuilderConfig();
    builder.addBuild(
      name: 'macos/host_debug',
      dimension: TestDroneDimension.mac,
      generatorTask: ('gen/script.py', ['--test-param']),
    );

    final configs = {
      'mac_test_config': builder.buildConfig(path: 'ci/builders/mac_test_config.json'),
    };
    final runner = ToolCommandRunner(environment: testEnv.environment, configs: configs);
    final result = await runner.run(['build', '--config', 'host_debug']);

    printOnFailure(testEnv.testLogs.map((r) => r.message).join('\n'));
    expect(result, equals(0));
    expect(
      testEnv.processHistory.map((p) => p.command),
      containsOnce(containsAllInOrder(['python3', 'gen/script.py'])),
    );
  });

  test('build command does not invoke tests', () async {
    final testEnv = TestEnvironment.withTestEngine(abi: Abi.macosArm64);
    addTearDown(testEnv.cleanup);

    final builder = TestBuilderConfig();
    builder.addBuild(
      name: 'macos/host_debug',
      dimension: TestDroneDimension.mac,
      testTask: ('test/script.py', ['--test-param']),
    );

    final configs = {
      'mac_test_config': builder.buildConfig(path: 'ci/builders/mac_test_config.json'),
    };
    final runner = ToolCommandRunner(environment: testEnv.environment, configs: configs);
    final result = await runner.run(['build', '--config', 'host_debug']);

    printOnFailure(testEnv.testLogs.map((r) => r.message).join('\n'));
    expect(result, equals(0));
    expect(
      testEnv.processHistory.map((p) => p.command),
      isNot(contains(containsAllInOrder(['python3', 'gen/script.py']))),
    );
  });

  test('build command runs rbe on an rbe build', () async {
    final testEnv = TestEnvironment.withTestEngine(abi: Abi.macosArm64, withRbe: true);
    addTearDown(testEnv.cleanup);

    final builder = TestBuilderConfig();
    builder.addBuild(
      name: 'ci/android_debug_rbe_arm64',
      dimension: TestDroneDimension.mac,
      enableRbe: true,
    );
    final configs = {
      'mac_test_config': builder.buildConfig(path: 'ci/builders/mac_test_config.json'),
    };

    final runner = ToolCommandRunner(environment: testEnv.environment, configs: configs);
    final result = await runner.run(['build', '--config', 'ci/android_debug_rbe_arm64']);

    printOnFailure(testEnv.testLogs.map((r) => r.message).join('\n'));
    expect(result, equals(0));

    final [gnCall, reclientCall, ..._] = testEnv.processHistory;
    expect(gnCall.command, containsAllInOrder([endsWith('tools/gn'), contains('--rbe')]));
    expect(reclientCall.command, containsAllInOrder([endsWith('reclient/bootstrap')]));
  });

  test('build command plumbs -j to ninja', () async {
    final testEnv = TestEnvironment.withTestEngine(abi: Abi.macosArm64, withRbe: true);
    addTearDown(testEnv.cleanup);

    final builder = TestBuilderConfig();
    builder.addBuild(name: 'ci/android_debug_arm64', dimension: TestDroneDimension.mac);

    final runner = ToolCommandRunner(
      environment: testEnv.environment,
      configs: {'mac_test_config': builder.buildConfig(path: 'ci/builders/mac_test_config.json')},
    );
    final result = await runner.run(['build', '--config', 'ci/android_debug_arm64', '-j', '500']);

    printOnFailure(testEnv.testLogs.map((r) => r.message).join('\n'));
    expect(result, equals(0));

    print(testEnv.processHistory);
    final [_, ninja, ..._] = testEnv.processHistory;
    expect(ninja.command, containsAllInOrder([endsWith('ninja/ninja'), '-j', '500']));
  });

  test('build command does not run rbe when disabled', () async {
    final testEnv = TestEnvironment.withTestEngine(abi: Abi.macosArm64, withRbe: true);
    addTearDown(testEnv.cleanup);

    final builder = TestBuilderConfig();
    builder.addBuild(
      name: 'ci/android_debug_rbe_arm64',
      dimension: TestDroneDimension.mac,

      // Intentionally show that RBE is disabled.
      // ignore: avoid_redundant_argument_values
      enableRbe: false,
    );
    final runner = ToolCommandRunner(
      environment: testEnv.environment,
      configs: {'mac_test_config': builder.buildConfig(path: 'ci/builders/mac_test_config.json')},
    );
    final result = await runner.run([
      'build',
      '--config',
      'ci/android_debug_rbe_arm64',
      '--no-rbe',
    ]);

    printOnFailure(testEnv.testLogs.map((r) => r.message).join('\n'));
    expect(result, equals(0));

    final [gn, ninja, ..._] = testEnv.processHistory;
    expect(gn.command, isNot(contains('--rbe')));

    expect(ninja.command, containsAllInOrder([endsWith('ninja/ninja')]));
  });

  test('build command does not run rbe when rbe configs do not exist', () async {
    final testEnv = TestEnvironment.withTestEngine(abi: Abi.macosArm64);
    addTearDown(testEnv.cleanup);

    final builder = TestBuilderConfig();
    builder.addBuild(
      name: 'ci/android_debug_rbe_arm64',
      dimension: TestDroneDimension.mac,
      enableRbe: true,
    );
    final runner = ToolCommandRunner(
      environment: testEnv.environment,
      configs: {'mac_test_config': builder.buildConfig(path: 'ci/builders/mac_test_config.json')},
    );
    final result = await runner.run(['build', '--config', 'ci/android_debug_rbe_arm64']);

    printOnFailure(testEnv.testLogs.map((r) => r.message).join('\n'));
    expect(result, equals(0));

    final [gn, ninja, ..._] = testEnv.processHistory;
    expect(gn.command, isNot(contains('--rbe')));
    expect(ninja.command, containsAllInOrder([endsWith('ninja/ninja')]));
  });

  test('mangleConfigName removes the OS and adds ci/ as needed', () {
    final testEnv = TestEnvironment.withTestEngine();
    addTearDown(testEnv.cleanup);

    final env = testEnv.environment;
    expect(mangleConfigName(env, 'linux/build'), equals('build'));
    expect(mangleConfigName(env, 'ci/build'), equals('ci/build'));
  });

  test('mangleConfigName throws when the input config name is malformed', () {
    final testEnv = TestEnvironment.withTestEngine();
    addTearDown(testEnv.cleanup);

    final env = testEnv.environment;
    expect(() => mangleConfigName(env, 'build'), throwsArgumentError);
  });

  test('demangleConfigName adds the OS and removes ci/ as needed', () {
    final testEnv = TestEnvironment.withTestEngine();
    addTearDown(testEnv.cleanup);

    final env = testEnv.environment;
    expect(demangleConfigName(env, 'build'), equals('linux/build'));
    expect(demangleConfigName(env, 'ci/build'), equals('ci/build'));
  });

  test('local config name on the command line is correctly translated', () async {
    final builder = TestBuilderConfig();
    builder.addBuild(
      name: 'linux/host_debug',
      dimension: TestDroneDimension.linux,
      targetDir: 'local_host_debug',
    );
    builder.addBuild(
      name: 'ci/host_debug',
      dimension: TestDroneDimension.linux,
      targetDir: 'ci/host_debug',
    );

    final testEnv = TestEnvironment.withTestEngine();
    addTearDown(testEnv.cleanup);

    final runner = ToolCommandRunner(
      environment: testEnv.environment,
      configs: {
        'namespace_test_config': builder.buildConfig(
          path: 'ci/builders/namespace_test_config.json',
        ),
      },
    );
    final result = await runner.run(['build', '--config', 'host_debug']);
    expect(result, equals(0));
    expect(testEnv.processHistory[1].command[0], contains(path.join('ninja', 'ninja')));
    expect(testEnv.processHistory[1].command[2], contains('local_host_debug'));
  });

  test('ci config name on the command line is correctly translated', () async {
    final builder = TestBuilderConfig();
    builder.addBuild(
      name: 'linux/host_debug',
      dimension: TestDroneDimension.linux,
      targetDir: 'local_host_debug',
    );
    builder.addBuild(
      name: 'ci/host_debug',
      dimension: TestDroneDimension.linux,
      targetDir: 'ci/host_debug',
    );
    final testEnv = TestEnvironment.withTestEngine();
    addTearDown(testEnv.cleanup);

    final runner = ToolCommandRunner(
      environment: testEnv.environment,
      configs: {
        'namespace_test_config': builder.buildConfig(
          path: 'ci/builders/namespace_test_config.json',
        ),
      },
    );
    final result = await runner.run(['build', '--config', 'ci/host_debug']);
    expect(result, equals(0));
    expect(testEnv.processHistory[1].command[0], contains(path.join('ninja', 'ninja')));
    expect(testEnv.processHistory[1].command[2], contains('ci/host_debug'));
  });

  test('build command invokes ninja with the specified target', () async {
    final testEnv = TestEnvironment.withTestEngine(
      abi: Abi.macosArm64,
      cannedProcesses: [
        CannedProcess(
          (command) => command.contains('desc'),
          stdout: convert.jsonEncode({
            '//flutter/fml:fml_unittests': {
              'outputs': ['//out/host_debug/fml_unittests'],
              'testonly': true,
              'type': 'executable',
            },
          }),
        ),
      ],
    );
    addTearDown(testEnv.cleanup);

    final builder = TestBuilderConfig();
    builder.addBuild(
      name: 'ci/host_debug',
      targetDir: 'host_debug',
      dimension: TestDroneDimension.mac,
    );
    final runner = ToolCommandRunner(
      environment: testEnv.environment,
      configs: {'mac_test_config': builder.buildConfig(path: 'ci/builders/mac_test_config.json')},
    );
    final result = await runner.run([
      'build',
      '--config',
      'ci/host_debug',
      '//flutter/fml:fml_unittests',
    ]);

    printOnFailure(testEnv.testLogs.map((r) => r.message).join('\n'));
    expect(result, equals(0));

    final ninjaCmd = testEnv.processHistory.firstWhere((p) => p.command.first.endsWith('ninja'));
    expect(ninjaCmd.command, containsAllInOrder([endsWith('ninja'), '-C', endsWith('host_debug')]));
    expect(ninjaCmd.command, contains(contains('flutter/fml:fml_unittests')));
  });

  test('build command invokes ninja with all matched targets', () async {
    final testEnv = TestEnvironment.withTestEngine(
      abi: Abi.macosArm64,
      cannedProcesses: [
        CannedProcess(
          (command) => command.contains('desc'),
          stdout: convert.jsonEncode({
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
    addTearDown(testEnv.cleanup);

    final builder = TestBuilderConfig();
    builder.addBuild(
      name: 'ci/host_debug',
      targetDir: 'host_debug',
      dimension: TestDroneDimension.mac,
    );
    final runner = ToolCommandRunner(
      environment: testEnv.environment,
      configs: {'mac_test_config': builder.buildConfig(path: 'ci/builders/mac_test_config.json')},
    );
    final result = await runner.run(['build', '--config', 'ci/host_debug', '//flutter/...']);
    printOnFailure(testEnv.testLogs.map((r) => r.message).join('\n'));
    expect(result, equals(0));

    final ninjaCmd = testEnv.processHistory.firstWhere((p) => p.command.first.endsWith('ninja'));
    expect(ninjaCmd.command, containsAllInOrder([endsWith('ninja'), '-C', endsWith('host_debug')]));

    expect(
      ninjaCmd.command,
      containsAll([
        'flutter/display_list:display_list_unittests',
        'flutter/flow:flow_unittests',
        'flutter/fml:fml_unittests',
      ]),
    );
  });

  test('build command gracefully handles no matched targets', () async {
    final testEnv = TestEnvironment.withTestEngine(
      abi: Abi.macosArm64,
      cannedProcesses: [
        CannedProcess(
          (command) => command.contains('desc'),
          stdout: '''
The input testing/scenario_app:sceario_app matches no targets, configs or files.
''',
          exitCode: 1,
        ),
      ],
    );
    addTearDown(testEnv.cleanup);

    final builder = TestBuilderConfig();
    builder.addBuild(
      name: 'ci/host_debug',
      targetDir: 'host_debug',
      dimension: TestDroneDimension.mac,
    );
    final runner = ToolCommandRunner(
      environment: testEnv.environment,
      configs: {'mac_test_config': builder.buildConfig(path: 'ci/builders/mac_test_config.json')},
    );
    final result = await runner.run([
      'build',
      '--config',
      'ci/host_debug',
      // Intentionally omit the prefix '//flutter/' to trigger the warning.
      '//testing/scenario_app',
    ]);

    printOnFailure(testEnv.testLogs.map((r) => r.message).join('\n'));
    expect(result, equals(0));

    expect(
      testEnv.testLogs.map((LogRecord r) => r.message).join(),
      contains('No targets matched the pattern `testing/scenario_app'),
    );
  });

  test('build command warns on an unrecognized action', () async {
    final testEnv = TestEnvironment.withTestEngine(
      abi: Abi.macosArm64,
      cannedProcesses: [
        CannedProcess(
          (command) => command.contains('desc'),
          stdout: convert.jsonEncode({
            '//flutter/tools/unrecognized:action': {
              'outputs': ['//out/host_debug/unrecognized_action'],
              'testonly': true,
              'type': 'unrecognized',
            },
          }),
        ),
      ],
    );
    addTearDown(testEnv.cleanup);

    final builder = TestBuilderConfig();
    builder.addBuild(
      name: 'ci/host_debug',
      targetDir: 'host_debug',
      dimension: TestDroneDimension.mac,
    );
    final runner = ToolCommandRunner(
      environment: testEnv.environment,
      configs: {'mac_test_config': builder.buildConfig(path: 'ci/builders/mac_test_config.json')},
    );

    final result = await runner.run([
      'build',
      '--config',
      'ci/host_debug',
      '//flutter/tools/unrecognized:action',
    ]);

    printOnFailure(testEnv.testLogs.map((r) => r.message).join('\n'));
    expect(result, equals(0));

    expect(
      testEnv.testLogs,
      contains(
        logRecord(
          stringContainsInOrder([
            'Unknown target type',
            '//flutter/tools/unrecognized:action',
            'type=unrecognized',
          ]),
          level: Logger.warningLevel,
        ),
      ),
    );

    expect(
      testEnv.testLogs,
      contains(
        logRecord(
          stringContainsInOrder([
            'One or more targets specified did not match',
            '//flutter/tools/unrecognized:action',
          ]),
          level: Logger.warningLevel,
        ),
      ),
    );
  });

  test('et help build line length is not too big', () async {
    final testEnv = TestEnvironment.withTestEngine(verbose: true);
    addTearDown(testEnv.cleanup);

    final runner = ToolCommandRunner(environment: testEnv.environment, configs: {}, help: true);
    final result = await runner.run(['help', 'build']);
    expect(result, equals(0));

    // Avoid a degenerate case where nothing is logged.
    expect(testEnv.testLogs, isNotEmpty, reason: 'No logs were emitted');

    expect(
      testEnv.testLogs.map((LogRecord r) => r.message.split('\n')),
      everyElement(hasLength(lessThanOrEqualTo(100))),
    );
  });

  test('verbose "et help build" contains CI builds', () async {
    final testEnv = TestEnvironment.withTestEngine(abi: Abi.macosArm64, verbose: true);
    addTearDown(testEnv.cleanup);

    final builder = TestBuilderConfig();
    builder.addBuild(name: 'ci/linux_android_debug', dimension: TestDroneDimension.mac);
    final runner = ToolCommandRunner(
      environment: testEnv.environment,
      configs: {
        'linux_test_config': builder.buildConfig(path: 'ci/builders/linux_test_config.json'),
      },
      help: true,
    );
    final result = await runner.run(['--verbose', 'help', 'build']);

    printOnFailure(testEnv.testLogs.map((r) => r.message).join('\n'));
    expect(result, equals(0));

    // Avoid a degenerate case where nothing is logged.
    expect(testEnv.testLogs, isNotEmpty, reason: 'No logs were emitted');
    print(testEnv.testLogs);

    expect(testEnv.testLogs.map((LogRecord r) => r.message), contains(contains('[ci/')));
  });

  test('non-verbose "et help build" does not contain ci builds', () async {
    final testEnv = TestEnvironment.withTestEngine(abi: Abi.macosArm64);
    addTearDown(testEnv.cleanup);

    final builder = TestBuilderConfig();
    builder.addBuild(name: 'ci/linux_android_debug', dimension: TestDroneDimension.mac);
    final runner = ToolCommandRunner(
      environment: testEnv.environment,
      configs: {
        'linux_test_config': builder.buildConfig(path: 'ci/builders/linux_test_config.json'),
      },
      help: true,
    );
    final result = await runner.run(['help', 'build']);

    printOnFailure(testEnv.testLogs.map((r) => r.message).join('\n'));
    expect(result, equals(0));

    // Avoid a degenerate case where nothing is logged.
    expect(testEnv.testLogs, isNotEmpty, reason: 'No logs were emitted');

    expect(
      testEnv.testLogs.map((LogRecord r) => r.message),
      isNot(contains(contains('[ci/'))),
      reason: 'The log should not contain CI-prefixed builds',
    );
  });
}
