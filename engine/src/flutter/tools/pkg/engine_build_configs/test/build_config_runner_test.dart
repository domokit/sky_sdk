// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert' as convert;
import 'dart:ffi' as ffi;
import 'dart:io' as io;

import 'package:engine_build_configs/src/build_config.dart';
import 'package:engine_build_configs/src/build_config_runner.dart';
import 'package:engine_repo_tools/engine_repo_tools.dart';
import 'package:path/path.dart' as path;
import 'package:platform/platform.dart';
import 'package:process_fakes/process_fakes.dart';
import 'package:process_runner/process_runner.dart';
import 'package:test/test.dart';

import 'fixtures.dart' as fixtures;

void main() {
  // Find the engine repo.
  final engine = Engine.findWithin();

  final BuilderConfig buildConfig = BuilderConfig.fromJson(
    path: 'linux_test_config',
    map: convert.jsonDecode(fixtures.buildConfigJson) as Map<String, Object?>,
  );

  test('BuildTaskRunner runs the right commands', () async {
    final BuildTask generator = buildConfig.builds[0].generators[0];
    final BuildTaskRunner taskRunner = BuildTaskRunner(
      platform: FakePlatform(operatingSystem: Platform.linux, numberOfProcessors: 32),
      processRunner: ProcessRunner(
        // dryRun should not try to spawn any processes.
        processManager: _fakeProcessManager(),
      ),
      abi: ffi.Abi.linuxX64,
      engineSrcDir: engine.srcDir,
      task: generator,
      dryRun: true,
    );
    final List<RunnerEvent> events = <RunnerEvent>[];
    void handler(RunnerEvent event) => events.add(event);
    final bool runResult = await taskRunner.run(handler);

    expect(runResult, isTrue);
    expect(events[0] is RunnerStart, isTrue);
    expect(events[0].name, equals('generator_task'));
    expect(events[0].command[0], contains('python3'));
    expect(events[0].command[1], contains('gen/script.py'));
    expect(events[0].command[2], contains('--gen-param'));
    expect(events[1] is RunnerResult, isTrue);
    expect(events[1].name, equals('generator_task'));
  });

  test('BuildTestRunner runs the right commands', () async {
    final BuildTest test = buildConfig.builds[0].tests[0];
    final BuildTestRunner testRunner = BuildTestRunner(
      platform: FakePlatform(operatingSystem: Platform.linux, numberOfProcessors: 32),
      processRunner: ProcessRunner(
        // dryRun should not try to spawn any processes.
        processManager: _fakeProcessManager(),
      ),
      abi: ffi.Abi.linuxX64,
      engineSrcDir: engine.srcDir,
      test: test,
      dryRun: true,
    );
    final List<RunnerEvent> events = <RunnerEvent>[];
    void handler(RunnerEvent event) => events.add(event);
    final bool runResult = await testRunner.run(handler);

    expect(runResult, isTrue);

    // Check that the events for the tests are correct.
    expect(events[0] is RunnerStart, isTrue);
    expect(events[0].name, equals('build_name tests'));
    expect(events[0].command[0], contains('python3'));
    expect(events[0].command[1], contains('test/script.py'));
    expect(events[0].command[2], contains('--test-params'));
    expect(events[1] is RunnerResult, isTrue);
    expect(events[1].name, equals('build_name tests'));
  });

  test('GlobalBuildRunner runs the right commands', () async {
    final Build targetBuild = buildConfig.builds[0];
    final BuildRunner buildRunner = BuildRunner(
      platform: FakePlatform(operatingSystem: Platform.linux, numberOfProcessors: 32),
      processRunner: ProcessRunner(
        // dryRun should not try to spawn any processes.
        processManager: _fakeProcessManager(),
      ),
      abi: ffi.Abi.linuxX64,
      engineSrcDir: engine.srcDir,
      build: targetBuild,
      dryRun: true,
    );
    final List<RunnerEvent> events = <RunnerEvent>[];
    void handler(RunnerEvent event) => events.add(event);
    final bool runResult = await buildRunner.run(handler);

    final String buildName = targetBuild.name;

    expect(runResult, isTrue);

    // Check that the events for the GN command are correct.
    expect(events[0] is RunnerStart, isTrue);
    expect(events[0].name, equals('$buildName: GN'));
    expect(events[0].command[0], contains('flutter/tools/gn'));
    for (final String gnArg in targetBuild.gn) {
      expect(events[0].command.contains(gnArg), isTrue);
    }
    expect(events[1] is RunnerResult, isTrue);
    expect(events[1].name, equals('$buildName: GN'));

    // Check that the events for the Ninja command are correct.
    expect(events[2] is RunnerStart, isTrue);
    expect(events[2].name, equals('$buildName: ninja'));
    expect(events[2].command[0], contains('ninja'));
    final String configPath = '${engine.srcDir.path}/out/${targetBuild.ninja.config}';
    expect(events[2].command.contains(configPath), isTrue);
    for (final String target in targetBuild.ninja.targets) {
      expect(events[2].command.contains(target), isTrue);
    }
    expect(events[3] is RunnerResult, isTrue);
    expect(events[3].name, equals('$buildName: ninja'));

    // Check that the events for generators are correct.
    expect(events[4] is RunnerStart, isTrue);
    expect(events[4].name, equals('generator_task'));
    expect(events[4].command[0], contains('python3'));
    expect(events[4].command[1], contains('gen/script.py'));
    expect(events[4].command[2], contains('--gen-param'));
    expect(events[5] is RunnerResult, isTrue);
    expect(events[5].name, equals('generator_task'));

    // Check that the events for the tests are correct.
    expect(events[6] is RunnerStart, isTrue);
    expect(events[6].name, equals('$buildName tests'));
    expect(events[6].command[0], contains('python3'));
    expect(events[6].command[1], contains('test/script.py'));
    expect(events[6].command[2], contains('--test-params'));
    expect(events[7] is RunnerResult, isTrue);
    expect(events[7].name, equals('$buildName tests'));
  });

  test('GlobalBuildRunner extra args are propagated correctly', () async {
    final Build targetBuild = buildConfig.builds[0];
    final BuildRunner buildRunner = BuildRunner(
      platform: FakePlatform(operatingSystem: Platform.linux, numberOfProcessors: 32),
      processRunner: ProcessRunner(
        // dryRun should not try to spawn any processes.
        processManager: _fakeProcessManager(),
      ),
      abi: ffi.Abi.linuxX64,
      engineSrcDir: engine.srcDir,
      build: targetBuild,
      extraGnArgs: <String>['--extra-gn-arg'],
      extraNinjaArgs: <String>['--extra-ninja-arg'],
      extraTestArgs: <String>['--extra-test-arg'],
      dryRun: true,
    );
    final List<RunnerEvent> events = <RunnerEvent>[];
    void handler(RunnerEvent event) => events.add(event);
    final bool runResult = await buildRunner.run(handler);

    final String buildName = targetBuild.name;

    expect(runResult, isTrue);

    // Check that the events for the GN command are correct.
    expect(events[0] is RunnerStart, isTrue);
    expect(events[0].name, equals('$buildName: GN'));
    expect(events[0].command.contains('--extra-gn-arg'), isTrue);

    // Check that the events for the Ninja command are correct.
    expect(events[2] is RunnerStart, isTrue);
    expect(events[2].name, equals('$buildName: ninja'));
    expect(events[2].command.contains('--extra-ninja-arg'), isTrue);

    // Check that the events for the tests are correct.
    expect(events[6] is RunnerStart, isTrue);
    expect(events[6].name, equals('$buildName tests'));
    expect(events[6].command.contains('--extra-test-arg'), isTrue);
  });

  test('GlobalBuildRunner passes large -j for an rbe build', () async {
    final Build targetBuild = buildConfig.builds[0];
    final BuildRunner buildRunner = BuildRunner(
      platform: FakePlatform(operatingSystem: Platform.linux, numberOfProcessors: 32),
      processRunner: ProcessRunner(processManager: _fakeProcessManager()),
      abi: ffi.Abi.linuxX64,
      engineSrcDir: engine.srcDir,
      build: targetBuild,
      extraGnArgs: <String>['--rbe'],
      dryRun: true,
    );
    final List<RunnerEvent> events = <RunnerEvent>[];
    void handler(RunnerEvent event) => events.add(event);
    final bool runResult = await buildRunner.run(handler);

    final String buildName = targetBuild.name;

    expect(runResult, isTrue);

    // Check that the events for the RBE bootstrap command are correct.
    expect(events[2] is RunnerStart, isTrue);
    expect(events[2].name, equals('$buildName: RBE startup'));
    expect(events[3] is RunnerResult, isTrue);
    expect(events[3].name, equals('$buildName: RBE startup'));

    // Check that the events for the Ninja command are correct.
    expect(events[4] is RunnerStart, isTrue);
    expect(events[4].name, equals('$buildName: ninja'));
    expect(events[4].command.contains('-j'), isTrue);
    expect(events[4].command.contains('1000'), isTrue);
    expect(events[5] is RunnerResult, isTrue);
    expect(events[5].name, equals('$buildName: ninja'));

    expect(events[6] is RunnerStart, isTrue);
    expect(events[6].name, equals('$buildName: RBE shutdown'));
    expect(events[7] is RunnerResult, isTrue);
    expect(events[7].name, equals('$buildName: RBE shutdown'));
    expect((events[7] as RunnerResult).okMessage, equals('OK'));
  });

  test(
    'GlobalBuildRunner passes the specified -j when explicitly provided in an RBE build',
    () async {
      final Build targetBuild = buildConfig.builds[0];
      final BuildRunner buildRunner = BuildRunner(
        platform: FakePlatform(operatingSystem: Platform.linux, numberOfProcessors: 32),
        processRunner: ProcessRunner(processManager: _fakeProcessManager()),
        abi: ffi.Abi.linuxX64,
        engineSrcDir: engine.srcDir,
        build: targetBuild,
        concurrency: 500,
        extraGnArgs: <String>['--rbe'],
        dryRun: true,
      );
      final List<RunnerEvent> events = <RunnerEvent>[];
      void handler(RunnerEvent event) => events.add(event);
      final bool runResult = await buildRunner.run(handler);

      final String buildName = targetBuild.name;

      expect(runResult, isTrue);

      // Check that the events for the Ninja command are correct.
      expect(events[4] is RunnerStart, isTrue);
      expect(events[4].name, equals('$buildName: ninja'));
      expect(events[4].command.contains('-j'), isTrue);
      expect(events[4].command.contains('500'), isTrue);
      expect(events[5] is RunnerResult, isTrue);
      expect(events[5].name, equals('$buildName: ninja'));
    },
  );

  test('GlobalBuildRunner sets default RBE env vars in an RBE build', () async {
    final Build targetBuild = buildConfig.builds[0];
    final BuildRunner buildRunner = BuildRunner(
      platform: FakePlatform(operatingSystem: Platform.linux, numberOfProcessors: 32),
      processRunner: ProcessRunner(processManager: _fakeProcessManager()),
      abi: ffi.Abi.linuxX64,
      engineSrcDir: engine.srcDir,
      build: targetBuild,
      concurrency: 500,
      extraGnArgs: <String>['--rbe'],
      dryRun: true,
    );
    final List<RunnerEvent> events = <RunnerEvent>[];
    void handler(RunnerEvent event) => events.add(event);
    final bool runResult = await buildRunner.run(handler);

    final String buildName = targetBuild.name;

    expect(runResult, isTrue);

    // Check that the events for the Ninja command are correct.
    expect(events[4] is RunnerStart, isTrue);
    expect(events[4].name, equals('$buildName: ninja'));
    expect(events[4].environment, isNotNull);
    expect(events[4].environment!.containsKey('RBE_exec_strategy'), isTrue);
    expect(events[4].environment!['RBE_exec_strategy'], equals(RbeExecStrategy.racing.toString()));
    expect(events[4].environment!.containsKey('RBE_racing_bias'), isTrue);
    expect(events[4].environment!['RBE_racing_bias'], equals('0.95'));
    expect(events[4].environment!.containsKey('RBE_local_resource_fraction'), isTrue);
    expect(events[4].environment!['RBE_local_resource_fraction'], equals('0.2'));
  });

  test('GlobalBuildRunner sets RBE_disable_remote when remote builds are disabled', () async {
    final Build targetBuild = buildConfig.builds[0];
    final BuildRunner buildRunner = BuildRunner(
      platform: FakePlatform(operatingSystem: Platform.linux, numberOfProcessors: 32),
      processRunner: ProcessRunner(processManager: _fakeProcessManager()),
      abi: ffi.Abi.linuxX64,
      engineSrcDir: engine.srcDir,
      build: targetBuild,
      concurrency: 500,
      rbeConfig: const RbeConfig(remoteDisabled: true),
      extraGnArgs: <String>['--rbe'],
      dryRun: true,
    );
    final List<RunnerEvent> events = <RunnerEvent>[];
    void handler(RunnerEvent event) => events.add(event);
    final bool runResult = await buildRunner.run(handler);

    final String buildName = targetBuild.name;

    expect(runResult, isTrue);

    // Check that the events for the Ninja command are correct.
    expect(events[4] is RunnerStart, isTrue);
    expect(events[4].name, equals('$buildName: ninja'));
    expect(events[4].environment, isNotNull);
    expect(events[4].environment!.containsKey('RBE_remote_disabled'), isTrue);
    expect(events[4].environment!['RBE_remote_disabled'], equals('1'));
    expect(events[4].environment!.containsKey('RBE_exec_strategy'), isFalse);
    expect(events[4].environment!.containsKey('RBE_racing_bias'), isFalse);
    expect(events[4].environment!.containsKey('RBE_local_resource_fraction'), isFalse);
  });

  test(
    'GlobalBuildRunner sets RBE_exec_strategy when a non-default value is passed in the RbeConfig',
    () async {
      final Build targetBuild = buildConfig.builds[0];
      final BuildRunner buildRunner = BuildRunner(
        platform: FakePlatform(operatingSystem: Platform.linux, numberOfProcessors: 32),
        processRunner: ProcessRunner(processManager: _fakeProcessManager()),
        abi: ffi.Abi.linuxX64,
        engineSrcDir: engine.srcDir,
        build: targetBuild,
        concurrency: 500,
        rbeConfig: const RbeConfig(execStrategy: RbeExecStrategy.local),
        extraGnArgs: <String>['--rbe'],
        dryRun: true,
      );
      final List<RunnerEvent> events = <RunnerEvent>[];
      void handler(RunnerEvent event) => events.add(event);
      final bool runResult = await buildRunner.run(handler);

      final String buildName = targetBuild.name;

      expect(runResult, isTrue);

      // Check that the events for the Ninja command are correct.
      expect(events[4] is RunnerStart, isTrue);
      expect(events[4].name, equals('$buildName: ninja'));
      expect(events[4].environment, isNotNull);
      expect(events[4].environment!.containsKey('RBE_remote_disabled'), isFalse);
      expect(events[4].environment!.containsKey('RBE_exec_strategy'), isTrue);
      expect(events[4].environment!['RBE_exec_strategy'], equals(RbeExecStrategy.local.toString()));
      expect(events[4].environment!.containsKey('RBE_racing_bias'), isFalse);
      expect(events[4].environment!.containsKey('RBE_local_resource_fraction'), isFalse);
    },
  );

  test(
    'GlobalBuildRunner passes the specified -j when explicitly provided in a non-RBE build',
    () async {
      final Build targetBuild = buildConfig.builds[0];
      final BuildRunner buildRunner = BuildRunner(
        platform: FakePlatform(operatingSystem: Platform.linux, numberOfProcessors: 32),
        processRunner: ProcessRunner(processManager: _fakeProcessManager()),
        abi: ffi.Abi.linuxX64,
        engineSrcDir: engine.srcDir,
        build: targetBuild,
        concurrency: 500,
        extraGnArgs: <String>['--no-rbe'],
        dryRun: true,
      );
      final List<RunnerEvent> events = <RunnerEvent>[];
      void handler(RunnerEvent event) => events.add(event);
      final bool runResult = await buildRunner.run(handler);

      final String buildName = targetBuild.name;

      expect(runResult, isTrue);

      // Check that the events for the Ninja command are correct.
      expect(events[2] is RunnerStart, isTrue);
      expect(events[2].name, equals('$buildName: ninja'));
      expect(events[2].command.contains('-j'), isTrue);
      expect(events[2].command.contains('500'), isTrue);
      expect(events[3] is RunnerResult, isTrue);
      expect(events[3].name, equals('$buildName: ninja'));
    },
  );

  test('GlobalBuildRunner skips GN when runGn is false', () async {
    final Build targetBuild = buildConfig.builds[0];
    final BuildRunner buildRunner = BuildRunner(
      platform: FakePlatform(operatingSystem: Platform.linux, numberOfProcessors: 32),
      processRunner: ProcessRunner(
        // dryRun should not try to spawn any processes.
        processManager: _fakeProcessManager(),
      ),
      abi: ffi.Abi.linuxX64,
      engineSrcDir: engine.srcDir,
      build: targetBuild,
      runGn: false,
      dryRun: true,
    );
    final List<RunnerEvent> events = <RunnerEvent>[];
    void handler(RunnerEvent event) => events.add(event);
    final bool runResult = await buildRunner.run(handler);

    final String buildName = targetBuild.name;

    expect(runResult, isTrue);

    // Check that the events for the Ninja command are correct.
    expect(events[0] is RunnerStart, isTrue);
    expect(events[0].name, equals('$buildName: ninja'));
    expect(events[0].command[0], contains('ninja'));
    final String configPath = '${engine.srcDir.path}/out/${targetBuild.ninja.config}';
    expect(events[0].command.contains(configPath), isTrue);
    for (final String target in targetBuild.ninja.targets) {
      expect(events[0].command.contains(target), isTrue);
    }
    expect(events[1] is RunnerResult, isTrue);
    expect(events[1].name, equals('$buildName: ninja'));
  });

  test('GlobalBuildRunner skips Ninja when runNinja is false', () async {
    final Build targetBuild = buildConfig.builds[0];
    final BuildRunner buildRunner = BuildRunner(
      platform: FakePlatform(operatingSystem: Platform.linux, numberOfProcessors: 32),
      processRunner: ProcessRunner(
        // dryRun should not try to spawn any processes.
        processManager: _fakeProcessManager(),
      ),
      abi: ffi.Abi.linuxX64,
      engineSrcDir: engine.srcDir,
      build: targetBuild,
      runNinja: false,
      dryRun: true,
    );
    final List<RunnerEvent> events = <RunnerEvent>[];
    void handler(RunnerEvent event) => events.add(event);
    final bool runResult = await buildRunner.run(handler);

    final String buildName = targetBuild.name;

    expect(runResult, isTrue);

    // Check that the events for the GN command are correct.
    expect(events[0] is RunnerStart, isTrue);
    expect(events[0].name, equals('$buildName: GN'));
    expect(events[0].command[0], contains('flutter/tools/gn'));
    for (final String gnArg in targetBuild.gn) {
      expect(events[0].command.contains(gnArg), isTrue);
    }
    expect(events[1] is RunnerResult, isTrue);
    expect(events[1].name, equals('$buildName: GN'));

    // Check that the events for generators are correct.
    expect(events[2] is RunnerStart, isTrue);
    expect(events[2].name, equals('generator_task'));
    expect(events[2].command[0], contains('python3'));
    expect(events[2].command[1], contains('gen/script.py'));
    expect(events[2].command[2], contains('--gen-param'));
    expect(events[3] is RunnerResult, isTrue);
    expect(events[3].name, equals('generator_task'));
  });

  test('fixes gcc paths', () {
    final String outDir = path.join(io.Directory.current.path, 'foo', 'bar');
    const String error =
        'flutter/impeller/renderer/backend/metal/allocator_mtl.h:69:33: error: foobar';
    final String fixed = BuildRunner.fixGccPaths('../../$error', outDir);
    expect(fixed, './$error');
  });

  test('fixes gcc paths with ansi colors', () {
    final String outDir = path.join(io.Directory.current.path, 'foo', 'bar');
    // An error string with ANSI escape codes for colors.
    final List<int> bytes = [
      27, 91, 49, 109, 46, 46, 47, 46, 46, 47, 102, //
      108, 117, 116, 116, 101, 114, 47, 105, 109, 112, 101, 108, 108, 101, //
      114, 47, 100, 105, 115, 112, 108, 97, 121, 95, 108, 105, 115, 116, 47, //
      100, 108, 95, 100, 105, 115, 112, 97, 116, 99, 104, 101, 114, 46, 99, //
      99, 58, 55, 51, 52, 58, 55, 58, 32, 27, 91, 48, 109, 27, 91, 48, 59, //
      49, 59, 51, 49, 109, 101, 114, 114, 111, 114, 58, 32, 27, 91, 48, 109, //
      27, 91, 49, 109, 117, 115, 101, 32, 111, 102, 32, 117, 110, 100, 101, //
      99, 108, 97, 114, 101, 100, 32, 105, 100, 101, 110, 116, 105, 102, 105, //
      101, 114, 32, 39, 114, 111, 99, 107, 101, 116, 39, 27, 91, 48, 109,
    ];
    final String error = convert.utf8.decode(bytes);
    final String fixed = BuildRunner.fixGccPaths(error, outDir);
    expect(
      fixed.contains('../../flutter/impeller/display_list/dl_dispatcher.cc'),
      isFalse,
      reason: 'Fixed string: $fixed',
    );
    expect(
      fixed.contains('./flutter/impeller/display_list/dl_dispatcher.cc'),
      isTrue,
      reason: 'Fixed string: $fixed',
    );
    expect(fixed[0], '\x1B', reason: 'Fixed string: $fixed');
  });

  test('GlobalBuildRunner skips generators when runGenerators is false', () async {
    final Build targetBuild = buildConfig.builds[0];
    final BuildRunner buildRunner = BuildRunner(
      platform: FakePlatform(operatingSystem: Platform.linux, numberOfProcessors: 32),
      processRunner: ProcessRunner(
        // dryRun should not try to spawn any processes.
        processManager: _fakeProcessManager(),
      ),
      abi: ffi.Abi.linuxX64,
      engineSrcDir: engine.srcDir,
      build: targetBuild,
      runGenerators: false,
      dryRun: true,
    );
    final List<RunnerEvent> events = <RunnerEvent>[];
    void handler(RunnerEvent event) => events.add(event);
    final bool runResult = await buildRunner.run(handler);

    final String buildName = targetBuild.name;

    expect(runResult, isTrue);

    // Check that the events for the Ninja command are correct.
    expect(events[2] is RunnerStart, isTrue);
    expect(events[2].name, equals('$buildName: ninja'));
    expect(events[2].command[0], contains('ninja'));
    final String configPath = '${engine.srcDir.path}/out/${targetBuild.ninja.config}';
    expect(events[2].command.contains(configPath), isTrue);
    for (final String target in targetBuild.ninja.targets) {
      expect(events[2].command.contains(target), isTrue);
    }
    expect(events[3] is RunnerResult, isTrue);
    expect(events[3].name, equals('$buildName: ninja'));

    // Check that the events for the tests are correct.
    expect(events[4] is RunnerStart, isTrue);
    expect(events[4].name, equals('$buildName tests'));
    expect(events[4].command[0], contains('python3'));
    expect(events[4].command[1], contains('test/script.py'));
    expect(events[4].command[2], contains('--test-params'));
    expect(events[5] is RunnerResult, isTrue);
    expect(events[5].name, equals('$buildName tests'));
  });

  test('GlobalBuildRunner skips tests when runTests is false', () async {
    final Build targetBuild = buildConfig.builds[0];
    final BuildRunner buildRunner = BuildRunner(
      platform: FakePlatform(operatingSystem: Platform.linux, numberOfProcessors: 32),
      processRunner: ProcessRunner(
        // dryRun should not try to spawn any processes.
        processManager: _fakeProcessManager(),
      ),
      abi: ffi.Abi.linuxX64,
      engineSrcDir: engine.srcDir,
      build: targetBuild,
      runTests: false,
      dryRun: true,
    );
    final List<RunnerEvent> events = <RunnerEvent>[];
    void handler(RunnerEvent event) => events.add(event);
    final bool runResult = await buildRunner.run(handler);

    expect(runResult, isTrue);

    // Check that the events for generators are correct.
    expect(events[4] is RunnerStart, isTrue);
    expect(events[4].name, equals('generator_task'));
    expect(events[4].command[0], contains('python3'));
    expect(events[4].command[1], contains('gen/script.py'));
    expect(events[4].command[2], contains('--gen-param'));
    expect(events[5] is RunnerResult, isTrue);
    expect(events[5].name, equals('generator_task'));

    expect(events.length, equals(6));
  });

  test('GlobalBuildRunner extraGnArgs overrides build config args', () async {
    final Build targetBuild = buildConfig.builds[0];
    final BuildRunner buildRunner = BuildRunner(
      platform: FakePlatform(operatingSystem: Platform.linux, numberOfProcessors: 32),
      processRunner: ProcessRunner(processManager: _fakeProcessManager()),
      abi: ffi.Abi.linuxX64,
      engineSrcDir: engine.srcDir,
      build: targetBuild,
      extraGnArgs: <String>['--no-lto', '--rbe'],
      dryRun: true,
    );
    final List<RunnerEvent> events = <RunnerEvent>[];
    void handler(RunnerEvent event) => events.add(event);
    final bool runResult = await buildRunner.run(handler);

    final String buildName = targetBuild.name;

    expect(runResult, isTrue);

    // Check that the events for the GN command are correct.
    expect(events[0] is RunnerStart, isTrue);
    expect(events[0].name, equals('$buildName: GN'));
    expect(events[0].command[0], contains('flutter/tools/gn'));
    expect(events[0].command.contains('--no-lto'), isTrue);
    expect(events[0].command.contains('--rbe'), isTrue);
    expect(events[0].command.contains('--lto'), isFalse);
    expect(events[0].command.contains('--no-rbe'), isFalse);
    expect(events[1] is RunnerResult, isTrue);
    expect(events[1].name, equals('$buildName: GN'));
  });

  test('GlobalBuildRunner canRun returns false on OS mismatch', () async {
    final Build targetBuild = buildConfig.builds[0];
    final BuildRunner buildRunner = BuildRunner(
      platform: FakePlatform(operatingSystem: Platform.macOS, numberOfProcessors: 32),
      processRunner: ProcessRunner(
        // dryRun should not try to spawn any processes.
        processManager: _fakeProcessManager(),
      ),
      abi: ffi.Abi.linuxX64,
      engineSrcDir: engine.srcDir,
      build: targetBuild,
      dryRun: true,
    );
    final List<RunnerEvent> events = <RunnerEvent>[];
    void handler(RunnerEvent event) => events.add(event);
    final bool runResult = await buildRunner.run(handler);

    expect(runResult, isFalse);
    expect(events[0] is RunnerError, isTrue);
  });

  test('GlobalBuildRunner fails when gn fails', () async {
    final Build targetBuild = buildConfig.builds[0];
    final BuildRunner buildRunner = BuildRunner(
      platform: FakePlatform(operatingSystem: Platform.linux, numberOfProcessors: 32),
      processRunner: ProcessRunner(
        processManager: _fakeProcessManager(gnResult: io.ProcessResult(1, 1, '', '')),
      ),
      abi: ffi.Abi.linuxX64,
      engineSrcDir: engine.srcDir,
      build: targetBuild,
    );
    final List<RunnerEvent> events = <RunnerEvent>[];
    void handler(RunnerEvent event) => events.add(event);
    final bool runResult = await buildRunner.run(handler);

    final String buildName = targetBuild.name;

    expect(runResult, isFalse);

    expect(events[0] is RunnerStart, isTrue);
    expect(events[0].name, equals('$buildName: GN'));
    expect(events[1] is RunnerResult, isTrue);
    expect((events[1] as RunnerResult).ok, isFalse);
  });

  test('GlobalBuildRunner fails when ninja fails', () async {
    final Build targetBuild = buildConfig.builds[0];
    final BuildRunner buildRunner = BuildRunner(
      platform: FakePlatform(operatingSystem: Platform.linux, numberOfProcessors: 32),
      processRunner: ProcessRunner(
        processManager: _fakeProcessManager(ninjaResult: io.ProcessResult(1, 1, '', '')),
      ),
      abi: ffi.Abi.linuxX64,
      engineSrcDir: engine.srcDir,
      build: targetBuild,
    );
    final List<RunnerEvent> events = <RunnerEvent>[];
    void handler(RunnerEvent event) => events.add(event);
    final bool runResult = await buildRunner.run(handler);

    final String buildName = targetBuild.name;

    expect(runResult, isFalse);

    expect(events[2] is RunnerStart, isTrue);
    expect(events[2].name, equals('$buildName: ninja'));
    expect(events[3] is RunnerResult, isTrue);
    expect((events[3] as RunnerResult).ok, isFalse);
  });

  test('GlobalBuildRunner fails an RBE build when bootstrap fails', () async {
    final Build targetBuild = buildConfig.builds[0];
    final BuildRunner buildRunner = BuildRunner(
      platform: FakePlatform(operatingSystem: Platform.linux, numberOfProcessors: 32),
      processRunner: ProcessRunner(
        processManager: _fakeProcessManager(bootstrapResult: io.ProcessResult(1, 1, '', '')),
      ),
      abi: ffi.Abi.linuxX64,
      engineSrcDir: engine.srcDir,
      build: targetBuild,
      extraGnArgs: <String>['--rbe'],
    );
    final List<RunnerEvent> events = <RunnerEvent>[];
    void handler(RunnerEvent event) => events.add(event);
    final bool runResult = await buildRunner.run(handler);

    final String buildName = targetBuild.name;

    expect(runResult, isFalse);

    expect(events[2] is RunnerStart, isTrue);
    expect(events[2].name, equals('$buildName: RBE startup'));
    expect(events[3] is RunnerResult, isTrue);
    expect(events[3].name, equals('$buildName: RBE startup'));
    expect((events[3] as RunnerResult).ok, isFalse);
  });

  test('GlobalBuildRunner fails an RBE build when bootstrap does not exist', () async {
    final Build targetBuild = buildConfig.builds[0];
    final BuildRunner buildRunner = BuildRunner(
      platform: FakePlatform(operatingSystem: Platform.linux, numberOfProcessors: 32),
      processRunner: ProcessRunner(
        processManager: _fakeProcessManager(
          canRun: (Object? exe, {String? workingDirectory}) {
            if (exe is String? && exe != null && exe.endsWith('bootstrap')) {
              return false;
            }
            return true;
          },
        ),
      ),
      abi: ffi.Abi.linuxX64,
      engineSrcDir: engine.srcDir,
      build: targetBuild,
      extraGnArgs: <String>['--rbe'],
    );
    final List<RunnerEvent> events = <RunnerEvent>[];
    void handler(RunnerEvent event) => events.add(event);
    final bool runResult = await buildRunner.run(handler);

    expect(runResult, isFalse);

    expect(events[2] is RunnerError, isTrue);
  });

  test('GlobalBuildRunner throws a StateError on an unsupported host cpu', () async {
    final Build targetBuild = buildConfig.builds[0];
    final BuildRunner buildRunner = BuildRunner(
      platform: FakePlatform(operatingSystem: Platform.linux, numberOfProcessors: 32),
      processRunner: ProcessRunner(processManager: _fakeProcessManager()),
      abi: ffi.Abi.linuxRiscv32,
      engineSrcDir: engine.srcDir,
      build: targetBuild,
      extraGnArgs: <String>['--rbe'],
    );

    bool caughtError = false;
    try {
      await buildRunner.run((RunnerEvent event) {});
    } on StateError catch (_) {
      caughtError = true;
    }
    expect(caughtError, isTrue);
  });

  test('GlobalBuildRunner trims RBE crud from compile_commands.json', () async {
    final io.Directory emptyDir = io.Directory.systemTemp.createTempSync(
      'build_config_runner.test',
    );
    try {
      final io.Directory srcDir = io.Directory(path.join(emptyDir.path, 'src'));
      final io.Directory hostDebug = io.Directory(path.join(srcDir.path, 'out', 'build_name'))
        ..createSync(recursive: true);
      final io.File file = io.File(path.join(hostDebug.path, 'compile_commands.json'));
      file.writeAsStringSync(r'''
[
  {
    "file": "../../flutter/assets/asset_manager.cc",
    "directory": "/Users/flutter/src/engine/src/out/host_debug_unopt_arm64",
    "command": "/Users/flutter/src/engine/src/flutter/buildtools/mac-arm64/reclient/rewrapper --cfg=/Users/flutter/src/engine/src/flutter/build/rbe/rewrapper-mac-arm64.cfg --exec_root=/Users/flutter/src/engine/src/ --remote_wrapper=../../flutter/build/rbe/remote_wrapper.sh --local_wrapper=../../flutter/build/rbe/local_wrapper.sh --labels=type=compile,compiler=clang,lang=cpp ../../flutter/buildtools/mac-x64/clang/bin/clang++ -MMD -MF  obj/flutter/assets/assets.asset_manager.o.d  -DUSE_OPENSSL=1 -D__STDC_CONSTANT_MACROS -D__STDC_FORMAT_MACROS -D_FORTIFY_SOURCE=2 -D_LIBCPP_DISABLE_AVAILABILITY=1 -D_LIBCPP_DISABLE_VISIBILITY_ANNOTATIONS -D_LIBCPP_ENABLE_THREAD_SAFETY_ANNOTATIONS -D_DEBUG -DFLUTTER_RUNTIME_MODE_DEBUG=1 -DFLUTTER_RUNTIME_MODE_PROFILE=2 -DFLUTTER_RUNTIME_MODE_RELEASE=3 -DFLUTTER_RUNTIME_MODE_JIT_RELEASE=4 \"-DDART_LEGACY_API=[[deprecated]]\" -DFLUTTER_RUNTIME_MODE=1 -DFLUTTER_JIT_RUNTIME=1  -I../.. -Igen -I../../flutter/third_party/libcxx/include -I../../flutter/third_party/libcxxabi/include -I../../flutter/build/secondary/flutter/third_party/libcxx/config -I../../flutter  -fno-strict-aliasing -fstack-protector-all --target=arm64-apple-macos -arch arm64 -fcolor-diagnostics -Wall -Wextra -Wendif-labels -Werror -Wno-missing-field-initializers -Wno-unused-parameter -Wno-unused-but-set-parameter -Wno-unused-but-set-variable -Wno-implicit-int-float-conversion -Wno-deprecated-copy -Wno-psabi -Wno-deprecated-literal-operator -Wno-unqualified-std-cast-call -Wno-non-c-typedef-for-linkage -Wno-range-loop-construct -Wunguarded-availability -Wno-deprecated-declarations -no-canonical-prefixes -fvisibility=hidden -Wstring-conversion -Wnewline-eof -O0 -g2 -Wunreachable-code  -fvisibility-inlines-hidden -std=c++17 -fno-rtti -nostdinc++ -nostdinc++ -fvisibility=hidden -fno-exceptions -stdlib=libc++ -isysroot ../../flutter/prebuilts/SDKs/MacOSX14.0.sdk -mmacosx-version-min=10.14.0  -c ../../flutter/assets/asset_manager.cc -o  obj/flutter/assets/assets.asset_manager.o"
  },
  {
    "file": "../../flutter/assets/directory_asset_bundle.cc",
    "directory": "/Users/flutter/src/engine/src/out/host_debug_unopt_arm64",
    "command": "/Users/flutter/src/engine/src/flutter/buildtools/mac-arm64/reclient/rewrapper --cfg=/Users/flutter/src/engine/src/flutter/build/rbe/rewrapper-mac-arm64.cfg --exec_root=/Users/flutter/src/engine/src/ --remote_wrapper=../../flutter/build/rbe/remote_wrapper.sh --local_wrapper=../../flutter/build/rbe/local_wrapper.sh --labels=type=compile,compiler=clang,lang=cpp ../../flutter/buildtools/mac-x64/clang/bin/clang++ -MMD -MF  obj/flutter/assets/assets.directory_asset_bundle.o.d  -DUSE_OPENSSL=1 -D__STDC_CONSTANT_MACROS -D__STDC_FORMAT_MACROS -D_FORTIFY_SOURCE=2 -D_LIBCPP_DISABLE_AVAILABILITY=1 -D_LIBCPP_DISABLE_VISIBILITY_ANNOTATIONS -D_LIBCPP_ENABLE_THREAD_SAFETY_ANNOTATIONS -D_DEBUG -DFLUTTER_RUNTIME_MODE_DEBUG=1 -DFLUTTER_RUNTIME_MODE_PROFILE=2 -DFLUTTER_RUNTIME_MODE_RELEASE=3 -DFLUTTER_RUNTIME_MODE_JIT_RELEASE=4 \"-DDART_LEGACY_API=[[deprecated]]\" -DFLUTTER_RUNTIME_MODE=1 -DFLUTTER_JIT_RUNTIME=1  -I../.. -Igen -I../../flutter/third_party/libcxx/include -I../../flutter/third_party/libcxxabi/include -I../../flutter/build/secondary/flutter/third_party/libcxx/config -I../../flutter  -fno-strict-aliasing -fstack-protector-all --target=arm64-apple-macos -arch arm64 -fcolor-diagnostics -Wall -Wextra -Wendif-labels -Werror -Wno-missing-field-initializers -Wno-unused-parameter -Wno-unused-but-set-parameter -Wno-unused-but-set-variable -Wno-implicit-int-float-conversion -Wno-deprecated-copy -Wno-psabi -Wno-deprecated-literal-operator -Wno-unqualified-std-cast-call -Wno-non-c-typedef-for-linkage -Wno-range-loop-construct -Wunguarded-availability -Wno-deprecated-declarations -no-canonical-prefixes -fvisibility=hidden -Wstring-conversion -Wnewline-eof -O0 -g2 -Wunreachable-code  -fvisibility-inlines-hidden -std=c++17 -fno-rtti -nostdinc++ -nostdinc++ -fvisibility=hidden -fno-exceptions -stdlib=libc++ -isysroot ../../flutter/prebuilts/SDKs/MacOSX14.0.sdk -mmacosx-version-min=10.14.0  -c ../../flutter/assets/directory_asset_bundle.cc -o  obj/flutter/assets/assets.directory_asset_bundle.o"
  },
]
''', flush: true);
      final Build targetBuild = buildConfig.builds[0];
      final BuildRunner buildRunner = BuildRunner(
        platform: FakePlatform(operatingSystem: Platform.linux, numberOfProcessors: 32),
        processRunner: ProcessRunner(
          // dryRun should not try to spawn any processes.
          processManager: _fakeProcessManager(),
        ),
        abi: ffi.Abi.linuxX64,
        engineSrcDir: srcDir,
        build: targetBuild,
      );
      final List<RunnerEvent> events = <RunnerEvent>[];
      void handler(RunnerEvent event) => events.add(event);
      await buildRunner.run(handler);
      expect(file.readAsStringSync(), r'''
[
  {
    "file": "../../flutter/assets/asset_manager.cc",
    "directory": "/Users/flutter/src/engine/src/out/host_debug_unopt_arm64",
    "command": "../../flutter/buildtools/mac-x64/clang/bin/clang++ -MMD -MF  obj/flutter/assets/assets.asset_manager.o.d  -DUSE_OPENSSL=1 -D__STDC_CONSTANT_MACROS -D__STDC_FORMAT_MACROS -D_FORTIFY_SOURCE=2 -D_LIBCPP_DISABLE_AVAILABILITY=1 -D_LIBCPP_DISABLE_VISIBILITY_ANNOTATIONS -D_LIBCPP_ENABLE_THREAD_SAFETY_ANNOTATIONS -D_DEBUG -DFLUTTER_RUNTIME_MODE_DEBUG=1 -DFLUTTER_RUNTIME_MODE_PROFILE=2 -DFLUTTER_RUNTIME_MODE_RELEASE=3 -DFLUTTER_RUNTIME_MODE_JIT_RELEASE=4 \"-DDART_LEGACY_API=[[deprecated]]\" -DFLUTTER_RUNTIME_MODE=1 -DFLUTTER_JIT_RUNTIME=1  -I../.. -Igen -I../../flutter/third_party/libcxx/include -I../../flutter/third_party/libcxxabi/include -I../../flutter/build/secondary/flutter/third_party/libcxx/config -I../../flutter  -fno-strict-aliasing -fstack-protector-all --target=arm64-apple-macos -arch arm64 -fcolor-diagnostics -Wall -Wextra -Wendif-labels -Werror -Wno-missing-field-initializers -Wno-unused-parameter -Wno-unused-but-set-parameter -Wno-unused-but-set-variable -Wno-implicit-int-float-conversion -Wno-deprecated-copy -Wno-psabi -Wno-deprecated-literal-operator -Wno-unqualified-std-cast-call -Wno-non-c-typedef-for-linkage -Wno-range-loop-construct -Wunguarded-availability -Wno-deprecated-declarations -no-canonical-prefixes -fvisibility=hidden -Wstring-conversion -Wnewline-eof -O0 -g2 -Wunreachable-code  -fvisibility-inlines-hidden -std=c++17 -fno-rtti -nostdinc++ -nostdinc++ -fvisibility=hidden -fno-exceptions -stdlib=libc++ -isysroot ../../flutter/prebuilts/SDKs/MacOSX14.0.sdk -mmacosx-version-min=10.14.0  -c ../../flutter/assets/asset_manager.cc -o  obj/flutter/assets/assets.asset_manager.o"
  },
  {
    "file": "../../flutter/assets/directory_asset_bundle.cc",
    "directory": "/Users/flutter/src/engine/src/out/host_debug_unopt_arm64",
    "command": "../../flutter/buildtools/mac-x64/clang/bin/clang++ -MMD -MF  obj/flutter/assets/assets.directory_asset_bundle.o.d  -DUSE_OPENSSL=1 -D__STDC_CONSTANT_MACROS -D__STDC_FORMAT_MACROS -D_FORTIFY_SOURCE=2 -D_LIBCPP_DISABLE_AVAILABILITY=1 -D_LIBCPP_DISABLE_VISIBILITY_ANNOTATIONS -D_LIBCPP_ENABLE_THREAD_SAFETY_ANNOTATIONS -D_DEBUG -DFLUTTER_RUNTIME_MODE_DEBUG=1 -DFLUTTER_RUNTIME_MODE_PROFILE=2 -DFLUTTER_RUNTIME_MODE_RELEASE=3 -DFLUTTER_RUNTIME_MODE_JIT_RELEASE=4 \"-DDART_LEGACY_API=[[deprecated]]\" -DFLUTTER_RUNTIME_MODE=1 -DFLUTTER_JIT_RUNTIME=1  -I../.. -Igen -I../../flutter/third_party/libcxx/include -I../../flutter/third_party/libcxxabi/include -I../../flutter/build/secondary/flutter/third_party/libcxx/config -I../../flutter  -fno-strict-aliasing -fstack-protector-all --target=arm64-apple-macos -arch arm64 -fcolor-diagnostics -Wall -Wextra -Wendif-labels -Werror -Wno-missing-field-initializers -Wno-unused-parameter -Wno-unused-but-set-parameter -Wno-unused-but-set-variable -Wno-implicit-int-float-conversion -Wno-deprecated-copy -Wno-psabi -Wno-deprecated-literal-operator -Wno-unqualified-std-cast-call -Wno-non-c-typedef-for-linkage -Wno-range-loop-construct -Wunguarded-availability -Wno-deprecated-declarations -no-canonical-prefixes -fvisibility=hidden -Wstring-conversion -Wnewline-eof -O0 -g2 -Wunreachable-code  -fvisibility-inlines-hidden -std=c++17 -fno-rtti -nostdinc++ -nostdinc++ -fvisibility=hidden -fno-exceptions -stdlib=libc++ -isysroot ../../flutter/prebuilts/SDKs/MacOSX14.0.sdk -mmacosx-version-min=10.14.0  -c ../../flutter/assets/directory_asset_bundle.cc -o  obj/flutter/assets/assets.directory_asset_bundle.o"
  },
]
''');
    } finally {
      emptyDir.deleteSync(recursive: true);
    }
  });
}

FakeProcessManager _fakeProcessManager({
  io.ProcessResult? bootstrapResult,
  io.ProcessResult? gnResult,
  io.ProcessResult? ninjaResult,
  bool Function(Object?, {String? workingDirectory})? canRun,
  bool failUnknown = true,
}) {
  final io.ProcessResult success = io.ProcessResult(1, 0, '', '');
  FakeProcess fakeProcess(io.ProcessResult? result) => FakeProcess(
    exitCode: result?.exitCode ?? 0,
    stdout: result?.stdout as String? ?? '',
    stderr: result?.stderr as String? ?? '',
  );
  return FakeProcessManager(
    canRun: canRun ?? (Object? exe, {String? workingDirectory}) => true,
    onRun:
        (List<String> cmd) => switch (cmd) {
          _ => failUnknown ? io.ProcessResult(1, 1, '', '') : success,
        },
    onStart:
        (List<String> cmd) => switch (cmd) {
          [final String exe, ...] when exe.endsWith('gn') => fakeProcess(gnResult),
          [final String exe, ...] when exe.endsWith('bootstrap') => fakeProcess(bootstrapResult),
          [final String exe, ...] when exe.endsWith('ninja') => fakeProcess(ninjaResult),
          _ => failUnknown ? FakeProcess(exitCode: 1) : FakeProcess(),
        },
  );
}
