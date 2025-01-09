// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:args/command_runner.dart';

import '../framework/runner.dart';

class TestCommand extends Command<void> {
  TestCommand() {
    argParser.addOption(
      'task',
      abbr: 't',
      help:
          'The name of a task listed under bin/tasks.\n'
          '   Example: complex_layout__start_up.\n',
    );
    argParser.addMultiOption(
      'task-args',
      help:
          'List of arguments to pass to the task.\n'
          'For example, "--task-args build" is passed as "bin/task/task.dart --build"',
    );
    argParser.addOption(
      'device-id',
      abbr: 'd',
      help:
          'Target device id (prefixes are allowed, names are not supported).\n'
          'The option will be ignored if the test target does not run on a\n'
          'mobile device. This still respects the device operating system\n'
          'settings in the test case, and will results in error if no device\n'
          'with given ID/ID prefix is found.',
    );
    argParser.addFlag(
      'exit',
      help:
          'Exit on the first test failure. Currently flakes are intentionally (though '
          'incorrectly) not considered to be failures.',
    );
    argParser.addOption(
      'git-branch',
      help:
          '[Flutter infrastructure] Git branch of the current commit. LUCI\n'
          'checkouts run in detached HEAD state, so the branch must be passed.',
    );
    argParser.addOption(
      'local-engine',
      help:
          'Name of a build output within the engine out directory, if you\n'
          'are building Flutter locally. Use this to select a specific\n'
          'version of the engine if you have built multiple engine targets.\n'
          'This path is relative to --local-engine-src-path/out. This option\n'
          'is required when running an A/B test (see the --ab option).',
    );
    argParser.addOption(
      'local-engine-host',
      help:
          'Name of a build output within the engine out directory, if you\n'
          'are building Flutter locally. Use this to select a specific\n'
          'version of the engine to use as the host platform if you have built '
          'multiple engine targets.\n'
          'This path is relative to --local-engine-src-path/out. This option\n'
          'is required when running an A/B test (see the --ab option).',
    );
    argParser.addOption(
      'local-engine-src-path',
      help:
          'Path to your engine src directory, if you are building Flutter\n'
          'locally. Defaults to \$FLUTTER_ENGINE if set, or tries to guess at\n'
          'the location based on the value of the --flutter-root option.',
    );
    argParser.addOption(
      'luci-builder',
      help: '[Flutter infrastructure] Name of the LUCI builder being run on.',
    );
    argParser.addOption(
      'results-file',
      help:
          '[Flutter infrastructure] File path for test results. If passed with\n'
          'task, will write test results to the file.',
    );
    argParser.addFlag(
      'silent',
      help: 'Suppresses standard output and only print standard error output.',
    );
    argParser.addFlag('use-emulator', help: 'Use an emulator instead of a device to run tests.');
  }

  @override
  String get name => 'test';

  @override
  String get description => 'Run Flutter DeviceLab test';

  @override
  Future<void> run() async {
    final List<String> taskArgsRaw = argResults!['task-args'] as List<String>;
    // Prepend '--' to convert args to options when passed to task
    final List<String> taskArgs = taskArgsRaw.map((String taskArg) => '--$taskArg').toList();
    print(taskArgs);
    await runTasks(
      <String>[argResults!['task'] as String],
      deviceId: argResults!['device-id'] as String?,
      gitBranch: argResults!['git-branch'] as String?,
      localEngine: argResults!['local-engine'] as String?,
      localEngineHost: argResults!['local-engine-host'] as String?,
      localEngineSrcPath: argResults!['local-engine-src-path'] as String?,
      luciBuilder: argResults!['luci-builder'] as String?,
      resultsPath: argResults!['results-file'] as String?,
      silent: (argResults!['silent'] as bool?) ?? false,
      useEmulator: (argResults!['use-emulator'] as bool?) ?? false,
      taskArgs: taskArgs,
      exitOnFirstTestFailure: argResults!['exit'] as bool,
    );
  }
}
