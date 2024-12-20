// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert';
import 'dart:io';

import 'package:args/args.dart';
import 'package:flutter_devicelab/framework/ab.dart';
import 'package:flutter_devicelab/framework/runner.dart';
import 'package:flutter_devicelab/framework/task_result.dart';
import 'package:flutter_devicelab/framework/utils.dart';

/// Runs tasks.
///
/// The tasks are chosen depending on the command-line options.
Future<void> main(List<String> rawArgs) async {
  // This is populated by a callback in the ArgParser.
  final List<String> taskNames = <String>[];
  final ArgParser argParser = createArgParser(taskNames);

  ArgResults args;
  try {
    args = argParser.parse(rawArgs); // populates taskNames as a side-effect
  } on FormatException catch (error) {
    stderr.writeln('${error.message}\n');
    stderr.writeln('Usage:\n');
    stderr.writeln(argParser.usage);
    exit(1);
  }

  /// Suppresses standard output, prints only standard error output.
  final bool silent = (args['silent'] as bool?) ?? false;

  /// The build of the local engine to use.
  ///
  /// Required for A/B test mode.
  final String? localEngine = args['local-engine'] as String?;

  /// The build of the local engine to use as the host platform.
  ///
  /// Required if [localEngine] is set.
  final String? localEngineHost = args['local-engine-host'] as String?;

  /// The build of the local Web SDK to use.
  ///
  /// Required for A/B test mode.
  final String? localWebSdk = args['local-web-sdk'] as String?;

  /// The path to the engine "src/" directory.
  final String? localEngineSrcPath = args['local-engine-src-path'] as String?;

  /// The device-id to run test on.
  final String? deviceId = args['device-id'] as String?;

  /// Whether to exit on first test failure.
  final bool exitOnFirstTestFailure = (args['exit'] as bool?) ?? false;

  /// Whether to tell tasks to clean up after themselves.
  final bool terminateStrayDartProcesses =
      (args['terminate-stray-dart-processes'] as bool?) ?? false;

  /// The git branch being tested on.
  final String? gitBranch = args['git-branch'] as String?;

  /// Name of the LUCI builder this test is currently running on.
  ///
  /// This is only passed on CI runs for Cocoon to be able to uniquely identify
  /// this test run.
  final String? luciBuilder = args['luci-builder'] as String?;

  /// Path to write test results to.
  final String? resultsPath = args['results-file'] as String?;

  /// Use an emulator for this test if it is an android test.
  final bool useEmulator = (args['use-emulator'] as bool?) ?? false;

  if (args.wasParsed('list')) {
    for (int i = 0; i < taskNames.length; i++) {
      print('${(i + 1).toString().padLeft(3)} - ${taskNames[i]}');
    }
    exit(0);
  }

  if (taskNames.isEmpty) {
    stderr.writeln('Failed to find tasks to run based on supplied options.');
    exit(1);
  }

  if (args.wasParsed('ab')) {
    final int runsPerTest = int.parse(args['ab'] as String);
    final String resultsFile = args['ab-result-file'] as String? ?? 'ABresults#.json';
    if (taskNames.length > 1) {
      stderr.writeln(
        'When running in A/B test mode exactly one task must be passed but got ${taskNames.join(', ')}.\n',
      );
      stderr.writeln(argParser.usage);
      exit(1);
    }
    if (localEngine == null && localWebSdk == null) {
      stderr.writeln(
        'When running in A/B test mode --local-engine or --local-web-sdk is required.\n',
      );
      stderr.writeln(argParser.usage);
      exit(1);
    }
    if (localWebSdk == null && localEngineHost == null) {
      stderr.writeln('When running in A/B test mode for mobile --local-engine-host is required.\n');
      stderr.writeln(argParser.usage);
      exit(1);
    }
    await _runABTest(
      runsPerTest: runsPerTest,
      silent: silent,
      localEngine: localEngine,
      localEngineHost: localEngineHost,
      localWebSdk: localWebSdk,
      localEngineSrcPath: localEngineSrcPath,
      deviceId: deviceId,
      resultsFile: resultsFile,
      taskName: taskNames.single,
      onlyLocalEngine: (args['ab-local-engine-only'] as bool?) ?? false,
    );
  } else {
    await runTasks(
      taskNames,
      silent: silent,
      localEngine: localEngine,
      localEngineHost: localEngineHost,
      localEngineSrcPath: localEngineSrcPath,
      deviceId: deviceId,
      exitOnFirstTestFailure: exitOnFirstTestFailure,
      terminateStrayDartProcesses: terminateStrayDartProcesses,
      gitBranch: gitBranch,
      luciBuilder: luciBuilder,
      resultsPath: resultsPath,
      useEmulator: useEmulator,
    );
  }
}

Future<void> _runABTest({
  required int runsPerTest,
  required bool silent,
  required String? localEngine,
  required String? localEngineHost,
  required String? localWebSdk,
  required String? localEngineSrcPath,
  required String? deviceId,
  required String resultsFile,
  required String taskName,
  bool onlyLocalEngine = false,
}) async {
  print('$taskName A/B test. Will run $runsPerTest times.');

  assert(localEngine != null || localWebSdk != null);

  final ABTest abTest = ABTest(
    localEngine: (localEngine ?? localWebSdk)!,
    localEngineHost: localEngineHost,
    taskName: taskName,
  );
  for (int i = 1; i <= runsPerTest; i++) {
    section('Run #$i');

    if (onlyLocalEngine) {
      print('Skipping default engine (A)');
    } else {
      print('Running with the default engine (A)');
      final TaskResult defaultEngineResult = await runTask(
        taskName,
        silent: silent,
        deviceId: deviceId,
      );

      print('Default engine result:');
      print(const JsonEncoder.withIndent('  ').convert(defaultEngineResult));

      if (!defaultEngineResult.succeeded) {
        stderr.writeln('Task failed on the default engine.');
        exit(1);
      }

      abTest.addAResult(defaultEngineResult);
    }

    print('Running with the local engine (B)');
    final TaskResult localEngineResult = await runTask(
      taskName,
      silent: silent,
      localEngine: localEngine,
      localEngineHost: localEngineHost,
      localWebSdk: localWebSdk,
      localEngineSrcPath: localEngineSrcPath,
      deviceId: deviceId,
    );

    print('Task localEngineResult:');
    print(const JsonEncoder.withIndent('  ').convert(localEngineResult));

    if (!localEngineResult.succeeded) {
      stderr.writeln('Task failed on the local engine.');
      exit(1);
    }

    abTest.addBResult(localEngineResult);

    if (!silent && i < runsPerTest) {
      section('A/B results so far');
      print(abTest.printSummary());
    }
  }
  abTest.finalize();

  final File jsonFile = _uniqueFile(resultsFile);
  jsonFile.writeAsStringSync(const JsonEncoder.withIndent('  ').convert(abTest.jsonMap));

  if (!silent) {
    section('Raw results');
    print(abTest.rawResults());
  }

  section('Final A/B results');
  print(abTest.printSummary());

  print('');
  print('Results saved to ${jsonFile.path}');
}

File _uniqueFile(String filenameTemplate) {
  final List<String> parts = filenameTemplate.split('#');
  if (parts.length != 2) {
    return File(filenameTemplate);
  }
  File file = File(parts[0] + parts[1]);
  int i = 1;
  while (file.existsSync()) {
    file = File(parts[0] + i.toString() + parts[1]);
    i++;
  }
  return file;
}

ArgParser createArgParser(List<String> taskNames) {
  return ArgParser()
    ..addMultiOption(
      'task',
      abbr: 't',
      help:
          'Name of a Dart file in bin/tasks.\n'
          '   Example: complex_layout__start_up\n'
          '\n'
          'This option may be repeated to specify multiple tasks.',
      callback: (List<String> tasks) {
        taskNames.addAll(tasks);
      },
    )
    ..addOption(
      'device-id',
      abbr: 'd',
      help:
          'Target device id (prefixes are allowed, names are not supported).\n'
          'The option will be ignored if the test target does not run on a\n'
          'mobile device. This still respects the device operating system\n'
          'settings in the test case, and will results in error if no device\n'
          'with given ID/ID prefix is found.',
    )
    ..addOption(
      'ab',
      help:
          'Runs an A/B test comparing the default engine with the local\n'
          'engine build for one task. This option does not support running\n'
          'multiple tasks. The value is the number of times to run the task.\n'
          'The task is expected to be a benchmark that reports score keys.\n'
          'The A/B test collects the metrics collected by the test and\n'
          'produces a report containing averages, noise, and the speed-up\n'
          'between the two engines. --local-engine is required when running\n'
          'an A/B test.',
      callback: (String? value) {
        if (value != null && int.tryParse(value) == null) {
          throw ArgParserException('Option --ab must be a number, but was "$value".');
        }
      },
    )
    ..addOption(
      'ab-result-file',
      help:
          'The filename in which to place the json encoded results of an A/B test.\n'
          'The filename may contain a single # character to be replaced by a sequence\n'
          'number if the name already exists.',
    )
    ..addFlag(
      'ab-local-engine-only',
      help:
          'When running the A/B aggregator, do not run benchmarks with the default engine (A), only the local engine (B).\n'
          'Shows the averages and noise report for the local engine without comparison to anything else.',
    )
    ..addFlag(
      'exit',
      help:
          'Exit on the first test failure. Currently flakes are intentionally (though '
          'incorrectly) not considered to be failures.',
    )
    ..addOption(
      'git-branch',
      help:
          '[Flutter infrastructure] Git branch of the current commit. LUCI\n'
          'checkouts run in detached HEAD state, so the branch must be passed.',
    )
    ..addOption(
      'local-engine',
      help:
          'Name of a build output within the engine out directory, if you\n'
          'are building Flutter locally. Use this to select a specific\n'
          'version of the engine if you have built multiple engine targets.\n'
          'This path is relative to --local-engine-src-path/out. This option\n'
          'is required when running an A/B test (see the --ab option).',
    )
    ..addOption(
      'local-engine-host',
      help:
          'Name of a build output within the engine out directory, if you\n'
          'are building Flutter locally. Use this to select a specific\n'
          'version of the engine to use as the host platform if you have built '
          'multiple engine targets.\n'
          'This path is relative to --local-engine-src-path/out. This option\n'
          'is required when running an A/B test (see the --ab option).',
    )
    ..addOption(
      'local-web-sdk',
      help:
          'Name of a build output within the engine out directory, if you\n'
          'are building Flutter locally. Use this to select a specific\n'
          'version of the engine if you have built multiple engine targets.\n'
          'This path is relative to --local-engine-src-path/out. This option\n'
          'is required when running an A/B test (see the --ab option).',
    )
    ..addFlag(
      'list',
      abbr: 'l',
      help:
          "Don't actually run the tasks, but list out the tasks that would\n"
          'have been run, in the order they would have run.',
    )
    ..addOption(
      'local-engine-src-path',
      help:
          'Path to your engine src directory, if you are building Flutter\n'
          'locally. Defaults to \$FLUTTER_ENGINE if set, or tries to guess at\n'
          'the location based on the value of the --flutter-root option.',
    )
    ..addOption(
      'luci-builder',
      help: '[Flutter infrastructure] Name of the LUCI builder being run on.',
    )
    ..addOption(
      'results-file',
      help:
          '[Flutter infrastructure] File path for test results. If passed with\n'
          'task, will write test results to the file.',
    )
    ..addOption(
      'service-account-token-file',
      help: '[Flutter infrastructure] Authentication for uploading results.',
    )
    ..addFlag('silent', help: 'Reduce verbosity slightly.')
    ..addFlag(
      'terminate-stray-dart-processes',
      defaultsTo: true,
      help:
          'Whether to send a SIGKILL signal to any Dart processes that are still '
          'running when a task is completed. If any Dart processes are terminated '
          'in this way, the test is considered to have failed.',
    )
    ..addFlag(
      'use-emulator',
      help:
          'If this is an android test, use an emulator to run the test instead of '
          'a physical device.',
    )
    ..addMultiOption(
      'test',
      hide: true,
      callback: (List<String> value) {
        if (value.isNotEmpty) {
          throw const FormatException('Invalid option --test. Did you mean --task (-t)?');
        }
      },
    );
}
