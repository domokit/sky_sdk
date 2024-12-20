// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// Runs the tests for the flutter/flutter repository.
//
// By default, test output is filtered and only errors are shown. (If a
// particular test takes longer than _quietTimeout in utils.dart, the output is
// shown then also, in case something has hung.)
//
//  --verbose stops the output cleanup and just outputs everything verbatim.
//
// By default, errors are non-fatal; all tests are executed and the output
// ends with a summary of the errors that were detected.
//
// Exit code is 1 if there was an error.
//
//  --abort-on-error causes the script to exit immediately when hitting an error.
//
// By default, all tests are run. However, the tests support being split by
// shard and subshard. Inspect the code to see what shards and subshards are
// supported, or run with `--dry-run` to get a list of tests that _would_ have
// been executed.
//
// If the CIRRUS_TASK_NAME environment variable exists, it is used to determine
// the shard and sub-shard, by parsing it in the form shard-subshard-platform,
// ignoring the platform.
//
// For local testing you can just set the SHARD and SUBSHARD environment
// variables. For example, to run all the framework tests you can just set
// SHARD=framework_tests. Some shards support named subshards, like
// SHARD=framework_tests SUBSHARD=widgets. Others support arbitrary numbered
// subsharding, like SHARD=build_tests SUBSHARD=1_2 (where 1_2 means "one of
// two" as in run the first half of the tests).
//
// So for example to run specifically the third subshard of the Web tests you
// would set SHARD=web_tests SUBSHARD=2 (it's zero-based).
//
// By default, where supported, tests within a shard are executed in a random
// order to (eventually) catch inter-test dependencies.
//
//  --test-randomize-ordering-seed=<n> sets the shuffle seed for reproducing runs.
//
// All other arguments are treated as arguments to pass to the flutter tool when
// running tests.

import 'dart:convert';
import 'dart:core' as system show print;
import 'dart:core' hide print;
import 'dart:io' as system show exit;
import 'dart:io' hide exit;
import 'dart:math' as math;

import 'package:path/path.dart' as path;

import 'run_command.dart';
import 'suite_runners/run_add_to_app_life_cycle_tests.dart';
import 'suite_runners/run_analyze_tests.dart';
import 'suite_runners/run_android_java11_integration_tool_tests.dart';
import 'suite_runners/run_android_preview_integration_tool_tests.dart';
import 'suite_runners/run_customer_testing_tests.dart';
import 'suite_runners/run_docs_tests.dart';
import 'suite_runners/run_flutter_driver_android_tests.dart';
import 'suite_runners/run_flutter_packages_tests.dart';
import 'suite_runners/run_framework_coverage_tests.dart';
import 'suite_runners/run_framework_tests.dart';
import 'suite_runners/run_fuchsia_precache.dart';
import 'suite_runners/run_skp_generator_tests.dart';
import 'suite_runners/run_test_harness_tests.dart';
import 'suite_runners/run_verify_binaries_codesigned_tests.dart';
import 'suite_runners/run_web_tests.dart';
import 'utils.dart';

typedef ShardRunner = Future<void> Function();

/// Environment variables to override the local engine when running `pub test`,
/// if such flags are provided to `test.dart`.
final Map<String, String> localEngineEnv = <String, String>{};

const String CIRRUS_TASK_NAME = 'CIRRUS_TASK_NAME';

/// When you call this, you can pass additional arguments to pass custom
/// arguments to flutter test. For example, you might want to call this
/// script with the parameter --local-engine=host_debug_unopt to
/// use your own build of the engine.
///
/// To run the tool_tests part, run it with SHARD=tool_tests
///
/// Examples:
/// SHARD=tool_tests bin/cache/dart-sdk/bin/dart dev/bots/test.dart
/// bin/cache/dart-sdk/bin/dart dev/bots/test.dart --local-engine=host_debug_unopt --local-engine-host=host_debug_unopt
Future<void> main(List<String> args) async {
  try {
    printProgress('STARTING ANALYSIS');
    bool dryRunArgSet = false;
    for (final String arg in args) {
      if (arg.startsWith('--local-engine=')) {
        localEngineEnv['FLUTTER_LOCAL_ENGINE'] = arg.substring('--local-engine='.length);
        flutterTestArgs.add(arg);
      } else if (arg.startsWith('--local-engine-host=')) {
        localEngineEnv['FLUTTER_LOCAL_ENGINE_HOST'] = arg.substring('--local-engine-host='.length);
        flutterTestArgs.add(arg);
      } else if (arg.startsWith('--local-engine-src-path=')) {
        localEngineEnv['FLUTTER_LOCAL_ENGINE_SRC_PATH'] = arg.substring(
          '--local-engine-src-path='.length,
        );
        flutterTestArgs.add(arg);
      } else if (arg.startsWith('--test-randomize-ordering-seed=')) {
        shuffleSeed = arg.substring('--test-randomize-ordering-seed='.length);
      } else if (arg.startsWith('--verbose')) {
        print = (Object? message) {
          system.print(message);
        };
      } else if (arg.startsWith('--abort-on-error')) {
        onError = () {
          system.exit(1);
        };
      } else if (arg == '--dry-run') {
        dryRunArgSet = true;
        printProgress('--dry-run enabled. Tests will not actually be executed.');
      } else {
        flutterTestArgs.add(arg);
      }
    }
    if (dryRunArgSet) {
      enableDryRun();
    }
    if (Platform.environment.containsKey(CIRRUS_TASK_NAME)) {
      printProgress('Running task: ${Platform.environment[CIRRUS_TASK_NAME]}');
    }
    final WebTestsSuite webTestsSuite = WebTestsSuite(flutterTestArgs);
    await selectShard(<String, ShardRunner>{
      'add_to_app_life_cycle_tests': addToAppLifeCycleRunner,
      'build_tests': _runBuildTests,
      'framework_coverage': frameworkCoverageRunner,
      'framework_tests': frameworkTestsRunner,
      'tool_tests': _runToolTests,
      'web_tool_tests': _runWebToolTests,
      'tool_integration_tests': _runIntegrationToolTests,
      'android_preview_tool_integration_tests': androidPreviewIntegrationToolTestsRunner,
      'android_java11_tool_integration_tests': androidJava11IntegrationToolTestsRunner,
      'tool_host_cross_arch_tests': _runToolHostCrossArchTests,
      // All the unit/widget tests run using `flutter test --platform=chrome --web-renderer=html`
      'web_tests': webTestsSuite.runWebHtmlUnitTests,
      // All the unit/widget tests run using `flutter test --platform=chrome --web-renderer=canvaskit`
      'web_canvaskit_tests': webTestsSuite.runWebCanvasKitUnitTests,
      // All the unit/widget tests run using `flutter test --platform=chrome --wasm --web-renderer=skwasm`
      'web_skwasm_tests': webTestsSuite.runWebSkwasmUnitTests,
      // All web integration tests
      'web_long_running_tests': webTestsSuite.webLongRunningTestsRunner,
      'flutter_driver_android': runFlutterDriverAndroidTests,
      'flutter_plugins': flutterPackagesRunner,
      'skp_generator': skpGeneratorTestsRunner,
      'customer_testing': customerTestingRunner,
      'analyze': analyzeRunner,
      'fuchsia_precache': fuchsiaPrecacheRunner,
      'snippets': _runSnippetsTests,
      'docs': docsRunner,
      'verify_binaries_codesigned': verifyCodesignedTestRunner,
      kTestHarnessShardName:
          testHarnessTestsRunner, // Used for testing this script; also run as part of SHARD=framework_tests, SUBSHARD=misc.
    });
  } catch (error, stackTrace) {
    foundError(<String>[
      'UNEXPECTED ERROR!',
      error.toString(),
      ...stackTrace.toString().split('\n'),
      'The test.dart script should be corrected to catch this error and call foundError().',
      '${yellow}Some tests are likely to have been skipped.$reset',
    ]);
    system.exit(255);
  }
  if (hasError) {
    reportErrorsAndExit('${bold}Test failed.$reset');
  }
  reportSuccessAndExit('${bold}Test successful.$reset');
}

final String _toolsPath = path.join(flutterRoot, 'packages', 'flutter_tools');

Future<void> _runGeneralToolTests() async {
  await runDartTest(
    _toolsPath,
    testPaths: <String>[path.join('test', 'general.shard')],
    enableFlutterToolAsserts: false,

    // Detect unit test time regressions (poor time delay handling, etc).
    // This overrides the 15 minute default for tools tests.
    // See the README.md and dart_test.yaml files in the flutter_tools package.
    perTestTimeout: const Duration(seconds: 2),
  );
}

Future<void> _runCommandsToolTests() async {
  await runDartTest(
    _toolsPath,
    forceSingleCore: true,
    testPaths: <String>[path.join('test', 'commands.shard')],
  );
}

Future<void> _runWebToolTests() async {
  final List<File> allFiles =
      Directory(
        path.join(_toolsPath, 'test', 'web.shard'),
      ).listSync(recursive: true).whereType<File>().toList();
  final List<String> allTests = <String>[];
  for (final File file in allFiles) {
    if (file.path.endsWith('_test.dart')) {
      allTests.add(file.path);
    }
  }
  await runDartTest(
    _toolsPath,
    forceSingleCore: true,
    testPaths: selectIndexOfTotalSubshard<String>(allTests),
    includeLocalEngineEnv: true,
  );
}

Future<void> _runToolHostCrossArchTests() {
  return runDartTest(
    _toolsPath,
    // These are integration tests
    forceSingleCore: true,
    testPaths: <String>[path.join('test', 'host_cross_arch.shard')],
  );
}

Future<void> _runIntegrationToolTests() async {
  final List<String> allTests =
      Directory(path.join(_toolsPath, 'test', 'integration.shard'))
          .listSync(recursive: true)
          .whereType<File>()
          .map<String>((FileSystemEntity entry) => path.relative(entry.path, from: _toolsPath))
          .where((String testPath) => path.basename(testPath).endsWith('_test.dart'))
          .toList();

  await runDartTest(
    _toolsPath,
    forceSingleCore: true,
    testPaths: selectIndexOfTotalSubshard<String>(allTests),
    collectMetrics: true,
  );
}

Future<void> _runToolTests() async {
  await selectSubshard(<String, ShardRunner>{
    'general': _runGeneralToolTests,
    'commands': _runCommandsToolTests,
  });
}

Future<void> _runSnippetsTests() async {
  final String snippetsPath = path.join(flutterRoot, 'dev', 'snippets');
  final List<String> allTests =
      Directory(path.join(snippetsPath, 'test'))
          .listSync(recursive: true)
          .whereType<File>()
          .map<String>((FileSystemEntity entry) => path.relative(entry.path, from: _toolsPath))
          .where((String testPath) => path.basename(testPath).endsWith('_test.dart'))
          .toList();

  await runDartTest(
    snippetsPath,
    forceSingleCore: true,
    testPaths: selectIndexOfTotalSubshard<String>(allTests),
    collectMetrics: true,
  );
}

Future<void> runForbiddenFromReleaseTests() async {
  // Build a release APK to get the snapshot json.
  final Directory tempDirectory = Directory.systemTemp.createTempSync('flutter_forbidden_imports.');
  final List<String> command = <String>[
    'build',
    'apk',
    '--target-platform',
    'android-arm64',
    '--release',
    '--analyze-size',
    '--code-size-directory',
    tempDirectory.path,
    '-v',
  ];

  await runCommand(
    flutter,
    command,
    workingDirectory: path.join(flutterRoot, 'examples', 'hello_world'),
  );

  // First, a smoke test.
  final List<String> smokeTestArgs = <String>[
    path.join(flutterRoot, 'dev', 'forbidden_from_release_tests', 'bin', 'main.dart'),
    '--snapshot',
    path.join(tempDirectory.path, 'snapshot.arm64-v8a.json'),
    '--package-config',
    path.join(flutterRoot, 'examples', 'hello_world', '.dart_tool', 'package_config.json'),
    '--forbidden-type',
    'package:flutter/src/widgets/framework.dart::Widget',
  ];
  await runCommand(dart, smokeTestArgs, workingDirectory: flutterRoot, expectNonZeroExit: true);

  // Actual test.
  final List<String> args = <String>[
    path.join(flutterRoot, 'dev', 'forbidden_from_release_tests', 'bin', 'main.dart'),
    '--snapshot',
    path.join(tempDirectory.path, 'snapshot.arm64-v8a.json'),
    '--package-config',
    path.join(flutterRoot, 'examples', 'hello_world', '.dart_tool', 'package_config.json'),
    '--forbidden-type',
    'package:flutter/src/widgets/widget_inspector.dart::WidgetInspectorService',
    '--forbidden-type',
    'package:flutter/src/widgets/framework.dart::DebugCreator',
    '--forbidden-type',
    'package:flutter/src/foundation/print.dart::debugPrint',
  ];
  await runCommand(dart, args, workingDirectory: flutterRoot);
}

/// Verifies that APK, and IPA (if on macOS), and native desktop builds the
/// examples apps without crashing. It does not actually launch the apps. That
/// happens later in the devicelab. This is just a smoke-test. In particular,
/// this will verify we can build when there are spaces in the path name for the
/// Flutter SDK and target app.
///
/// Also does some checking about types included in hello_world.
Future<void> _runBuildTests() async {
  final List<Directory> exampleDirectories =
      Directory(path.join(flutterRoot, 'examples'))
          .listSync()
          // API example builds will be tested in a separate shard.
          .where(
            (FileSystemEntity entity) => entity is Directory && path.basename(entity.path) != 'api',
          )
          .cast<Directory>()
          .toList()
        ..add(Directory(path.join(flutterRoot, 'packages', 'integration_test', 'example')))
        ..add(
          Directory(
            path.join(flutterRoot, 'dev', 'integration_tests', 'android_semantics_testing'),
          ),
        )
        ..add(Directory(path.join(flutterRoot, 'dev', 'integration_tests', 'android_views')))
        ..add(Directory(path.join(flutterRoot, 'dev', 'integration_tests', 'channels')))
        ..add(Directory(path.join(flutterRoot, 'dev', 'integration_tests', 'hybrid_android_views')))
        ..add(Directory(path.join(flutterRoot, 'dev', 'integration_tests', 'flutter_gallery')))
        ..add(
          Directory(path.join(flutterRoot, 'dev', 'integration_tests', 'ios_platform_view_tests')),
        )
        ..add(
          Directory(path.join(flutterRoot, 'dev', 'integration_tests', 'ios_app_with_extensions')),
        )
        ..add(Directory(path.join(flutterRoot, 'dev', 'integration_tests', 'non_nullable')))
        ..add(Directory(path.join(flutterRoot, 'dev', 'integration_tests', 'platform_interaction')))
        ..add(Directory(path.join(flutterRoot, 'dev', 'integration_tests', 'spell_check')))
        ..add(Directory(path.join(flutterRoot, 'dev', 'integration_tests', 'ui')));

  // The tests are randomly distributed into subshards so as to get a uniform
  // distribution of costs, but the seed is fixed so that issues are reproducible.
  final List<ShardRunner> tests = <ShardRunner>[
    for (final Directory exampleDirectory in exampleDirectories)
      () => _runExampleProjectBuildTests(exampleDirectory),
    ...<ShardRunner>[
      // Web compilation tests.
      () => _flutterBuildDart2js(
        path.join('dev', 'integration_tests', 'web'),
        path.join('lib', 'main.dart'),
      ),
      // Should not fail to compile with dart:io.
      () => _flutterBuildDart2js(
        path.join('dev', 'integration_tests', 'web_compile_tests'),
        path.join('lib', 'dart_io_import.dart'),
      ),
      // Should be able to compile with a call to:
      // BackgroundIsolateBinaryMessenger.ensureInitialized.
      () => _flutterBuildDart2js(
        path.join('dev', 'integration_tests', 'web_compile_tests'),
        path.join('lib', 'background_isolate_binary_messenger.dart'),
      ),
    ],
    runForbiddenFromReleaseTests,
  ]..shuffle(math.Random(0));

  await runShardRunnerIndexOfTotalSubshard(tests);
}

Future<void> _runExampleProjectBuildTests(Directory exampleDirectory, [File? mainFile]) async {
  // Only verify caching with flutter gallery.
  final bool verifyCaching = exampleDirectory.path.contains('flutter_gallery');
  final String examplePath = path.relative(exampleDirectory.path, from: Directory.current.path);
  final List<String> additionalArgs = <String>[
    if (mainFile != null) path.relative(mainFile.path, from: exampleDirectory.absolute.path),
  ];
  if (Directory(path.join(examplePath, 'android')).existsSync()) {
    await _flutterBuildApk(
      examplePath,
      release: false,
      additionalArgs: additionalArgs,
      verifyCaching: verifyCaching,
    );
    await _flutterBuildApk(
      examplePath,
      release: true,
      additionalArgs: additionalArgs,
      verifyCaching: verifyCaching,
    );
  } else {
    print('Example project ${path.basename(examplePath)} has no android directory, skipping apk');
  }
  if (Platform.isMacOS) {
    if (Directory(path.join(examplePath, 'ios')).existsSync()) {
      await _flutterBuildIpa(
        examplePath,
        release: false,
        additionalArgs: additionalArgs,
        verifyCaching: verifyCaching,
      );
      await _flutterBuildIpa(
        examplePath,
        release: true,
        additionalArgs: additionalArgs,
        verifyCaching: verifyCaching,
      );
    } else {
      print('Example project ${path.basename(examplePath)} has no ios directory, skipping ipa');
    }
  }
  if (Platform.isLinux) {
    if (Directory(path.join(examplePath, 'linux')).existsSync()) {
      await _flutterBuildLinux(
        examplePath,
        release: false,
        additionalArgs: additionalArgs,
        verifyCaching: verifyCaching,
      );
      await _flutterBuildLinux(
        examplePath,
        release: true,
        additionalArgs: additionalArgs,
        verifyCaching: verifyCaching,
      );
    } else {
      print('Example project ${path.basename(examplePath)} has no linux directory, skipping Linux');
    }
  }
  if (Platform.isMacOS) {
    if (Directory(path.join(examplePath, 'macos')).existsSync()) {
      await _flutterBuildMacOS(
        examplePath,
        release: false,
        additionalArgs: additionalArgs,
        verifyCaching: verifyCaching,
      );
      await _flutterBuildMacOS(
        examplePath,
        release: true,
        additionalArgs: additionalArgs,
        verifyCaching: verifyCaching,
      );
    } else {
      print('Example project ${path.basename(examplePath)} has no macos directory, skipping macOS');
    }
  }
  if (Platform.isWindows) {
    if (Directory(path.join(examplePath, 'windows')).existsSync()) {
      await _flutterBuildWin32(
        examplePath,
        release: false,
        additionalArgs: additionalArgs,
        verifyCaching: verifyCaching,
      );
      await _flutterBuildWin32(
        examplePath,
        release: true,
        additionalArgs: additionalArgs,
        verifyCaching: verifyCaching,
      );
    } else {
      print(
        'Example project ${path.basename(examplePath)} has no windows directory, skipping Win32',
      );
    }
  }
}

Future<void> _flutterBuildApk(
  String relativePathToApplication, {
  required bool release,
  bool verifyCaching = false,
  List<String> additionalArgs = const <String>[],
}) async {
  printProgress(
    '${green}Testing APK ${release ? 'release' : 'debug'} build$reset for $cyan$relativePathToApplication$reset...',
  );
  await _flutterBuild(
    relativePathToApplication,
    'APK',
    'apk',
    release: release,
    verifyCaching: verifyCaching,
    additionalArgs: additionalArgs,
  );
}

Future<void> _flutterBuildIpa(
  String relativePathToApplication, {
  required bool release,
  List<String> additionalArgs = const <String>[],
  bool verifyCaching = false,
}) async {
  assert(Platform.isMacOS);
  printProgress(
    '${green}Testing IPA ${release ? 'release' : 'debug'} build$reset for $cyan$relativePathToApplication$reset...',
  );
  await _flutterBuild(
    relativePathToApplication,
    'IPA',
    'ios',
    release: release,
    verifyCaching: verifyCaching,
    additionalArgs: <String>[...additionalArgs, '--no-codesign'],
  );
}

Future<void> _flutterBuildLinux(
  String relativePathToApplication, {
  required bool release,
  bool verifyCaching = false,
  List<String> additionalArgs = const <String>[],
}) async {
  assert(Platform.isLinux);
  await runCommand(flutter, <String>['config', '--enable-linux-desktop']);
  printProgress(
    '${green}Testing Linux ${release ? 'release' : 'debug'} build$reset for $cyan$relativePathToApplication$reset...',
  );
  await _flutterBuild(
    relativePathToApplication,
    'Linux',
    'linux',
    release: release,
    verifyCaching: verifyCaching,
    additionalArgs: additionalArgs,
  );
}

Future<void> _flutterBuildMacOS(
  String relativePathToApplication, {
  required bool release,
  bool verifyCaching = false,
  List<String> additionalArgs = const <String>[],
}) async {
  assert(Platform.isMacOS);
  await runCommand(flutter, <String>['config', '--enable-macos-desktop']);
  printProgress(
    '${green}Testing macOS ${release ? 'release' : 'debug'} build$reset for $cyan$relativePathToApplication$reset...',
  );
  await _flutterBuild(
    relativePathToApplication,
    'macOS',
    'macos',
    release: release,
    verifyCaching: verifyCaching,
    additionalArgs: additionalArgs,
  );
}

Future<void> _flutterBuildWin32(
  String relativePathToApplication, {
  required bool release,
  bool verifyCaching = false,
  List<String> additionalArgs = const <String>[],
}) async {
  assert(Platform.isWindows);
  printProgress(
    '${green}Testing ${release ? 'release' : 'debug'} Windows build$reset for $cyan$relativePathToApplication$reset...',
  );
  await _flutterBuild(
    relativePathToApplication,
    'Windows',
    'windows',
    release: release,
    verifyCaching: verifyCaching,
    additionalArgs: additionalArgs,
  );
}

Future<void> _flutterBuild(
  String relativePathToApplication,
  String platformLabel,
  String platformBuildName, {
  required bool release,
  bool verifyCaching = false,
  List<String> additionalArgs = const <String>[],
}) async {
  await runCommand(flutter, <String>[
    'build',
    platformBuildName,
    ...additionalArgs,
    if (release) '--release' else '--debug',
    '-v',
  ], workingDirectory: path.join(flutterRoot, relativePathToApplication));

  if (verifyCaching) {
    printProgress(
      '${green}Testing $platformLabel cache$reset for $cyan$relativePathToApplication$reset...',
    );
    await runCommand(flutter, <String>[
      'build',
      platformBuildName,
      '--performance-measurement-file=perf.json',
      ...additionalArgs,
      if (release) '--release' else '--debug',
      '-v',
    ], workingDirectory: path.join(flutterRoot, relativePathToApplication));
    final File file = File(path.join(flutterRoot, relativePathToApplication, 'perf.json'));
    if (!_allTargetsCached(file)) {
      foundError(<String>[
        '${red}Not all build targets cached after second run.$reset',
        'The target performance data was: ${file.readAsStringSync().replaceAll('},', '},\n')}',
      ]);
    }
  }
}

bool _allTargetsCached(File performanceFile) {
  if (dryRun) {
    return true;
  }
  final Map<String, Object?> data =
      json.decode(performanceFile.readAsStringSync()) as Map<String, Object?>;
  final List<Map<String, Object?>> targets =
      (data['targets']! as List<Object?>).cast<Map<String, Object?>>();
  return targets.every((Map<String, Object?> element) => element['skipped'] == true);
}

Future<void> _flutterBuildDart2js(
  String relativePathToApplication,
  String target, {
  bool expectNonZeroExit = false,
}) async {
  printProgress('${green}Testing Dart2JS build$reset for $cyan$relativePathToApplication$reset...');
  await runCommand(
    flutter,
    <String>['build', 'web', '-v', '--target=$target'],
    workingDirectory: path.join(flutterRoot, relativePathToApplication),
    expectNonZeroExit: expectNonZeroExit,
    environment: <String, String>{'FLUTTER_WEB': 'true'},
  );
}
