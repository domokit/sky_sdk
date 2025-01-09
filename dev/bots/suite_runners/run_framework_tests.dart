// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert';
import 'dart:io' show Directory, File, FileSystemEntity, Platform, Process;
import 'dart:typed_data';

import 'package:archive/archive.dart';
import 'package:path/path.dart' as path;

import '../run_command.dart';
import '../utils.dart';
import 'run_test_harness_tests.dart';

Future<void> frameworkTestsRunner() async {
  final List<String> trackWidgetCreationAlternatives = <String>[
    '--track-widget-creation',
    '--no-track-widget-creation',
  ];

  Future<void> runWidgets() async {
    printProgress('${green}Running packages/flutter tests $reset for ${cyan}test/widgets/$reset');
    for (final String trackWidgetCreationOption in trackWidgetCreationAlternatives) {
      await runFlutterTest(
        path.join(flutterRoot, 'packages', 'flutter'),
        options: <String>[trackWidgetCreationOption],
        tests: <String>[path.join('test', 'widgets') + path.separator],
      );
    }
    // Try compiling code outside of the packages/flutter directory with and without --track-widget-creation
    for (final String trackWidgetCreationOption in trackWidgetCreationAlternatives) {
      await runFlutterTest(
        path.join(flutterRoot, 'dev', 'integration_tests', 'flutter_gallery'),
        options: <String>[trackWidgetCreationOption],
        fatalWarnings: false, // until we've migrated video_player
      );
    }
    // Run release mode tests (see packages/flutter/test_release/README.md)
    await runFlutterTest(
      path.join(flutterRoot, 'packages', 'flutter'),
      options: <String>['--dart-define=dart.vm.product=true'],
      tests: <String>['test_release${path.separator}'],
    );
    // Run profile mode tests (see packages/flutter/test_profile/README.md)
    await runFlutterTest(
      path.join(flutterRoot, 'packages', 'flutter'),
      options: <String>[
        '--dart-define=dart.vm.product=false',
        '--dart-define=dart.vm.profile=true',
      ],
      tests: <String>['test_profile${path.separator}'],
    );
  }

  Future<void> runImpeller() async {
    printProgress('${green}Running packages/flutter tests $reset in Impeller$reset');
    await runFlutterTest(
      path.join(flutterRoot, 'packages', 'flutter'),
      options: <String>['--enable-impeller'],
    );
  }

  Future<void> runLibraries() async {
    final List<String> tests =
        Directory(path.join(flutterRoot, 'packages', 'flutter', 'test'))
            .listSync(followLinks: false)
            .whereType<Directory>()
            .where((Directory dir) => !dir.path.endsWith('widgets'))
            .map<String>(
              (Directory dir) => path.join('test', path.basename(dir.path)) + path.separator,
            )
            .toList();
    printProgress(
      '${green}Running packages/flutter tests$reset for $cyan${tests.join(", ")}$reset',
    );
    for (final String trackWidgetCreationOption in trackWidgetCreationAlternatives) {
      await runFlutterTest(
        path.join(flutterRoot, 'packages', 'flutter'),
        options: <String>[trackWidgetCreationOption],
        tests: tests,
      );
    }
  }

  Future<void> runExampleTests() async {
    await runCommand(flutter, <String>[
      'config',
      '--enable-${Platform.operatingSystem}-desktop',
    ], workingDirectory: flutterRoot);
    await runCommand(dart, <String>[
      path.join(flutterRoot, 'dev', 'tools', 'examples_smoke_test.dart'),
    ], workingDirectory: path.join(flutterRoot, 'examples', 'api'));
    for (final FileSystemEntity entity
        in Directory(path.join(flutterRoot, 'examples')).listSync()) {
      if (entity is! Directory || !Directory(path.join(entity.path, 'test')).existsSync()) {
        continue;
      }
      await runFlutterTest(entity.path);
    }
  }

  Future<void> runTracingTests() async {
    final String tracingDirectory = path.join(flutterRoot, 'dev', 'tracing_tests');

    // run the tests for debug mode
    await runFlutterTest(tracingDirectory, options: <String>['--enable-vmservice']);

    Future<List<String>> verifyTracingAppBuild({
      required String modeArgument,
      required String sourceFile,
      required Set<String> allowed,
      required Set<String> disallowed,
    }) async {
      try {
        await runCommand(flutter, <String>[
          'build',
          'appbundle',
          '--$modeArgument',
          path.join('lib', sourceFile),
        ], workingDirectory: tracingDirectory);
        final Archive archive = ZipDecoder().decodeBytes(
          File(
            path.join(
              tracingDirectory,
              'build',
              'app',
              'outputs',
              'bundle',
              modeArgument,
              'app-$modeArgument.aab',
            ),
          ).readAsBytesSync(),
        );
        final ArchiveFile libapp = archive.findFile('base/lib/arm64-v8a/libapp.so')!;
        final Uint8List libappBytes = libapp.content as Uint8List; // bytes decompressed here
        final String libappStrings = utf8.decode(libappBytes, allowMalformed: true);
        await runCommand(flutter, <String>['clean'], workingDirectory: tracingDirectory);
        final List<String> results = <String>[];
        for (final String pattern in allowed) {
          if (!libappStrings.contains(pattern)) {
            results.add(
              'When building with --$modeArgument, expected to find "$pattern" in libapp.so but could not find it.',
            );
          }
        }
        for (final String pattern in disallowed) {
          if (libappStrings.contains(pattern)) {
            results.add(
              'When building with --$modeArgument, expected to not find "$pattern" in libapp.so but did find it.',
            );
          }
        }
        return results;
      } catch (error, stackTrace) {
        return <String>[error.toString(), ...stackTrace.toString().trimRight().split('\n')];
      }
    }

    final List<String> results = <String>[];
    results.addAll(
      await verifyTracingAppBuild(
        modeArgument: 'profile',
        sourceFile: 'control.dart', // this is the control, the other two below are the actual test
        allowed: <String>{
          'TIMELINE ARGUMENTS TEST CONTROL FILE',
          'toTimelineArguments used in non-debug build', // we call toTimelineArguments directly to check the message does exist
        },
        disallowed: <String>{'BUILT IN DEBUG MODE', 'BUILT IN RELEASE MODE'},
      ),
    );
    results.addAll(
      await verifyTracingAppBuild(
        modeArgument: 'profile',
        sourceFile: 'test.dart',
        allowed: <String>{
          'BUILT IN PROFILE MODE', 'RenderTest.performResize called', // controls
          'BUILD', 'LAYOUT', 'PAINT', // we output these to the timeline in profile builds
          // (LAYOUT and PAINT also exist because of NEEDS-LAYOUT and NEEDS-PAINT in RenderObject.toStringShort)
        },
        disallowed: <String>{
          'BUILT IN DEBUG MODE', 'BUILT IN RELEASE MODE',
          'TestWidget.debugFillProperties called',
          'RenderTest.debugFillProperties called', // debug only
          'toTimelineArguments used in non-debug build', // entire function should get dropped by tree shaker
        },
      ),
    );
    results.addAll(
      await verifyTracingAppBuild(
        modeArgument: 'release',
        sourceFile: 'test.dart',
        allowed: <String>{
          'BUILT IN RELEASE MODE', 'RenderTest.performResize called', // controls
        },
        disallowed: <String>{
          'BUILT IN DEBUG MODE', 'BUILT IN PROFILE MODE',
          'BUILD',
          'LAYOUT',
          'PAINT', // these are only used in Timeline.startSync calls that should not appear in release builds
          'TestWidget.debugFillProperties called',
          'RenderTest.debugFillProperties called', // debug only
          'toTimelineArguments used in non-debug build', // not included in release builds
        },
      ),
    );
    if (results.isNotEmpty) {
      foundError(results);
    }
  }

  Future<void> runFixTests(String package) async {
    final List<String> args = <String>['fix', '--compare-to-golden'];
    await runCommand(
      dart,
      args,
      workingDirectory: path.join(flutterRoot, 'packages', package, 'test_fixes'),
    );
  }

  Future<void> runPrivateTests() async {
    final List<String> args = <String>['run', 'bin/test_private.dart'];
    final Map<String, String> environment = <String, String>{
      'FLUTTER_ROOT': flutterRoot,
      if (Directory(pubCache).existsSync()) 'PUB_CACHE': pubCache,
    };
    adjustEnvironmentToEnableFlutterAsserts(environment);
    await runCommand(
      dart,
      args,
      workingDirectory: path.join(flutterRoot, 'packages', 'flutter', 'test_private'),
      environment: environment,
    );
  }

  // Tests that take longer than average to run. This is usually because they
  // need to compile something large or make use of the analyzer for the test.
  // These tests need to be platform agnostic as they are only run on a linux
  // machine to save on execution time and cost.
  Future<void> runSlow() async {
    printProgress(
      '${green}Running slow package tests$reset for directories other than packages/flutter',
    );
    await runTracingTests();
    await runFixTests('flutter');
    await runFixTests('flutter_test');
    await runFixTests('integration_test');
    await runFixTests('flutter_driver');
    await runPrivateTests();
  }

  Future<void> runMisc() async {
    printProgress(
      '${green}Running package tests$reset for directories other than packages/flutter',
    );
    await testHarnessTestsRunner();
    await runExampleTests();
    await runFlutterTest(
      path.join(flutterRoot, 'dev', 'a11y_assessments'),
      tests: <String>['test'],
    );
    await runDartTest(path.join(flutterRoot, 'dev', 'bots'));
    await runDartTest(
      path.join(flutterRoot, 'dev', 'devicelab'),
      ensurePrecompiledTool: false, // See https://github.com/flutter/flutter/issues/86209
    );
    await runDartTest(path.join(flutterRoot, 'dev', 'conductor', 'core'), forceSingleCore: true);
    // TODO(gspencergoog): Remove the exception for fatalWarnings once https://github.com/flutter/flutter/issues/113782 has landed.
    await runFlutterTest(
      path.join(flutterRoot, 'dev', 'integration_tests', 'android_semantics_testing'),
      fatalWarnings: false,
    );
    await runFlutterTest(path.join(flutterRoot, 'dev', 'integration_tests', 'ui'));
    await runFlutterTest(path.join(flutterRoot, 'dev', 'manual_tests'));
    await runFlutterTest(path.join(flutterRoot, 'dev', 'tools'));
    await runFlutterTest(path.join(flutterRoot, 'dev', 'tools', 'vitool'));
    await runFlutterTest(path.join(flutterRoot, 'dev', 'tools', 'gen_defaults'));
    await runFlutterTest(path.join(flutterRoot, 'dev', 'tools', 'gen_keycodes'));
    await runFlutterTest(path.join(flutterRoot, 'dev', 'benchmarks', 'test_apps', 'stocks'));
    await runFlutterTest(
      path.join(flutterRoot, 'packages', 'flutter_driver'),
      tests: <String>[path.join('test', 'src', 'real_tests')],
    );
    await runFlutterTest(
      path.join(flutterRoot, 'packages', 'integration_test'),
      options: <String>[
        '--enable-vmservice',
        // Web-specific tests depend on Chromium, so they run as part of the web_long_running_tests shard.
        '--exclude-tags=web',
      ],
    );
    // Run java unit tests for integration_test
    //
    // Generate Gradle wrapper if it doesn't exist.
    Process.runSync(
      flutter,
      <String>['build', 'apk', '--config-only'],
      workingDirectory: path.join(
        flutterRoot,
        'packages',
        'integration_test',
        'example',
        'android',
      ),
    );
    await runCommand(
      path.join(flutterRoot, 'packages', 'integration_test', 'example', 'android', 'gradlew$bat'),
      <String>[
        ':integration_test:testDebugUnitTest',
        '--tests',
        'dev.flutter.plugins.integration_test.FlutterDeviceScreenshotTest',
      ],
      workingDirectory: path.join(
        flutterRoot,
        'packages',
        'integration_test',
        'example',
        'android',
      ),
    );
    await runFlutterTest(path.join(flutterRoot, 'packages', 'flutter_goldens'));
    await runFlutterTest(path.join(flutterRoot, 'packages', 'flutter_localizations'));
    await runFlutterTest(path.join(flutterRoot, 'packages', 'flutter_test'));
    await runFlutterTest(path.join(flutterRoot, 'packages', 'fuchsia_remote_debug_protocol'));
    await runFlutterTest(path.join(flutterRoot, 'dev', 'integration_tests', 'non_nullable'));
    const String httpClientWarning =
        'Warning: At least one test in this suite creates an HttpClient. When running a test suite that uses\n'
        'TestWidgetsFlutterBinding, all HTTP requests will return status code 400, and no network request\n'
        'will actually be made. Any test expecting a real network connection and status code will fail.\n'
        'To test code that needs an HttpClient, provide your own HttpClient implementation to the code under\n'
        'test, so that your test can consistently provide a testable response to the code under test.';
    await runFlutterTest(
      path.join(flutterRoot, 'packages', 'flutter_test'),
      script: path.join('test', 'bindings_test_failure.dart'),
      expectFailure: true,
      printOutput: false,
      outputChecker: (CommandResult result) {
        final Iterable<Match> matches = httpClientWarning.allMatches(result.flattenedStdout!);
        if (matches.isEmpty || matches.length > 1) {
          return 'Failed to print warning about HttpClientUsage, or printed it too many times.\n\n'
              'stdout:\n${result.flattenedStdout}\n\n'
              'stderr:\n${result.flattenedStderr}';
        }
        return null;
      },
    );
  }

  await selectSubshard(<String, ShardRunner>{
    'widgets': runWidgets,
    'libraries': runLibraries,
    'slow': runSlow,
    'misc': runMisc,
    'impeller': runImpeller,
  });
}
