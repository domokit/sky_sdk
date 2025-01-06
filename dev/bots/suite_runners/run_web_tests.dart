// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert';
import 'dart:io'
    show
        Directory,
        File,
        FileSystemEntity,
        HttpClient,
        HttpClientRequest,
        HttpClientResponse,
        Platform,
        Process,
        RawSocket,
        SocketDirection,
        SocketException;
import 'dart:math' as math;
import 'package:file/local.dart';
import 'package:path/path.dart' as path;

import '../browser.dart';
import '../run_command.dart';
import '../service_worker_test.dart';
import '../utils.dart';

typedef ShardRunner = Future<void> Function();

class WebTestsSuite {
  WebTestsSuite(this.flutterTestArgs);

  /// Tests that we don't run on Web.
  ///
  /// In general avoid adding new tests here. If a test cannot run on the web
  /// because it fails at runtime, such as when a piece of functionality is not
  /// implemented or not implementable on the web, prefer using `skip` in the
  /// test code. Only add tests here that cannot be skipped using `skip`. For
  /// example:
  ///
  ///  * Test code cannot be compiled because it uses Dart VM-specific
  ///    functionality. In this case `skip` doesn't help because the code cannot
  ///    reach the point where it can even run the skipping logic.
  ///  * Migrations. It is OK to put tests here that need to be temporarily
  ///    disabled in certain modes because of some migration or initial bringup.
  ///
  /// The key in the map is the renderer type that the list applies to. The value
  /// is the list of tests known to fail for that renderer.
  //
  // TODO(yjbanov): we're getting rid of this as part of https://github.com/flutter/flutter/projects/60
  static const Map<String, List<String>> kWebTestFileKnownFailures = <String, List<String>>{
    'html': <String>[
      // These tests are not compilable on the web due to dependencies on
      // VM-specific functionality.
      'test/services/message_codecs_vm_test.dart',
      'test/examples/sector_layout_test.dart',
    ],
    'canvaskit': <String>[
      // These tests are not compilable on the web due to dependencies on
      // VM-specific functionality.
      'test/services/message_codecs_vm_test.dart',
      'test/examples/sector_layout_test.dart',

      // These tests are broken and need to be fixed.
      // TODO(yjbanov): https://github.com/flutter/flutter/issues/71604
      'test/material/text_field_test.dart',
      'test/widgets/performance_overlay_test.dart',
      'test/widgets/html_element_view_test.dart',
      'test/cupertino/scaffold_test.dart',
      'test/rendering/platform_view_test.dart',
    ],
    'skwasm': <String>[
      // These tests are not compilable on the web due to dependencies on
      // VM-specific functionality.
      'test/services/message_codecs_vm_test.dart',
      'test/examples/sector_layout_test.dart',

      // These tests are broken and need to be fixed.
      // TODO(jacksongardner): https://github.com/flutter/flutter/issues/71604
      'test/material/text_field_test.dart',
      'test/widgets/performance_overlay_test.dart',
    ],
  };

  /// The number of Cirrus jobs that run Web tests in parallel.
  ///
  /// The default is 8 shards. Typically .cirrus.yml would define the
  /// WEB_SHARD_COUNT environment variable rather than relying on the default.
  ///
  /// WARNING: if you change this number, also change .cirrus.yml
  /// and make sure it runs _all_ shards.
  ///
  /// The last shard also runs the Web plugin tests.
  int get webShardCount =>
      Platform.environment.containsKey('WEB_SHARD_COUNT')
          ? int.parse(Platform.environment['WEB_SHARD_COUNT']!)
          : 8;

  static const List<String> _kAllBuildModes = <String>['debug', 'profile', 'release'];

  final List<String> flutterTestArgs;

  /// Coarse-grained integration tests running on the Web.
  Future<void> webLongRunningTestsRunner() async {
    final String engineVersionFile = path.join(flutterRoot, 'bin', 'internal', 'engine.version');
    final String engineRealmFile = path.join(flutterRoot, 'bin', 'internal', 'engine.realm');
    final String engineVersion = File(engineVersionFile).readAsStringSync().trim();
    final String engineRealm = File(engineRealmFile).readAsStringSync().trim();
    if (engineRealm.isNotEmpty) {
      return;
    }
    final List<ShardRunner> tests = <ShardRunner>[
      for (final String buildMode in _kAllBuildModes) ...<ShardRunner>[
        () => _runFlutterDriverWebTest(
          testAppDirectory: path.join('packages', 'integration_test', 'example'),
          target: path.join('test_driver', 'failure.dart'),
          buildMode: buildMode,
          webRenderer: 'canvaskit',
          // This test intentionally fails and prints stack traces in the browser
          // logs. To avoid confusion, silence browser output.
          silenceBrowserOutput: true,
        ),
        () => _runFlutterDriverWebTest(
          testAppDirectory: path.join('packages', 'integration_test', 'example'),
          target: path.join('integration_test', 'example_test.dart'),
          driver: path.join('test_driver', 'integration_test.dart'),
          buildMode: buildMode,
          webRenderer: 'canvaskit',
          expectWriteResponseFile: true,
          expectResponseFileContent: 'null',
        ),
        () => _runFlutterDriverWebTest(
          testAppDirectory: path.join('packages', 'integration_test', 'example'),
          target: path.join('integration_test', 'extended_test.dart'),
          driver: path.join('test_driver', 'extended_integration_test.dart'),
          buildMode: buildMode,
          webRenderer: 'canvaskit',
          expectWriteResponseFile: true,
          expectResponseFileContent: '''
{
  "screenshots": [
    {
      "screenshotName": "platform_name",
      "bytes": []
    },
    {
      "screenshotName": "platform_name_2",
      "bytes": []
    }
  ]
}''',
        ),
      ],

      // This test doesn't do anything interesting w.r.t. rendering, so we don't run the full build mode x renderer matrix.
      () => _runWebE2eTest('profile_diagnostics_integration', buildMode: 'debug', renderer: 'html'),
      () => _runWebE2eTest(
        'profile_diagnostics_integration',
        buildMode: 'profile',
        renderer: 'canvaskit',
      ),
      () =>
          _runWebE2eTest('profile_diagnostics_integration', buildMode: 'release', renderer: 'html'),

      // This test is only known to work in debug mode.
      () => _runWebE2eTest('scroll_wheel_integration', buildMode: 'debug', renderer: 'html'),

      // This test doesn't do anything interesting w.r.t. rendering, so we don't run the full build mode x renderer matrix.
      // These tests have been extremely flaky, so we are temporarily disabling them until we figure out how to make them more robust.
      () => _runWebE2eTest('text_editing_integration', buildMode: 'debug', renderer: 'canvaskit'),
      () => _runWebE2eTest('text_editing_integration', buildMode: 'profile', renderer: 'html'),
      () => _runWebE2eTest('text_editing_integration', buildMode: 'release', renderer: 'html'),

      // This test doesn't do anything interesting w.r.t. rendering, so we don't run the full build mode x renderer matrix.
      () => _runWebE2eTest('url_strategy_integration', buildMode: 'debug', renderer: 'html'),
      () => _runWebE2eTest('url_strategy_integration', buildMode: 'profile', renderer: 'canvaskit'),
      () => _runWebE2eTest('url_strategy_integration', buildMode: 'release', renderer: 'html'),

      // This test doesn't do anything interesting w.r.t. rendering, so we don't run the full build mode x renderer matrix.
      () => _runWebE2eTest(
        'capabilities_integration_canvaskit',
        buildMode: 'debug',
        renderer: 'auto',
      ),
      () => _runWebE2eTest(
        'capabilities_integration_canvaskit',
        buildMode: 'profile',
        renderer: 'canvaskit',
      ),
      () => _runWebE2eTest('capabilities_integration_html', buildMode: 'release', renderer: 'html'),

      // This test doesn't do anything interesting w.r.t. rendering, so we don't run the full build mode x renderer matrix.
      // CacheWidth and CacheHeight are only currently supported in CanvasKit mode, so we don't run the test in HTML mode.
      () => _runWebE2eTest(
        'cache_width_cache_height_integration',
        buildMode: 'debug',
        renderer: 'auto',
      ),
      () => _runWebE2eTest(
        'cache_width_cache_height_integration',
        buildMode: 'profile',
        renderer: 'canvaskit',
      ),

      () => _runWebTreeshakeTest(),

      () => _runFlutterDriverWebTest(
        testAppDirectory: path.join(flutterRoot, 'examples', 'hello_world'),
        target: 'test_driver/smoke_web_engine.dart',
        buildMode: 'profile',
        webRenderer: 'auto',
      ),
      () => _runGalleryE2eWebTest('debug'),
      () => _runGalleryE2eWebTest('debug', canvasKit: true),
      () => _runGalleryE2eWebTest('profile'),
      () => _runGalleryE2eWebTest('profile', canvasKit: true),
      () => _runGalleryE2eWebTest('release'),
      () => _runGalleryE2eWebTest('release', canvasKit: true),
      () =>
          runWebServiceWorkerTest(headless: true, testType: ServiceWorkerTestType.withoutFlutterJs),
      () => runWebServiceWorkerTest(headless: true, testType: ServiceWorkerTestType.withFlutterJs),
      () => runWebServiceWorkerTest(
        headless: true,
        testType: ServiceWorkerTestType.withFlutterJsShort,
      ),
      () => runWebServiceWorkerTest(
        headless: true,
        testType: ServiceWorkerTestType.withFlutterJsEntrypointLoadedEvent,
      ),
      () => runWebServiceWorkerTest(
        headless: true,
        testType: ServiceWorkerTestType.withFlutterJsTrustedTypesOn,
      ),
      () => runWebServiceWorkerTest(
        headless: true,
        testType: ServiceWorkerTestType.withFlutterJsNonceOn,
      ),
      () => runWebServiceWorkerTestWithCachingResources(
        headless: true,
        testType: ServiceWorkerTestType.withoutFlutterJs,
      ),
      () => runWebServiceWorkerTestWithCachingResources(
        headless: true,
        testType: ServiceWorkerTestType.withFlutterJs,
      ),
      () => runWebServiceWorkerTestWithCachingResources(
        headless: true,
        testType: ServiceWorkerTestType.withFlutterJsShort,
      ),
      () => runWebServiceWorkerTestWithCachingResources(
        headless: true,
        testType: ServiceWorkerTestType.withFlutterJsEntrypointLoadedEvent,
      ),
      () => runWebServiceWorkerTestWithCachingResources(
        headless: true,
        testType: ServiceWorkerTestType.withFlutterJsTrustedTypesOn,
      ),
      () => runWebServiceWorkerTestWithGeneratedEntrypoint(headless: true),
      () => runWebServiceWorkerTestWithBlockedServiceWorkers(headless: true),
      () => runWebServiceWorkerTestWithCustomServiceWorkerVersion(headless: true),
      () => _runWebStackTraceTest('profile', 'lib/stack_trace.dart'),
      () => _runWebStackTraceTest('release', 'lib/stack_trace.dart'),
      () => _runWebStackTraceTest('profile', 'lib/framework_stack_trace.dart'),
      () => _runWebStackTraceTest('release', 'lib/framework_stack_trace.dart'),
      () => _runWebDebugTest('lib/stack_trace.dart'),
      () => _runWebDebugTest('lib/framework_stack_trace.dart'),
      () => _runWebDebugTest('lib/web_directory_loading.dart'),
      () => _runWebDebugTest(
        'lib/web_resources_cdn_test.dart',
        additionalArguments: <String>['--dart-define=TEST_FLUTTER_ENGINE_VERSION=$engineVersion'],
      ),
      () => _runWebDebugTest('test/test.dart'),
      () => _runWebDebugTest('lib/null_safe_main.dart'),
      () => _runWebDebugTest(
        'lib/web_define_loading.dart',
        additionalArguments: <String>[
          '--dart-define=test.valueA=Example,A',
          '--dart-define=test.valueB=Value',
        ],
      ),
      () => _runWebReleaseTest(
        'lib/web_define_loading.dart',
        additionalArguments: <String>[
          '--dart-define=test.valueA=Example,A',
          '--dart-define=test.valueB=Value',
        ],
      ),
      () => _runWebDebugTest('lib/assertion_test.dart'),
      () => _runWebReleaseTest('lib/assertion_test.dart'),
      () => _runWebDebugTest('lib/sound_mode.dart'),
      () => _runWebReleaseTest('lib/sound_mode.dart'),
      () => _runFlutterWebTest(
        'html',
        path.join(flutterRoot, 'packages', 'integration_test'),
        <String>['test/web_extension_test.dart'],
        false,
      ),
      () => _runFlutterWebTest(
        'canvaskit',
        path.join(flutterRoot, 'packages', 'integration_test'),
        <String>['test/web_extension_test.dart'],
        false,
      ),
      () => _runFlutterWebTest(
        'skwasm',
        path.join(flutterRoot, 'packages', 'integration_test'),
        <String>['test/web_extension_test.dart'],
        true,
      ),
    ];

    // Shuffling mixes fast tests with slow tests so shards take roughly the same
    // amount of time to run.
    tests.shuffle(math.Random(0));

    await _ensureChromeDriverIsRunning();
    await runShardRunnerIndexOfTotalSubshard(tests);
    await _stopChromeDriver();
  }

  Future<void> runWebHtmlUnitTests() {
    return _runWebUnitTests('html', false);
  }

  Future<void> runWebCanvasKitUnitTests() {
    return _runWebUnitTests('canvaskit', false);
  }

  Future<void> runWebSkwasmUnitTests() {
    return _runWebUnitTests('skwasm', true);
  }

  /// Runs one of the `dev/integration_tests/web_e2e_tests` tests.
  Future<void> _runWebE2eTest(
    String name, {
    required String buildMode,
    required String renderer,
  }) async {
    await _runFlutterDriverWebTest(
      target: path.join('test_driver', '$name.dart'),
      buildMode: buildMode,
      webRenderer: renderer,
      testAppDirectory: path.join(flutterRoot, 'dev', 'integration_tests', 'web_e2e_tests'),
    );
  }

  Future<void> _runFlutterDriverWebTest({
    required String target,
    required String buildMode,
    required String webRenderer,
    required String testAppDirectory,
    String? driver,
    bool expectFailure = false,
    bool silenceBrowserOutput = false,
    bool expectWriteResponseFile = false,
    String expectResponseFileContent = '',
  }) async {
    printProgress('${green}Running integration tests $target in $buildMode mode.$reset');
    await runCommand(flutter, <String>['clean'], workingDirectory: testAppDirectory);
    // This must match the testOutputsDirectory defined in flutter_driver's driver/common.dart.
    final String driverOutputPath =
        Platform.environment['FLUTTER_TEST_OUTPUTS_DIR'] ?? path.join(testAppDirectory, 'build');
    final String responseFile = path.join(driverOutputPath, 'integration_response_data.json');
    if (File(responseFile).existsSync()) {
      File(responseFile).deleteSync();
    }
    await runCommand(
      flutter,
      <String>[
        ...flutterTestArgs,
        'drive',
        if (driver != null) '--driver=$driver',
        '--target=$target',
        '--browser-name=chrome',
        '-d',
        'web-server',
        '--$buildMode',
        // '--web-renderer=$webRenderer',
        if (webRenderer == 'skwasm') ...<String>[
          // See: WebRendererMode.dartDefines[skwasm]
          '--dart-define=FLUTTER_WEB_USE_SKIA=false',
          '--dart-define=FLUTTER_WEB_USE_SKWASM=true',
        ],
        if (webRenderer == 'canvaskit') ...<String>[
          // See: WebRendererMode.dartDefines[canvaskit]
          '--dart-define=FLUTTER_WEB_USE_SKIA=true',
          '--dart-define=FLUTTER_WEB_USE_SKWASM=false',
        ],
        if (webRenderer == 'html') ...<String>[
          // See: WebRendererMode.dartDefines[html]
          '--dart-define=FLUTTER_WEB_USE_SKIA=false',
          '--dart-define=FLUTTER_WEB_USE_SKWASM=false',
        ],
      ],
      expectNonZeroExit: expectFailure,
      workingDirectory: testAppDirectory,
      environment: <String, String>{'FLUTTER_WEB': 'true'},
      removeLine: (String line) {
        if (!silenceBrowserOutput) {
          return false;
        }
        if (line.trim().startsWith('[INFO]')) {
          return true;
        }
        return false;
      },
    );
    if (expectWriteResponseFile) {
      if (!File(responseFile).existsSync()) {
        foundError(<String>[
          '$bold${red}Command did not write the response file but expected response file written.$reset',
        ]);
      } else {
        final String response = File(responseFile).readAsStringSync();
        if (response != expectResponseFileContent) {
          foundError(<String>[
            '$bold${red}Command write the response file with $response but expected response file with $expectResponseFileContent.$reset',
          ]);
        }
      }
    }
  }

  // Compiles a sample web app and checks that its JS doesn't contain certain
  // debug code that we expect to be tree shaken out.
  //
  // The app is compiled in `--profile` mode to prevent the compiler from
  // minifying the symbols.
  Future<void> _runWebTreeshakeTest() async {
    final String testAppDirectory = path.join(
      flutterRoot,
      'dev',
      'integration_tests',
      'web_e2e_tests',
    );
    final String target = path.join('lib', 'treeshaking_main.dart');
    await runCommand(flutter, <String>['clean'], workingDirectory: testAppDirectory);
    await runCommand(
      flutter,
      <String>['build', 'web', '--target=$target', '--profile'],
      workingDirectory: testAppDirectory,
      environment: <String, String>{'FLUTTER_WEB': 'true'},
    );

    final File mainDartJs = File(path.join(testAppDirectory, 'build', 'web', 'main.dart.js'));
    final String javaScript = mainDartJs.readAsStringSync();

    // Check that we're not looking at minified JS. Otherwise this test would result in false positive.
    expect(javaScript.contains('RootElement'), true);

    const String word = 'debugFillProperties';
    int count = 0;
    int pos = javaScript.indexOf(word);
    final int contentLength = javaScript.length;
    while (pos != -1) {
      count += 1;
      pos += word.length;
      if (pos >= contentLength || count > 100) {
        break;
      }
      pos = javaScript.indexOf(word, pos);
    }

    // The following are classes from `timeline.dart` that should be treeshaken
    // off unless the app (typically a benchmark) uses methods that need them.
    expect(javaScript.contains('AggregatedTimedBlock'), false);
    expect(javaScript.contains('AggregatedTimings'), false);
    expect(javaScript.contains('_BlockBuffer'), false);
    expect(javaScript.contains('_StringListChain'), false);
    expect(javaScript.contains('_Float64ListChain'), false);

    const int kMaxExpectedDebugFillProperties = 11;
    if (count > kMaxExpectedDebugFillProperties) {
      throw Exception(
        'Too many occurrences of "$word" in compiled JavaScript.\n'
        'Expected no more than $kMaxExpectedDebugFillProperties, but found $count.',
      );
    }
  }

  /// Exercises the old gallery in a browser for a long period of time, looking
  /// for memory leaks and dangling pointers.
  ///
  /// This is not a performance test.
  ///
  /// If [canvasKit] is set to true, runs the test in CanvasKit mode.
  ///
  /// The test is written using `package:integration_test` (despite the "e2e" in
  /// the name, which is there for historic reasons).
  Future<void> _runGalleryE2eWebTest(String buildMode, {bool canvasKit = false}) async {
    // TODO(yjbanov): this is temporarily disabled due to https://github.com/flutter/flutter/issues/147731
    if (buildMode == 'debug' && canvasKit) {
      print(
        'SKIPPED: Gallery e2e web test in debug CanvasKit mode due to https://github.com/flutter/flutter/issues/147731',
      );
      return;
    }
    printProgress(
      '${green}Running flutter_gallery integration test in --$buildMode using ${canvasKit ? 'CanvasKit' : 'HTML'} renderer.$reset',
    );
    final String testAppDirectory = path.join(
      flutterRoot,
      'dev',
      'integration_tests',
      'flutter_gallery',
    );
    await runCommand(flutter, <String>['clean'], workingDirectory: testAppDirectory);
    await runCommand(
      flutter,
      <String>[
        ...flutterTestArgs,
        'drive',
        if (canvasKit) '--dart-define=FLUTTER_WEB_USE_SKIA=true',
        if (!canvasKit) '--dart-define=FLUTTER_WEB_USE_SKIA=false',
        '--driver=test_driver/transitions_perf_e2e_test.dart',
        '--target=test_driver/transitions_perf_e2e.dart',
        '--browser-name=chrome',
        '-d',
        'web-server',
        '--$buildMode',
      ],
      workingDirectory: testAppDirectory,
      environment: <String, String>{'FLUTTER_WEB': 'true'},
    );
  }

  Future<void> _runWebStackTraceTest(String buildMode, String entrypoint) async {
    final String testAppDirectory = path.join(flutterRoot, 'dev', 'integration_tests', 'web');
    final String appBuildDirectory = path.join(testAppDirectory, 'build', 'web');

    // Build the app.
    await runCommand(flutter, <String>['clean'], workingDirectory: testAppDirectory);
    await runCommand(
      flutter,
      <String>['build', 'web', '--$buildMode', '-t', entrypoint],
      workingDirectory: testAppDirectory,
      environment: <String, String>{'FLUTTER_WEB': 'true'},
    );

    // Run the app.
    final int serverPort = await findAvailablePortAndPossiblyCauseFlakyTests();
    final int browserDebugPort = await findAvailablePortAndPossiblyCauseFlakyTests();
    final String result = await evalTestAppInChrome(
      appUrl: 'http://localhost:$serverPort/index.html',
      appDirectory: appBuildDirectory,
      serverPort: serverPort,
      browserDebugPort: browserDebugPort,
    );

    if (!result.contains('--- TEST SUCCEEDED ---')) {
      foundError(<String>[result, '${red}Web stack trace integration test failed.$reset']);
    }
  }

  /// Debug mode is special because `flutter build web` doesn't build in debug mode.
  ///
  /// Instead, we use `flutter run --debug` and sniff out the standard output.
  Future<void> _runWebDebugTest(
    String target, {
    List<String> additionalArguments = const <String>[],
  }) async {
    final String testAppDirectory = path.join(flutterRoot, 'dev', 'integration_tests', 'web');
    bool success = false;
    final Map<String, String> environment = <String, String>{'FLUTTER_WEB': 'true'};
    adjustEnvironmentToEnableFlutterAsserts(environment);
    final CommandResult result = await runCommand(
      flutter,
      <String>[
        'run',
        '--verbose',
        '--debug',
        '-d',
        'chrome',
        '--web-run-headless',
        '--dart-define=FLUTTER_WEB_USE_SKIA=false',
        ...additionalArguments,
        '-t',
        target,
      ],
      outputMode: OutputMode.capture,
      outputListener: (String line, Process process) {
        bool shutdownFlutterTool = false;
        if (line.contains('--- TEST SUCCEEDED ---')) {
          success = true;
          shutdownFlutterTool = true;
        }
        if (line.contains('--- TEST FAILED ---')) {
          shutdownFlutterTool = true;
        }
        if (shutdownFlutterTool) {
          process.stdin.add('q'.codeUnits);
        }
      },
      workingDirectory: testAppDirectory,
      environment: environment,
    );

    if (!success) {
      foundError(<String>[
        result.flattenedStdout!,
        result.flattenedStderr!,
        '${red}Web stack trace integration test failed.$reset',
      ]);
    }
  }

  /// Run a web integration test in release mode.
  Future<void> _runWebReleaseTest(
    String target, {
    List<String> additionalArguments = const <String>[],
  }) async {
    final String testAppDirectory = path.join(flutterRoot, 'dev', 'integration_tests', 'web');
    final String appBuildDirectory = path.join(testAppDirectory, 'build', 'web');

    // Build the app.
    await runCommand(flutter, <String>['clean'], workingDirectory: testAppDirectory);
    await runCommand(
      flutter,
      <String>[
        ...flutterTestArgs,
        'build',
        'web',
        '--release',
        ...additionalArguments,
        '-t',
        target,
      ],
      workingDirectory: testAppDirectory,
      environment: <String, String>{'FLUTTER_WEB': 'true'},
    );

    // Run the app.
    final int serverPort = await findAvailablePortAndPossiblyCauseFlakyTests();
    final int browserDebugPort = await findAvailablePortAndPossiblyCauseFlakyTests();
    final String result = await evalTestAppInChrome(
      appUrl: 'http://localhost:$serverPort/index.html',
      appDirectory: appBuildDirectory,
      serverPort: serverPort,
      browserDebugPort: browserDebugPort,
    );

    if (!result.contains('--- TEST SUCCEEDED ---')) {
      foundError(<String>[result, '${red}Web release mode test failed.$reset']);
    }
  }

  Future<void> _runWebUnitTests(String webRenderer, bool useWasm) async {
    final Map<String, ShardRunner> subshards = <String, ShardRunner>{};

    final Directory flutterPackageDirectory = Directory(
      path.join(flutterRoot, 'packages', 'flutter'),
    );
    final Directory flutterPackageTestDirectory = Directory(
      path.join(flutterPackageDirectory.path, 'test'),
    );

    final List<String> allTests =
        flutterPackageTestDirectory
            .listSync()
            .whereType<Directory>()
            .expand(
              (Directory directory) => directory
                  .listSync(recursive: true)
                  .where((FileSystemEntity entity) => entity.path.endsWith('_test.dart')),
            )
            .whereType<File>()
            .map<String>(
              (File file) => path.relative(file.path, from: flutterPackageDirectory.path),
            )
            .where(
              (String filePath) =>
                  !kWebTestFileKnownFailures[webRenderer]!.contains(path.split(filePath).join('/')),
            )
            .toList()
          // Finally we shuffle the list because we want the average cost per file to be uniformly
          // distributed. If the list is not sorted then different shards and batches may have
          // very different characteristics.
          // We use a constant seed for repeatability.
          ..shuffle(math.Random(0));

    assert(webShardCount >= 1);
    final int testsPerShard = (allTests.length / webShardCount).ceil();
    assert(testsPerShard * webShardCount >= allTests.length);

    // This for loop computes all but the last shard.
    for (int index = 0; index < webShardCount - 1; index += 1) {
      subshards['$index'] =
          () => _runFlutterWebTest(
            webRenderer,
            flutterPackageDirectory.path,
            allTests.sublist(index * testsPerShard, (index + 1) * testsPerShard),
            useWasm,
          );
    }

    // The last shard also runs the flutter_web_plugins tests.
    //
    // We make sure the last shard ends in _last so it's easier to catch mismatches
    // between `.cirrus.yml` and `test.dart`.
    subshards['${webShardCount - 1}_last'] = () async {
      await _runFlutterWebTest(
        webRenderer,
        flutterPackageDirectory.path,
        allTests.sublist((webShardCount - 1) * testsPerShard, allTests.length),
        useWasm,
      );
      await _runFlutterWebTest(
        webRenderer,
        path.join(flutterRoot, 'packages', 'flutter_web_plugins'),
        <String>['test'],
        useWasm,
      );
      await _runFlutterWebTest(
        webRenderer,
        path.join(flutterRoot, 'packages', 'flutter_driver'),
        <String>[path.join('test', 'src', 'web_tests', 'web_extension_test.dart')],
        useWasm,
      );
    };

    await selectSubshard(subshards);
  }

  Future<void> _runFlutterWebTest(
    String webRenderer,
    String workingDirectory,
    List<String> tests,
    bool useWasm,
  ) async {
    const LocalFileSystem fileSystem = LocalFileSystem();
    final String suffix = DateTime.now().microsecondsSinceEpoch.toString();
    final File metricFile = fileSystem.systemTempDirectory.childFile('metrics_$suffix.json');
    await runCommand(
      flutter,
      <String>[
        'test',
        '--reporter=expanded',
        '--file-reporter=json:${metricFile.path}',
        '-v',
        '--platform=chrome',
        if (useWasm) '--wasm',
        // '--web-renderer=$webRenderer',
        if (webRenderer == 'skwasm') ...<String>[
          // See: WebRendererMode.dartDefines[skwasm]
          '--dart-define=FLUTTER_WEB_USE_SKIA=false',
          '--dart-define=FLUTTER_WEB_USE_SKWASM=true',
        ],
        if (webRenderer == 'canvaskit') ...<String>[
          // See: WebRendererMode.dartDefines[canvaskit]
          '--dart-define=FLUTTER_WEB_USE_SKIA=true',
          '--dart-define=FLUTTER_WEB_USE_SKWASM=false',
        ],
        if (webRenderer == 'html') ...<String>[
          // See: WebRendererMode.dartDefines[html]
          '--dart-define=FLUTTER_WEB_USE_SKIA=false',
          '--dart-define=FLUTTER_WEB_USE_SKWASM=false',
        ],
        '--dart-define=DART_HHH_BOT=$runningInDartHHHBot',
        ...flutterTestArgs,
        ...tests,
      ],
      workingDirectory: workingDirectory,
      environment: <String, String>{'FLUTTER_WEB': 'true'},
    );
    // metriciFile is a transitional file that needs to be deleted once it is parsed.
    // TODO(godofredoc): Ensure metricFile is parsed and aggregated before deleting.
    // https://github.com/flutter/flutter/issues/146003
    if (!dryRun) {
      metricFile.deleteSync();
    }
  }

  // The `chromedriver` process created by this test.
  //
  // If an existing chromedriver is already available on port 4444, the existing
  // process is reused and this variable remains null.
  Command? _chromeDriver;

  Future<bool> _isChromeDriverRunning() async {
    try {
      final RawSocket socket = await RawSocket.connect('localhost', 4444);
      socket.shutdown(SocketDirection.both);
      await socket.close();
      return true;
    } on SocketException {
      return false;
    }
  }

  Future<void> _stopChromeDriver() async {
    if (_chromeDriver == null) {
      return;
    }
    print('Stopping chromedriver');
    _chromeDriver!.process.kill();
  }

  Future<void> _ensureChromeDriverIsRunning() async {
    // If we cannot connect to ChromeDriver, assume it is not running. Launch it.
    if (!await _isChromeDriverRunning()) {
      printProgress('Starting chromedriver');
      // Assume chromedriver is in the PATH.
      _chromeDriver = await startCommand(
        // TODO(ianh): this is the only remaining consumer of startCommand other than runCommand
        // and it doesn't use most of startCommand's features; we could simplify this a lot by
        // inlining the relevant parts of startCommand here.
        'chromedriver',
        <String>['--port=4444', '--log-level=INFO', '--enable-chrome-logs'],
      );
      while (!await _isChromeDriverRunning()) {
        await Future<void>.delayed(const Duration(milliseconds: 100));
        print('Waiting for chromedriver to start up.');
      }
    }

    final HttpClient client = HttpClient();
    final Uri chromeDriverUrl = Uri.parse('http://localhost:4444/status');
    final HttpClientRequest request = await client.getUrl(chromeDriverUrl);
    final HttpClientResponse response = await request.close();
    final String responseString = await response.transform(utf8.decoder).join();
    final Map<String, dynamic> webDriverStatus =
        json.decode(responseString) as Map<String, dynamic>;
    client.close();
    final bool webDriverReady = (webDriverStatus['value'] as Map<String, dynamic>)['ready'] as bool;
    if (!webDriverReady) {
      throw Exception('WebDriver not available.');
    }
  }
}
