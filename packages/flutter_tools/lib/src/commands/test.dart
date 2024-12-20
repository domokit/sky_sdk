// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:meta/meta.dart';
import 'package:package_config/package_config_types.dart';

import '../asset.dart';
import '../base/common.dart';
import '../base/file_system.dart';
import '../build_info.dart';
import '../bundle_builder.dart';
import '../devfs.dart';
import '../device.dart';
import '../globals.dart' as globals;
import '../native_assets.dart';
import '../project.dart';
import '../runner/flutter_command.dart';
import '../runner/flutter_command_runner.dart';
import '../test/coverage_collector.dart';
import '../test/event_printer.dart';
import '../test/runner.dart';
import '../test/test_time_recorder.dart';
import '../test/test_wrapper.dart';
import '../test/watcher.dart';
import '../web/compile.dart';
import '../web/web_constants.dart';

/// The name of the directory where Integration Tests are placed.
///
/// When there are test files specified for the test command that are part of
/// this directory, *relative to the package root*, the files will be executed
/// as Integration Tests.
const String _kIntegrationTestDirectory = 'integration_test';

/// A command to run tests.
///
/// This command has two modes of execution:
///
/// ## Unit / Widget Tests
///
/// These tests run in the Flutter Tester, which is a desktop-based Flutter
/// embedder. In this mode, tests are quick to compile and run.
///
/// By default, if no flags are passed to the `flutter test` command, the Tool
/// will recursively find all files within the `test/` directory that end with
/// the `*_test.dart` suffix, and run them in a single invocation.
///
/// See:
/// - https://flutter.dev/to/unit-testing
/// - https://flutter.dev/to/widget-testing
///
/// ## Integration Tests
///
/// These tests run in a connected Flutter Device, similar to `flutter run`. As
/// a result, iteration is slower because device-based artifacts have to be
/// built.
///
/// Integration tests should be placed in the `integration_test/` directory of
/// your package. To run these tests, use `flutter test integration_test`.
///
/// See:
/// - https://flutter.dev/to/integration-testing
class TestCommand extends FlutterCommand with DeviceBasedDevelopmentArtifacts {
  TestCommand({
    bool verboseHelp = false,
    this.testWrapper = const TestWrapper(),
    this.testRunner = const FlutterTestRunner(),
    this.verbose = false,
    this.nativeAssetsBuilder,
  }) {
    requiresPubspecYaml();
    usesPubOption();
    addNullSafetyModeOptions(hide: !verboseHelp);
    usesFrontendServerStarterPathOption(verboseHelp: verboseHelp);
    usesTrackWidgetCreation(verboseHelp: verboseHelp);
    addEnableExperimentation(hide: !verboseHelp);
    usesDartDefineOption();
    usesDeviceUserOption();
    usesFlavorOption();
    addEnableImpellerFlag(verboseHelp: verboseHelp);

    argParser
      ..addFlag(
        'experimental-faster-testing',
        negatable: false,
        hide: !verboseHelp,
        help: 'Run each test in a separate lightweight Flutter Engine to speed up testing.',
      )
      ..addMultiOption(
        'name',
        help: 'A regular expression matching substrings of the names of tests to run.',
        valueHelp: 'regexp',
        splitCommas: false,
      )
      ..addMultiOption(
        'plain-name',
        help: 'A plain-text substring of the names of tests to run.',
        valueHelp: 'substring',
        splitCommas: false,
      )
      ..addOption(
        'tags',
        abbr: 't',
        help:
            'Run only tests associated with the specified tags. See: https://pub.dev/packages/test#tagging-tests',
      )
      ..addOption(
        'exclude-tags',
        abbr: 'x',
        help:
            'Run only tests that do not have the specified tags. See: https://pub.dev/packages/test#tagging-tests',
      )
      ..addFlag(
        'start-paused',
        negatable: false,
        help:
            'Start in a paused mode and wait for a debugger to connect.\n'
            'You must specify a single test file to run, explicitly.\n'
            'Instructions for connecting with a debugger are printed to the '
            'console once the test has started.',
      )
      ..addFlag('fail-fast', help: 'Stop running tests after the first failure.')
      ..addFlag('run-skipped', help: 'Run skipped tests instead of skipping them.')
      ..addFlag(
        'disable-service-auth-codes',
        negatable: false,
        hide: !verboseHelp,
        help:
            '(deprecated) Allow connections to the VM service without using authentication codes. '
            '(Not recommended! This can open your device to remote code execution attacks!)',
      )
      ..addFlag('coverage', negatable: false, help: 'Whether to collect coverage information.')
      ..addFlag(
        'merge-coverage',
        negatable: false,
        help:
            'Whether to merge coverage data with "coverage/lcov.base.info".\n'
            'Implies collecting coverage data. (Requires lcov.)',
      )
      ..addFlag(
        'branch-coverage',
        negatable: false,
        help:
            'Whether to collect branch coverage information. '
            'Implies collecting coverage data.',
      )
      ..addFlag(
        'ipv6',
        negatable: false,
        hide: !verboseHelp,
        help: 'Whether to use IPv6 for the test harness server socket.',
      )
      ..addOption(
        'coverage-path',
        defaultsTo: 'coverage/lcov.info',
        help: 'Where to store coverage information (if coverage is enabled).',
      )
      ..addMultiOption(
        'coverage-package',
        help:
            'A regular expression matching packages names '
            'to include in the coverage report (if coverage is enabled). '
            'If unset, matches the current package name.',
        valueHelp: 'package-name-regexp',
        splitCommas: false,
      )
      ..addFlag(
        'machine',
        hide: !verboseHelp,
        negatable: false,
        help:
            'Handle machine structured JSON command input '
            'and provide output and progress in machine friendly format.',
      )
      ..addFlag(
        'update-goldens',
        negatable: false,
        help:
            'Whether "matchesGoldenFile()" calls within your test methods should ' // flutter_ignore: golden_tag (see analyze.dart)
            'update the golden files rather than test for an existing match.',
      )
      ..addOption(
        'concurrency',
        abbr: 'j',
        help:
            'The number of concurrent test processes to run. This will be ignored '
            'when running integration tests.',
        valueHelp: 'jobs',
      )
      ..addFlag(
        'test-assets',
        defaultsTo: true,
        help:
            'Whether to build the assets bundle for testing. '
            'This takes additional time before running the tests. '
            'Consider using "--no-test-assets" if assets are not required.',
      )
      // --platform is not supported to be used by Flutter developers. It only
      // exists to test the Flutter framework itself and may be removed entirely
      // in the future. Developers should either use plain `flutter test`, or
      // `package:integration_test` instead.
      ..addOption(
        'platform',
        allowed: const <String>['tester', 'chrome'],
        hide: !verboseHelp,
        defaultsTo: 'tester',
        help: 'Selects the test backend.',
        allowedHelp: <String, String>{
          'tester': 'Run tests using the VM-based test environment.',
          'chrome':
              '(deprecated) Run tests using the Google Chrome web browser. '
              'This value is intended for testing the Flutter framework '
              'itself and may be removed at any time.',
        },
      )
      ..addOption(
        'test-randomize-ordering-seed',
        help:
            'The seed to randomize the execution order of test cases within test files. '
            'Must be a 32bit unsigned integer or the string "random", '
            'which indicates that a seed should be selected randomly. '
            'By default, tests run in the order they are declared.',
      )
      ..addOption(
        'total-shards',
        help:
            'Tests can be sharded with the "--total-shards" and "--shard-index" '
            'arguments, allowing you to split up your test suites and run '
            'them separately.',
      )
      ..addOption(
        'shard-index',
        help:
            'Tests can be sharded with the "--total-shards" and "--shard-index" '
            'arguments, allowing you to split up your test suites and run '
            'them separately.',
      )
      ..addFlag(
        'enable-vmservice',
        hide: !verboseHelp,
        help:
            'Enables the VM service without "--start-paused". This flag is '
            'intended for use with tests that will use "dart:developer" to '
            'interact with the VM service at runtime.\n'
            'This flag is ignored if "--start-paused" or coverage are requested, as '
            'the VM service will be enabled in those cases regardless.',
      )
      ..addOption(
        'reporter',
        abbr: 'r',
        help:
            'Set how to print test results. If unset, value will default to either compact or expanded.',
        allowed: <String>['compact', 'expanded', 'failures-only', 'github', 'json', 'silent'],
        allowedHelp: <String, String>{
          'compact': 'A single line, updated continuously (the default).',
          'expanded':
              'A separate line for each update. May be preferred when logging to a file or in continuous integration.',
          'failures-only': 'A separate line for failing tests, with no output for passing tests.',
          'github':
              'A custom reporter for GitHub Actions (the default reporter when running on GitHub Actions).',
          'json': 'A machine-readable format. See: https://dart.dev/go/test-docs/json_reporter.md',
          'silent':
              'A reporter with no output. May be useful when only the exit code is meaningful.',
        },
      )
      ..addOption(
        'file-reporter',
        help:
            'Enable an additional reporter writing test results to a file.\n'
            'Should be in the form <reporter>:<filepath>, '
            'Example: "json:reports/tests.json".',
      )
      ..addOption(
        'timeout',
        help:
            'The default test timeout, specified either '
            'in seconds (e.g. "60s"), '
            'as a multiplier of the default timeout (e.g. "2x"), '
            'or as the string "none" to disable the timeout entirely.',
      )
      ..addFlag(
        FlutterOptions.kWebWasmFlag,
        help: 'Compile to WebAssembly rather than JavaScript.\n$kWasmMoreInfo',
        negatable: false,
      );

    addDdsOptions(verboseHelp: verboseHelp);
    addServeObservatoryOptions(verboseHelp: verboseHelp);
    usesFatalWarningsOption(verboseHelp: verboseHelp);
  }

  /// The interface for starting and configuring the tester.
  final TestWrapper testWrapper;

  /// Interface for running the tester process.
  final FlutterTestRunner testRunner;

  final TestCompilerNativeAssetsBuilder? nativeAssetsBuilder;

  final bool verbose;

  @visibleForTesting
  bool get isIntegrationTest => _isIntegrationTest;
  bool _isIntegrationTest = false;

  final Set<Uri> _testFileUris = <Uri>{};

  bool get isWeb => stringArg('platform') == 'chrome';
  bool get useWasm => boolArg(FlutterOptions.kWebWasmFlag);

  @override
  Future<Set<DevelopmentArtifact>> get requiredArtifacts async {
    final Set<DevelopmentArtifact> results =
        _isIntegrationTest
            // Use [DeviceBasedDevelopmentArtifacts].
            ? await super.requiredArtifacts
            : <DevelopmentArtifact>{};
    if (isWeb) {
      results.add(DevelopmentArtifact.web);
    }
    return results;
  }

  @override
  String get name => 'test';

  @override
  String get description => 'Run Flutter unit tests for the current project.';

  @override
  String get category => FlutterCommandCategory.project;

  @override
  Future<FlutterCommandResult> verifyThenRunCommand(String? commandPath) {
    final List<Uri> testUris = argResults!.rest.map(_parseTestArgument).toList();
    if (testUris.isEmpty) {
      // We don't scan the entire package, only the test/ subdirectory, so that
      // files with names like "hit_test.dart" don't get run.
      final Directory testDir = globals.fs.directory('test');
      if (!testDir.existsSync()) {
        throwToolExit('Test directory "${testDir.path}" not found.');
      }
      _testFileUris.addAll(_findTests(testDir).map(Uri.file));
      if (_testFileUris.isEmpty) {
        throwToolExit(
          'Test directory "${testDir.path}" does not appear to contain any test files.\n'
          'Test files must be in that directory and end with the pattern "_test.dart".',
        );
      }
    } else {
      for (final Uri uri in testUris) {
        // Test files may have query strings to support name/line/col:
        //     flutter test test/foo.dart?name=a&line=1
        String testPath = uri.replace(query: '').toFilePath();
        testPath = globals.fs.path.absolute(testPath);
        testPath = globals.fs.path.normalize(testPath);
        if (globals.fs.isDirectorySync(testPath)) {
          _testFileUris.addAll(_findTests(globals.fs.directory(testPath)).map(Uri.file));
        } else {
          _testFileUris.add(Uri.file(testPath).replace(query: uri.query));
        }
      }
    }

    // This needs to be set before [super.verifyThenRunCommand] so that the
    // correct [requiredArtifacts] can be identified before [run] takes place.
    final List<String> testFilePaths =
        _testFileUris.map((Uri uri) => uri.replace(query: '').toFilePath()).toList();
    _isIntegrationTest = _shouldRunAsIntegrationTests(
      globals.fs.currentDirectory.absolute.path,
      testFilePaths,
    );

    globals.printTrace(
      'Found ${_testFileUris.length} files which will be executed as '
      '${_isIntegrationTest ? 'Integration' : 'Widget'} Tests.',
    );
    return super.verifyThenRunCommand(commandPath);
  }

  // Keep in sync with the [RunCommandBase.webRenderer] getter.
  WebRendererMode get webRenderer {
    final List<String> dartDefines = extractDartDefines(
      defineConfigJsonMap: extractDartDefineConfigJsonMap(),
    );
    return WebRendererMode.fromDartDefines(dartDefines, useWasm: useWasm);
  }

  @override
  Future<FlutterCommandResult> runCommand() async {
    if (!globals.fs.isFileSync('pubspec.yaml')) {
      throwToolExit(
        'Error: No pubspec.yaml file found in the current working directory.\n'
        'Run this command from the root of your project. Test files must be '
        "called *_test.dart and must reside in the package's 'test' "
        'directory (or one of its subdirectories).',
      );
    }
    final FlutterProject flutterProject = FlutterProject.current();
    final bool buildTestAssets = boolArg('test-assets');
    final List<String> names = stringsArg('name');
    final List<String> plainNames = stringsArg('plain-name');
    final String? tags = stringArg('tags');
    final String? excludeTags = stringArg('exclude-tags');
    final BuildInfo buildInfo = await getBuildInfo(
      forcedBuildMode: BuildMode.debug,
      forcedUseLocalCanvasKit: true,
    );

    TestTimeRecorder? testTimeRecorder;
    if (verbose) {
      testTimeRecorder = TestTimeRecorder(globals.logger);
    }

    if (buildInfo.packageConfig['test_api'] == null) {
      throwToolExit(
        'Error: cannot run without a dependency on either "package:flutter_test" or "package:test". '
        'Ensure the following lines are present in your pubspec.yaml:'
        '\n\n'
        'dev_dependencies:\n'
        '  flutter_test:\n'
        '    sdk: flutter\n',
      );
    }

    bool experimentalFasterTesting = boolArg('experimental-faster-testing');
    if (experimentalFasterTesting) {
      if (_isIntegrationTest || isWeb) {
        experimentalFasterTesting = false;
        globals.printStatus(
          '--experimental-faster-testing was parsed but will be ignored. This '
          'option is not supported when running integration tests or web tests.',
        );
      } else if (_testFileUris.length == 1) {
        experimentalFasterTesting = false;
        globals.printStatus(
          '--experimental-faster-testing was parsed but will be ignored. This '
          'option should not be used when running a single test file.',
        );
      }
    }

    final bool startPaused = boolArg('start-paused');
    if (startPaused && _testFileUris.length != 1) {
      throwToolExit(
        'When using --start-paused, you must specify a single test file to run.',
        exitCode: 1,
      );
    }

    final DebuggingOptions debuggingOptions = DebuggingOptions.enabled(
      buildInfo,
      startPaused: startPaused,
      disableServiceAuthCodes: boolArg('disable-service-auth-codes'),
      serveObservatory: boolArg('serve-observatory'),
      // On iOS >=14, keeping this enabled will leave a prompt on the screen.
      disablePortPublication: true,
      enableDds: enableDds,
      nullAssertions: boolArg(FlutterOptions.kNullAssertions),
      usingCISystem: usingCISystem,
      enableImpeller: ImpellerStatus.fromBool(argResults!['enable-impeller'] as bool?),
      debugLogsDirectoryPath: debugLogsDirectoryPath,
      webRenderer: webRenderer,
      printDtd: boolArg(FlutterGlobalOptions.kPrintDtd, global: true),
      webUseWasm: useWasm,
    );

    final Uri? nativeAssetsJson = await nativeAssetsBuilder?.build(buildInfo);
    String? testAssetPath;
    if (buildTestAssets) {
      await _buildTestAsset(
        flavor: buildInfo.flavor,
        impellerStatus: debuggingOptions.enableImpeller,
        buildMode: debuggingOptions.buildInfo.mode,
        packageConfigPath: buildInfo.packageConfigPath,
      );
    }
    if (buildTestAssets || nativeAssetsJson != null) {
      testAssetPath = globals.fs.path.join(
        flutterProject.directory.path,
        'build',
        'unit_test_assets',
      );
    }
    if (nativeAssetsJson != null) {
      final Directory testAssetDirectory = globals.fs.directory(testAssetPath);
      if (!testAssetDirectory.existsSync()) {
        await testAssetDirectory.create(recursive: true);
      }
      final File nativeAssetsManifest = testAssetDirectory.childFile('NativeAssetsManifest.json');
      await globals.fs.file(nativeAssetsJson).copy(nativeAssetsManifest.path);
    }

    final String? concurrencyString = stringArg('concurrency');
    int? jobs = concurrencyString == null ? null : int.tryParse(concurrencyString);
    if (jobs != null && (jobs <= 0 || !jobs.isFinite)) {
      throwToolExit(
        'Could not parse -j/--concurrency argument. It must be an integer greater than zero.',
      );
    }

    if (_isIntegrationTest || isWeb) {
      if (argResults!.wasParsed('concurrency')) {
        globals.printStatus(
          '-j/--concurrency was parsed but will be ignored, this option is not '
          'supported when running Integration Tests or web tests.',
        );
      }
      // Running with concurrency will result in deploying multiple test apps
      // on the connected device concurrently, which is not supported.
      jobs = 1;
    } else if (experimentalFasterTesting) {
      if (argResults!.wasParsed('concurrency')) {
        globals.printStatus(
          '-j/--concurrency was parsed but will be ignored. This option is not '
          'compatible with --experimental-faster-testing.',
        );
      }
    }

    final int? shardIndex = int.tryParse(stringArg('shard-index') ?? '');
    if (shardIndex != null && (shardIndex < 0 || !shardIndex.isFinite)) {
      throwToolExit(
        'Could not parse --shard-index=$shardIndex argument. It must be an integer greater than -1.',
      );
    }

    final int? totalShards = int.tryParse(stringArg('total-shards') ?? '');
    if (totalShards != null && (totalShards <= 0 || !totalShards.isFinite)) {
      throwToolExit(
        'Could not parse --total-shards=$totalShards argument. It must be an integer greater than zero.',
      );
    }

    if (totalShards != null && shardIndex == null) {
      throwToolExit('If you set --total-shards you need to also set --shard-index.');
    }
    if (shardIndex != null && totalShards == null) {
      throwToolExit('If you set --shard-index you need to also set --total-shards.');
    }

    final bool enableVmService = boolArg('enable-vmservice');
    if (experimentalFasterTesting && enableVmService) {
      globals.printStatus(
        '--enable-vmservice was parsed but will be ignored. This option is not '
        'compatible with --experimental-faster-testing.',
      );
    }

    final bool ipv6 = boolArg('ipv6');
    if (experimentalFasterTesting && ipv6) {
      // [ipv6] is set when the user desires for the test harness server to use
      // IPv6, but a test harness server will not be started at all when
      // [experimentalFasterTesting] is set.
      globals.printStatus(
        '--ipv6 was parsed but will be ignored. This option is not compatible '
        'with --experimental-faster-testing.',
      );
    }

    final bool machine = boolArg('machine');
    CoverageCollector? collector;
    if (boolArg('coverage') || boolArg('merge-coverage') || boolArg('branch-coverage')) {
      final Set<String> packagesToInclude = _getCoveragePackages(
        stringsArg('coverage-package'),
        flutterProject,
        buildInfo.packageConfig,
      );
      collector = CoverageCollector(
        verbose: !machine,
        libraryNames: packagesToInclude,
        packagesPath: buildInfo.packageConfigPath,
        resolver: await CoverageCollector.getResolver(buildInfo.packageConfigPath),
        testTimeRecorder: testTimeRecorder,
        branchCoverage: boolArg('branch-coverage'),
      );
    }

    TestWatcher? watcher;
    if (machine) {
      watcher = EventPrinter(parent: collector, out: globals.stdio.stdout);
    } else if (collector != null) {
      watcher = collector;
    }

    if (!isWeb && useWasm) {
      throwToolExit('--wasm is only supported on the web platform');
    }

    if (webRenderer == WebRendererMode.skwasm && !useWasm) {
      throwToolExit('Skwasm renderer requires --wasm');
    }

    if (webRenderer.isDeprecated) {
      globals.logger.printWarning(webRenderer.deprecationWarning);
    }

    Device? integrationTestDevice;
    if (_isIntegrationTest) {
      integrationTestDevice = await findTargetDevice();

      // Disable reporting of test results to native test frameworks. This isn't
      // needed as the Flutter Tool will be responsible for reporting results.
      buildInfo.dartDefines.add('INTEGRATION_TEST_SHOULD_REPORT_RESULTS_TO_NATIVE=false');

      if (integrationTestDevice == null) {
        throwToolExit(
          'No devices are connected. '
          'Ensure that `flutter doctor` shows at least one connected device',
        );
      }
      if (integrationTestDevice.platformType == PlatformType.web) {
        // TODO(jiahaog): Support web. https://github.com/flutter/flutter/issues/66264
        throwToolExit('Web devices are not supported for integration tests yet.');
      }

      if (buildInfo.packageConfig['integration_test'] == null) {
        throwToolExit(
          'Error: cannot run without a dependency on "package:integration_test". '
          'Ensure the following lines are present in your pubspec.yaml:'
          '\n\n'
          'dev_dependencies:\n'
          '  integration_test:\n'
          '    sdk: flutter\n',
        );
      }

      if (stringArg('flavor') != null && !integrationTestDevice.supportsFlavors) {
        throwToolExit('--flavor is only supported for Android, macOS, and iOS devices.');
      }
    }

    final Stopwatch? testRunnerTimeRecorderStopwatch = testTimeRecorder?.start(
      TestTimePhases.TestRunner,
    );
    final int result;
    if (experimentalFasterTesting) {
      assert(!isWeb && !_isIntegrationTest && _testFileUris.length > 1);
      result = await testRunner.runTestsBySpawningLightweightEngines(
        _testFileUris.toList(),
        debuggingOptions: debuggingOptions,
        names: names,
        plainNames: plainNames,
        tags: tags,
        excludeTags: excludeTags,
        machine: machine,
        updateGoldens: boolArg('update-goldens'),
        concurrency: jobs,
        testAssetDirectory: testAssetPath,
        flutterProject: flutterProject,
        randomSeed: stringArg('test-randomize-ordering-seed'),
        reporter: stringArg('reporter'),
        fileReporter: stringArg('file-reporter'),
        timeout: stringArg('timeout'),
        failFast: boolArg('fail-fast'),
        runSkipped: boolArg('run-skipped'),
        shardIndex: shardIndex,
        totalShards: totalShards,
        testTimeRecorder: testTimeRecorder,
        nativeAssetsBuilder: nativeAssetsBuilder,
      );
    } else {
      result = await testRunner.runTests(
        testWrapper,
        _testFileUris.toList(),
        debuggingOptions: debuggingOptions,
        names: names,
        plainNames: plainNames,
        tags: tags,
        excludeTags: excludeTags,
        watcher: watcher,
        enableVmService: collector != null || startPaused || enableVmService,
        machine: machine,
        updateGoldens: boolArg('update-goldens'),
        concurrency: jobs,
        testAssetDirectory: testAssetPath,
        flutterProject: flutterProject,
        web: isWeb,
        randomSeed: stringArg('test-randomize-ordering-seed'),
        reporter: stringArg('reporter'),
        fileReporter: stringArg('file-reporter'),
        timeout: stringArg('timeout'),
        failFast: boolArg('fail-fast'),
        runSkipped: boolArg('run-skipped'),
        shardIndex: shardIndex,
        totalShards: totalShards,
        integrationTestDevice: integrationTestDevice,
        integrationTestUserIdentifier: stringArg(FlutterOptions.kDeviceUser),
        testTimeRecorder: testTimeRecorder,
        nativeAssetsBuilder: nativeAssetsBuilder,
        buildInfo: buildInfo,
      );
    }
    testTimeRecorder?.stop(TestTimePhases.TestRunner, testRunnerTimeRecorderStopwatch!);

    if (collector != null) {
      final Stopwatch? collectTimeRecorderStopwatch = testTimeRecorder?.start(
        TestTimePhases.CoverageDataCollect,
      );
      final bool collectionResult = await collector.collectCoverageData(
        stringArg('coverage-path'),
        mergeCoverageData: boolArg('merge-coverage'),
      );
      testTimeRecorder?.stop(TestTimePhases.CoverageDataCollect, collectTimeRecorderStopwatch!);
      if (!collectionResult) {
        testTimeRecorder?.print();
        throwToolExit(null);
      }
    }

    testTimeRecorder?.print();

    if (result != 0) {
      throwToolExit(null);
    }
    return FlutterCommandResult.success();
  }

  Set<String> _getCoveragePackages(
    List<String> packagesRegExps,
    FlutterProject flutterProject,
    PackageConfig packageConfig,
  ) {
    final String projectName = flutterProject.manifest.appName;
    final Set<String> packagesToInclude = <String>{if (packagesRegExps.isEmpty) projectName};
    try {
      for (final String regExpStr in packagesRegExps) {
        final RegExp regExp = RegExp(regExpStr);
        packagesToInclude.addAll(
          packageConfig.packages.map((Package e) => e.name).where((String e) => regExp.hasMatch(e)),
        );
      }
    } on FormatException catch (e) {
      throwToolExit('Regular expression syntax is invalid. $e');
    }
    return packagesToInclude;
  }

  /// Parses a test file/directory target passed as an argument and returns it
  /// as an absolute file:/// [URI] with optional querystring for name/line/col.
  Uri _parseTestArgument(String arg) {
    // We can't parse Windows paths as URIs if they have query strings, so
    // parse the file and query parts separately.
    final int queryStart = arg.indexOf('?');
    String filePart = queryStart == -1 ? arg : arg.substring(0, queryStart);
    final String queryPart = queryStart == -1 ? '' : arg.substring(queryStart + 1);

    filePart = globals.fs.path.absolute(filePart);
    filePart = globals.fs.path.normalize(filePart);

    return Uri.file(filePart).replace(query: queryPart.isEmpty ? null : queryPart);
  }

  Future<void> _buildTestAsset({
    required String? flavor,
    required ImpellerStatus impellerStatus,
    required BuildMode buildMode,
    required String packageConfigPath,
  }) async {
    final AssetBundle assetBundle = AssetBundleFactory.instance.createBundle();
    final int build = await assetBundle.build(packageConfigPath: packageConfigPath, flavor: flavor);
    if (build != 0) {
      throwToolExit('Error: Failed to build asset bundle');
    }
    if (_needsRebuild(assetBundle.entries, flavor)) {
      await writeBundle(
        globals.fs.directory(globals.fs.path.join('build', 'unit_test_assets')),
        assetBundle.entries,
        targetPlatform: TargetPlatform.tester,
        impellerStatus: impellerStatus,
        processManager: globals.processManager,
        fileSystem: globals.fs,
        artifacts: globals.artifacts!,
        logger: globals.logger,
        projectDir: globals.fs.currentDirectory,
        buildMode: buildMode,
      );

      final File cachedFlavorFile = globals.fs.file(
        globals.fs.path.join('build', 'test_cache', 'flavor.txt'),
      );
      if (cachedFlavorFile.existsSync()) {
        await cachedFlavorFile.delete();
      }
      if (flavor != null) {
        cachedFlavorFile.createSync(recursive: true);
        cachedFlavorFile.writeAsStringSync(flavor);
      }
    }
  }

  bool _needsRebuild(Map<String, AssetBundleEntry> entries, String? flavor) {
    // TODO(andrewkolos): This logic might fail in the future if we change the
    //  schema of the contents of the asset manifest file and the user does not
    //  perform a `flutter clean` after upgrading.
    //  See https://github.com/flutter/flutter/issues/128563.
    final File manifest = globals.fs.file(
      globals.fs.path.join('build', 'unit_test_assets', 'AssetManifest.bin'),
    );
    if (!manifest.existsSync()) {
      return true;
    }
    final DateTime lastModified = manifest.lastModifiedSync();
    final File pub = globals.fs.file('pubspec.yaml');
    if (pub.lastModifiedSync().isAfter(lastModified)) {
      return true;
    }

    final Iterable<DevFSFileContent> files =
        entries.values.map((AssetBundleEntry asset) => asset.content).whereType<DevFSFileContent>();
    for (final DevFSFileContent entry in files) {
      // Calling isModified to access file stats first in order for isModifiedAfter
      // to work.
      if (entry.isModified && entry.isModifiedAfter(lastModified)) {
        return true;
      }
    }

    final File cachedFlavorFile = globals.fs.file(
      globals.fs.path.join('build', 'test_cache', 'flavor.txt'),
    );
    final String? cachedFlavor =
        cachedFlavorFile.existsSync() ? cachedFlavorFile.readAsStringSync() : null;
    if (cachedFlavor != flavor) {
      return true;
    }

    return false;
  }
}

/// Searches [directory] and returns files that end with `_test.dart` as
/// absolute paths.
Iterable<String> _findTests(Directory directory) {
  return directory
      .listSync(recursive: true, followLinks: false)
      .where(
        (FileSystemEntity entity) =>
            entity.path.endsWith('_test.dart') && globals.fs.isFileSync(entity.path),
      )
      .map((FileSystemEntity entity) => globals.fs.path.absolute(entity.path));
}

/// Returns true if there are files that are Integration Tests.
///
/// The [currentDirectory] and [testFiles] parameters here must be provided as
/// absolute paths.
///
/// Throws an exception if there are both Integration Tests and Widget Tests
/// found in [testFiles].
bool _shouldRunAsIntegrationTests(String currentDirectory, List<String> testFiles) {
  final String integrationTestDirectory = globals.fs.path.join(
    currentDirectory,
    _kIntegrationTestDirectory,
  );

  if (testFiles.every(
    (String absolutePath) => !absolutePath.startsWith(integrationTestDirectory),
  )) {
    return false;
  }

  if (testFiles.every((String absolutePath) => absolutePath.startsWith(integrationTestDirectory))) {
    return true;
  }

  throwToolExit(
    'Integration tests and unit tests cannot be run in a single invocation.'
    ' Use separate invocations of `flutter test` to run integration tests'
    ' and unit tests.',
  );
}
