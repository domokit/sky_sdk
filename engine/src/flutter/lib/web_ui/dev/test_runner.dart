// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';
import 'dart:io' as io;

import 'package:args/command_runner.dart';
import 'package:path/path.dart' as path;

import 'package:watcher/src/watch_event.dart';

import 'common.dart';
import 'environment.dart';
import 'exceptions.dart';
import 'felt_config.dart';
import 'pipeline.dart';
import 'steps/compile_bundle_step.dart';
import 'steps/copy_artifacts_step.dart';
import 'steps/run_suite_step.dart';
import 'suite_filter.dart';
import 'utils.dart';

/// Runs tests.
class TestCommand extends Command<bool> with ArgUtils<bool> {
  TestCommand() {
    argParser
      ..addFlag(
        'start-paused',
        help:
            'Pauses the browser before running a test, giving you an '
            'opportunity to add breakpoints or inspect loaded code before '
            'running the code.',
      )
      ..addFlag('verbose', abbr: 'v', help: 'Enable verbose output.')
      ..addFlag(
        'watch',
        abbr: 'w',
        help:
            'Run in watch mode so the tests re-run whenever a change is '
            'made.',
      )
      ..addFlag(
        'list',
        help:
            'Lists the bundles that would be compiled and the suites that '
            'will be run as part of this invocation, without actually '
            'compiling or running them.',
      )
      ..addFlag(
        'compile',
        help:
            'Compile test bundles. If this is specified on its own, we will '
            'only compile and not run the suites.',
      )
      ..addFlag(
        'run',
        help:
            'Run test suites. If this is specified on its own, we will only '
            'run the suites and not compile the bundles.',
      )
      ..addFlag(
        'copy-artifacts',
        help:
            'Copy artifacts needed for test suites. If this is specified on '
            'its own, we will only copy the artifacts and not compile or run'
            'the tests bundles or suites.',
      )
      ..addFlag('profile', help: 'Use artifacts from the profile build instead of release.')
      ..addFlag('debug', help: 'Use artifacts from the debug build instead of release.')
      ..addFlag('release', help: 'Use artifacts from the release build. This is the default.')
      ..addFlag('gcs-prod', help: 'Use artifacts from the prod gcs bucket populated by CI.')
      ..addFlag('gcs-staging', help: 'Use artifacts from the staging gcs bucket populated by CI.')
      ..addFlag('gcs-try', help: 'Use artifacts from the try gcs bucket populated by CI.')
      ..addFlag('dwarf', help: 'Debug wasm modules using embedded DWARF data.')
      ..addFlag(
        'require-skia-gold',
        help:
            'Whether we require Skia Gold to be available or not. When this '
            'flag is true, the tests will fail if Skia Gold is not available.',
      )
      ..addFlag(
        'update-screenshot-goldens',
        help:
            'When running screenshot tests writes them to the file system into '
            '.dart_tool/goldens. Use this option to bulk-update all screenshots, '
            'for example, when a new browser version affects pixels.',
      )
      ..addMultiOption('browser', help: 'Filter test suites by browser.')
      ..addMultiOption('compiler', help: 'Filter test suites by compiler.')
      ..addMultiOption('renderer', help: 'Filter test suites by renderer.')
      ..addMultiOption('canvaskit-variant', help: 'Filter test suites by CanvasKit variant.')
      ..addMultiOption('suite', help: 'Filter test suites by suite name.')
      ..addMultiOption('bundle', help: 'Filter test suites by bundle name.')
      ..addFlag(
        'fail-early',
        help:
            'If set, causes the test runner to exit upon the first test '
            'failure. If not set, the test runner will continue running '
            'test despite failures and will report them after all tests '
            'finish.',
      )
      ..addOption(
        'canvaskit-path',
        help:
            'Optional. The path to a local build of CanvasKit to use in '
            'tests. If omitted, the test runner uses the default CanvasKit '
            'build.',
      )
      ..addFlag('wasm', help: 'Whether the test we are running are compiled to webassembly.');
  }

  @override
  final String name = 'test';

  @override
  final String description = 'Run tests.';

  bool get isWatchMode => boolArg('watch');

  bool get isList => boolArg('list');

  bool get failEarly => boolArg('fail-early');

  /// Whether to start the browser in debug mode.
  ///
  /// In this mode the browser pauses before running the test to allow
  /// you set breakpoints or inspect the code.
  bool get startPaused => boolArg('start-paused');

  bool get isVerbose => boolArg('verbose');

  /// The target test files to run.
  List<FilePath> get targetFiles =>
      argResults!.rest.map((String t) => FilePath.fromCwd(t)).toList();

  /// When running screenshot tests, require Skia Gold to be available and
  /// reachable.
  bool get requireSkiaGold => boolArg('require-skia-gold');

  /// When running screenshot tests writes them to the file system into
  /// ".dart_tool/goldens".
  bool get doUpdateScreenshotGoldens => boolArg('update-screenshot-goldens');

  /// Path to a CanvasKit build. Overrides the default CanvasKit.
  String? get overridePathToCanvasKit => argResults!['canvaskit-path'] as String?;

  final FeltConfig config = FeltConfig.fromFile(
    path.join(environment.webUiTestDir.path, 'felt_config.yaml'),
  );

  BrowserSuiteFilter? makeBrowserFilter() {
    final List<String>? browserArgs = argResults!['browser'] as List<String>?;
    if (browserArgs == null || browserArgs.isEmpty) {
      return null;
    }
    final Set<BrowserName> browserNames = Set<BrowserName>.from(
      browserArgs.map((String arg) => BrowserName.values.byName(arg)),
    );
    return BrowserSuiteFilter(allowList: browserNames);
  }

  CompilerFilter? makeCompilerFilter() {
    final List<String>? compilerArgs = argResults!['compiler'] as List<String>?;
    if (compilerArgs == null || compilerArgs.isEmpty) {
      return null;
    }
    final Set<Compiler> compilers = Set<Compiler>.from(
      compilerArgs.map((String arg) => Compiler.values.byName(arg)),
    );
    return CompilerFilter(allowList: compilers);
  }

  RendererFilter? makeRendererFilter() {
    final List<String>? rendererArgs = argResults!['renderer'] as List<String>?;
    if (rendererArgs == null || rendererArgs.isEmpty) {
      return null;
    }
    final Set<Renderer> renderers = Set<Renderer>.from(
      rendererArgs.map((String arg) => Renderer.values.byName(arg)),
    );
    return RendererFilter(allowList: renderers);
  }

  CanvasKitVariantFilter? makeCanvasKitVariantFilter() {
    final List<String>? variantArgs = argResults!['canvaskit-variant'] as List<String>?;
    if (variantArgs == null || variantArgs.isEmpty) {
      return null;
    }
    final Set<CanvasKitVariant> variants = Set<CanvasKitVariant>.from(
      variantArgs.map((String arg) => CanvasKitVariant.values.byName(arg)),
    );
    return CanvasKitVariantFilter(allowList: variants);
  }

  SuiteNameFilter? makeSuiteNameFilter() {
    final List<String>? suiteNameArgs = argResults!['suite'] as List<String>?;
    if (suiteNameArgs == null || suiteNameArgs.isEmpty) {
      return null;
    }

    final Iterable<String> allSuiteNames = config.testSuites.map((TestSuite suite) => suite.name);
    for (final String suiteName in suiteNameArgs) {
      if (!allSuiteNames.contains(suiteName)) {
        throw ToolExit('No suite found named $suiteName');
      }
    }
    return SuiteNameFilter(allowList: Set<String>.from(suiteNameArgs));
  }

  BundleNameFilter? makeBundleNameFilter() {
    final List<String>? bundleNameArgs = argResults!['bundle'] as List<String>?;
    if (bundleNameArgs == null || bundleNameArgs.isEmpty) {
      return null;
    }

    final Iterable<String> allBundleNames = config.testSuites.map(
      (TestSuite suite) => suite.testBundle.name,
    );
    for (final String bundleName in bundleNameArgs) {
      if (!allBundleNames.contains(bundleName)) {
        throw ToolExit('No bundle found named $bundleName');
      }
    }
    return BundleNameFilter(allowList: Set<String>.from(bundleNameArgs));
  }

  FileFilter? makeFileFilter() {
    final List<FilePath> tests = targetFiles;
    if (tests.isEmpty) {
      return null;
    }
    final Set<String> bundleNames = <String>{};
    for (final FilePath testPath in tests) {
      if (!io.File(testPath.absolute).existsSync()) {
        throw ToolExit('Test path not found: $testPath');
      }
      bool bundleFound = false;
      for (final TestBundle bundle in config.testBundles) {
        final String testSetPath = getTestSetDirectory(bundle.testSet).path;
        if (path.isWithin(testSetPath, testPath.absolute)) {
          bundleFound = true;
          bundleNames.add(bundle.name);
        }
      }
      if (!bundleFound) {
        throw ToolExit('Test path not in any known test bundle: $testPath');
      }
    }
    return FileFilter(allowList: bundleNames);
  }

  List<SuiteFilter> get suiteFilters {
    final BrowserSuiteFilter? browserFilter = makeBrowserFilter();
    final CompilerFilter? compilerFilter = makeCompilerFilter();
    final RendererFilter? rendererFilter = makeRendererFilter();
    final CanvasKitVariantFilter? canvaskitVariantFilter = makeCanvasKitVariantFilter();
    final SuiteNameFilter? suiteNameFilter = makeSuiteNameFilter();
    final BundleNameFilter? bundleNameFilter = makeBundleNameFilter();
    final FileFilter? fileFilter = makeFileFilter();
    return <SuiteFilter>[
      PlatformBrowserFilter(),
      if (browserFilter != null) browserFilter,
      if (compilerFilter != null) compilerFilter,
      if (rendererFilter != null) rendererFilter,
      if (canvaskitVariantFilter != null) canvaskitVariantFilter,
      if (suiteNameFilter != null) suiteNameFilter,
      if (bundleNameFilter != null) bundleNameFilter,
      if (fileFilter != null) fileFilter,
    ];
  }

  List<TestSuite> _filterTestSuites() {
    if (isVerbose) {
      print('Filtering suites...');
    }
    final List<SuiteFilter> filters = suiteFilters;
    final List<TestSuite> filteredSuites =
        config.testSuites.where((TestSuite suite) {
          for (final SuiteFilter filter in filters) {
            final SuiteFilterResult result = filter.filterSuite(suite);
            if (!result.isAccepted) {
              if (isVerbose) {
                print('  ${suite.name.ansiCyan} rejected for reason: ${result.rejectReason}');
              }
              return false;
            }
          }
          return true;
        }).toList();
    return filteredSuites;
  }

  List<TestBundle> _filterBundlesForSuites(List<TestSuite> suites) {
    final Set<TestBundle> seenBundles = Set<TestBundle>.from(
      suites.map((TestSuite suite) => suite.testBundle),
    );
    return config.testBundles.where((TestBundle bundle) => seenBundles.contains(bundle)).toList();
  }

  ArtifactDependencies _artifactsForSuites(List<TestSuite> suites) {
    return suites.fold(
      ArtifactDependencies.none(),
      (ArtifactDependencies deps, TestSuite suite) => deps | suite.artifactDependencies,
    );
  }

  ArtifactSource get artifactSource {
    final List<ArtifactSource> sources = <ArtifactSource>[];
    if (boolArg('debug')) {
      sources.add(LocalArtifactSource(mode: RuntimeMode.debug));
    }
    if (boolArg('profile')) {
      sources.add(LocalArtifactSource(mode: RuntimeMode.profile));
    }
    if (boolArg('release')) {
      sources.add(LocalArtifactSource(mode: RuntimeMode.release));
    }
    if (boolArg('gcs-prod')) {
      sources.add(GcsArtifactSource(realm: LuciRealm.Prod));
    }
    if (boolArg('gcs-staging')) {
      sources.add(GcsArtifactSource(realm: LuciRealm.Staging));
    }
    if (boolArg('gcs-try')) {
      sources.add(GcsArtifactSource(realm: LuciRealm.Try));
    }
    if (sources.length > 1) {
      throw ToolExit('Cannot specify more than one artifact source.');
    }
    if (sources.length == 1) {
      return sources.first;
    }
    final realm = luciConfig?.realm;
    if (realm != null) {
      return GcsArtifactSource(realm: realm);
    } else {
      return LocalArtifactSource(mode: RuntimeMode.release);
    }
  }

  @override
  Future<bool> run() async {
    final List<TestSuite> filteredSuites = _filterTestSuites();
    final List<TestBundle> bundles = _filterBundlesForSuites(filteredSuites);
    final ArtifactDependencies artifacts = _artifactsForSuites(filteredSuites);
    if (isList || isVerbose) {
      print('Suites:');
      for (final TestSuite suite in filteredSuites) {
        print('  ${suite.name.ansiCyan}');
      }
      print('Bundles:');
      for (final TestBundle bundle in bundles) {
        print('  ${bundle.name.ansiMagenta}');
      }
      print('Artifacts:');
      if (artifacts.canvasKit) {
        print('  canvaskit'.ansiYellow);
      }
      if (artifacts.canvasKitChromium) {
        print('  canvaskit_chromium'.ansiYellow);
      }
      if (artifacts.skwasm) {
        print('  skwasm'.ansiYellow);
      }
    }
    if (isList) {
      return true;
    }

    bool shouldRun = boolArg('run');
    bool shouldCompile = boolArg('compile');
    bool shouldCopyArtifacts = boolArg('copy-artifacts');
    if (!shouldRun && !shouldCompile && !shouldCopyArtifacts) {
      // If none of these is specified, we should assume we need to do all of them.
      shouldRun = true;
      shouldCompile = true;
      shouldCopyArtifacts = true;
    }

    final Set<FilePath>? testFiles = targetFiles.isEmpty ? null : Set<FilePath>.from(targetFiles);
    final Pipeline testPipeline = Pipeline(
      steps: <PipelineStep>[
        if (isWatchMode) ClearTerminalScreenStep(),
        if (shouldCopyArtifacts) CopyArtifactsStep(artifacts, source: artifactSource),
        if (shouldCompile)
          for (final TestBundle bundle in bundles)
            CompileBundleStep(bundle: bundle, isVerbose: isVerbose, testFiles: testFiles),
        if (shouldRun)
          for (final TestSuite suite in filteredSuites)
            RunSuiteStep(
              suite,
              startPaused: startPaused,
              isVerbose: isVerbose,
              doUpdateScreenshotGoldens: doUpdateScreenshotGoldens,
              requireSkiaGold: requireSkiaGold,
              overridePathToCanvasKit: overridePathToCanvasKit,
              testFiles: testFiles,
              useDwarf: boolArg('dwarf'),
            ),
      ],
    );

    try {
      await testPipeline.run();
      if (isWatchMode) {
        print('');
        print('Initial test succeeded!');
      }
    } catch (error, stackTrace) {
      if (isWatchMode) {
        // The error is printed but not rethrown in watch mode because
        // failures are expected. The idea is that the developer corrects the
        // error, saves the file, and the pipeline reruns.
        print('');
        print('Initial test failed!\n');
        print(error);
        print(stackTrace);
      } else {
        rethrow;
      }
    }

    if (isWatchMode) {
      final FilePath dir = FilePath.fromWebUi('');
      print('');
      print('Watching ${dir.relativeToCwd}/lib and ${dir.relativeToCwd}/test to re-run tests');
      print('');
      await PipelineWatcher(
        dir: dir.absolute,
        pipeline: testPipeline,
        ignore: (WatchEvent event) {
          // Ignore font files that are copied whenever tests run.
          if (event.path.endsWith('.ttf')) {
            return true;
          }

          // React to changes in lib/ and test/ folders.
          final String relativePath = path.relative(event.path, from: dir.absolute);
          if (path.isWithin('lib', relativePath) || path.isWithin('test', relativePath)) {
            return false;
          }

          // Ignore anything else.
          return true;
        },
      ).start();
    }
    return true;
  }
}

/// Clears the terminal screen and places the cursor at the top left corner.
///
/// This works on Linux and Mac. On Windows, it's a no-op.
class ClearTerminalScreenStep implements PipelineStep {
  @override
  String get description => 'clearing terminal screen';

  @override
  bool get isSafeToInterrupt => false;

  @override
  Future<void> interrupt() async {}

  @override
  Future<void> run() async {
    if (!io.Platform.isWindows) {
      // See: https://en.wikipedia.org/wiki/ANSI_escape_code#CSI_sequences
      print('\x1B[2J\x1B[1;2H');
    }
  }
}
