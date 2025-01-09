// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:coverage/coverage.dart' as coverage;
import 'package:meta/meta.dart';

import '../base/file_system.dart';
import '../base/io.dart';
import '../base/process.dart';
import '../globals.dart' as globals;
import '../vmservice.dart';

import 'test_device.dart';
import 'test_time_recorder.dart';
import 'watcher.dart';

/// A class that collects code coverage data during test runs.
class CoverageCollector extends TestWatcher {
  CoverageCollector({
    this.libraryNames,
    this.verbose = true,
    required this.packagesPath,
    this.resolver,
    this.testTimeRecorder,
    this.branchCoverage = false,
  });

  /// True when log messages should be emitted.
  final bool verbose;

  /// The path to the package_config.json of the package for which code
  /// coverage is computed.
  final String packagesPath;

  /// Map of file path to coverage hit map for that file.
  Map<String, coverage.HitMap>? _globalHitmap;

  /// The names of the libraries to gather coverage for. If null, all libraries
  /// will be accepted.
  Set<String>? libraryNames;

  final coverage.Resolver? resolver;
  final Map<String, List<List<int>>?> _ignoredLinesInFilesCache = <String, List<List<int>>?>{};
  final Map<String, Set<int>> _coverableLineCache = <String, Set<int>>{};

  final TestTimeRecorder? testTimeRecorder;

  /// Whether to collect branch coverage information.
  bool branchCoverage;

  static Future<coverage.Resolver> getResolver(String? packagesPath) async {
    try {
      return await coverage.Resolver.create(packagesPath: packagesPath);
    } on FileSystemException {
      // When given a bad packages path (as for instance done in some tests)
      // just ignore it and return one without a packages path.
      return coverage.Resolver.create();
    }
  }

  @override
  Future<void> handleFinishedTest(TestDevice testDevice) async {
    _logMessage('Starting coverage collection');
    await collectCoverage(testDevice);
  }

  void _logMessage(String line, {bool error = false}) {
    if (!verbose) {
      return;
    }
    if (error) {
      globals.printError(line);
    } else {
      globals.printTrace(line);
    }
  }

  void _addHitmap(Map<String, coverage.HitMap> hitmap) {
    final Stopwatch? stopwatch = testTimeRecorder?.start(TestTimePhases.CoverageAddHitmap);
    if (_globalHitmap == null) {
      _globalHitmap = hitmap;
    } else {
      _globalHitmap!.merge(hitmap);
    }
    testTimeRecorder?.stop(TestTimePhases.CoverageAddHitmap, stopwatch!);
  }

  /// The directory of the package for which coverage is being collected.
  String get packageDirectory {
    // The coverage package expects the directory of the package itself, and
    // uses that to locate the package_info.json file, which it treats as a
    // private implementation detail. In general, the package_info.json file is
    // located in `.dart_tool/package_info.json` relative to the package
    // directory, so we return the grandparent directory of that file.
    //
    // This may not be a safe assumption in non-standard environments, such as
    // when building under build systems such as Bazel. In those cases, this
    // getter should be overridden.
    return globals.fs.directory(globals.fs.file(packagesPath).dirname).dirname;
  }

  /// Collects coverage for an isolate using the given `port`.
  ///
  /// This should be called when the code whose coverage data is being collected
  /// has been run to completion so that all coverage data has been recorded.
  ///
  /// The returned [Future] completes when the coverage is collected.
  Future<void> collectCoverageIsolate(Uri vmServiceUri) async {
    _logMessage('collecting coverage data from $vmServiceUri...');
    final Map<String, dynamic> data = await collect(
      vmServiceUri,
      libraryNames,
      branchCoverage: branchCoverage,
      coverableLineCache: _coverableLineCache,
    );

    _logMessage('($vmServiceUri): collected coverage data; merging...');
    _addHitmap(
      await coverage.HitMap.parseJson(
        data['coverage'] as List<Map<String, dynamic>>,
        packagePath: packageDirectory,
        checkIgnoredLines: true,
      ),
    );
    _logMessage('($vmServiceUri): done merging coverage data into global coverage map.');
  }

  /// Collects coverage for the given [Process] using the given `port`.
  ///
  /// This should be called when the code whose coverage data is being collected
  /// has been run to completion so that all coverage data has been recorded.
  ///
  /// The returned [Future] completes when the coverage is collected.
  Future<void> collectCoverage(
    TestDevice testDevice, {
    @visibleForTesting FlutterVmService? serviceOverride,
  }) async {
    final Stopwatch? totalTestTimeRecorderStopwatch = testTimeRecorder?.start(
      TestTimePhases.CoverageTotal,
    );

    late Map<String, dynamic> data;

    final Stopwatch? collectTestTimeRecorderStopwatch = testTimeRecorder?.start(
      TestTimePhases.CoverageCollect,
    );

    final Future<void> processComplete = testDevice.finished.then(
      (Object? obj) => obj,
      onError: (Object error, StackTrace stackTrace) {
        if (error is TestDeviceException) {
          throw Exception(
            'Failed to collect coverage, test device terminated prematurely with '
            'error: ${error.message}.\n$stackTrace',
          );
        }
        return Future<Object?>.error(error, stackTrace);
      },
    );

    final Future<void> collectionComplete = testDevice.vmServiceUri.then((Uri? vmServiceUri) {
      _logMessage('collecting coverage data from $testDevice at $vmServiceUri...');
      return collect(
        vmServiceUri!,
        libraryNames,
        serviceOverride: serviceOverride,
        branchCoverage: branchCoverage,
        coverableLineCache: _coverableLineCache,
      ).then<void>((Map<String, dynamic> result) {
        _logMessage('Collected coverage data.');
        data = result;
      });
    });

    await Future.any<void>(<Future<void>>[processComplete, collectionComplete]);

    testTimeRecorder?.stop(TestTimePhases.CoverageCollect, collectTestTimeRecorderStopwatch!);

    _logMessage('Merging coverage data...');
    final Stopwatch? parseTestTimeRecorderStopwatch = testTimeRecorder?.start(
      TestTimePhases.CoverageParseJson,
    );

    final Map<String, coverage.HitMap> hitmap = coverage.HitMap.parseJsonSync(
      data['coverage'] as List<Map<String, dynamic>>,
      checkIgnoredLines: true,
      resolver: resolver ?? await CoverageCollector.getResolver(packageDirectory),
      ignoredLinesInFilesCache: _ignoredLinesInFilesCache,
    );
    testTimeRecorder?.stop(TestTimePhases.CoverageParseJson, parseTestTimeRecorderStopwatch!);

    _addHitmap(hitmap);
    _logMessage('Done merging coverage data into global coverage map.');
    testTimeRecorder?.stop(TestTimePhases.CoverageTotal, totalTestTimeRecorderStopwatch!);
  }

  /// Returns formatted coverage data once all coverage data has been collected.
  ///
  /// This will not start any collection tasks. It us up to the caller of to
  /// call [collectCoverage] for each process first.
  Future<String?> finalizeCoverage({
    String Function(Map<String, coverage.HitMap> hitmap)? formatter,
    coverage.Resolver? resolver,
    Directory? coverageDirectory,
  }) async {
    if (_globalHitmap == null) {
      return null;
    }
    if (formatter == null) {
      final coverage.Resolver usedResolver =
          resolver ?? this.resolver ?? await CoverageCollector.getResolver(packagesPath);
      final String packagePath = globals.fs.currentDirectory.path;
      // find paths for libraryNames so we can include them to report
      final List<String>? libraryPaths =
          libraryNames
              ?.map((String e) => usedResolver.resolve('package:$e'))
              .whereType<String>()
              .toList();
      final List<String>? reportOn =
          coverageDirectory == null ? libraryPaths : <String>[coverageDirectory.path];
      formatter =
          (Map<String, coverage.HitMap> hitmap) =>
              hitmap.formatLcov(usedResolver, reportOn: reportOn, basePath: packagePath);
    }
    final String result = formatter(_globalHitmap!);
    _globalHitmap = null;
    return result;
  }

  Future<bool> collectCoverageData(
    String? coveragePath, {
    bool mergeCoverageData = false,
    Directory? coverageDirectory,
  }) async {
    final String? coverageData = await finalizeCoverage(coverageDirectory: coverageDirectory);
    _logMessage('coverage information collection complete');
    if (coverageData == null) {
      return false;
    }

    final File coverageFile =
        globals.fs.file(coveragePath)
          ..createSync(recursive: true)
          ..writeAsStringSync(coverageData, flush: true);
    _logMessage('wrote coverage data to $coveragePath (size=${coverageData.length})');

    const String baseCoverageData = 'coverage/lcov.base.info';
    if (mergeCoverageData) {
      if (!globals.fs.isFileSync(baseCoverageData)) {
        _logMessage('Missing "$baseCoverageData". Unable to merge coverage data.', error: true);
        return false;
      }

      if (globals.os.which('lcov') == null) {
        String installMessage = 'Please install lcov.';
        if (globals.platform.isLinux) {
          installMessage = 'Consider running "sudo apt-get install lcov".';
        } else if (globals.platform.isMacOS) {
          installMessage = 'Consider running "brew install lcov".';
        }
        _logMessage(
          'Missing "lcov" tool. Unable to merge coverage data.\n$installMessage',
          error: true,
        );
        return false;
      }

      final Directory tempDir = globals.fs.systemTempDirectory.createTempSync(
        'flutter_tools_test_coverage.',
      );
      try {
        final File sourceFile = coverageFile.copySync(
          globals.fs.path.join(tempDir.path, 'lcov.source.info'),
        );
        final RunResult result = globals.processUtils.runSync(<String>[
          'lcov',
          '--add-tracefile',
          baseCoverageData,
          '--add-tracefile',
          sourceFile.path,
          '--output-file',
          coverageFile.path,
        ]);
        if (result.exitCode != 0) {
          return false;
        }
      } finally {
        tempDir.deleteSync(recursive: true);
      }
    }
    return true;
  }

  @override
  Future<void> handleTestCrashed(TestDevice testDevice) async {}

  @override
  Future<void> handleTestTimedOut(TestDevice testDevice) async {}
}

Future<Map<String, dynamic>> collect(
  Uri serviceUri,
  Set<String>? libraryNames, {
  bool waitPaused = false,
  String? debugName,
  @visibleForTesting bool forceSequential = false,
  @visibleForTesting FlutterVmService? serviceOverride,
  bool branchCoverage = false,
  Map<String, Set<int>>? coverableLineCache,
}) {
  return coverage.collect(
    serviceUri,
    false,
    false,
    false,
    libraryNames,
    serviceOverrideForTesting: serviceOverride?.service,
    branchCoverage: branchCoverage,
    coverableLineCache: coverableLineCache,
  );
}
