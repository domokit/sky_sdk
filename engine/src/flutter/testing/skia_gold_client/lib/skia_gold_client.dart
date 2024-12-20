// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert';
import 'dart:io' as io;

import 'package:crypto/crypto.dart';
import 'package:engine_repo_tools/engine_repo_tools.dart';
import 'package:meta/meta.dart';
import 'package:path/path.dart' as path;
import 'package:process/process.dart';

import 'src/errors.dart';
import 'src/release_version.dart';

export 'src/errors.dart' show SkiaGoldProcessError;

const String _kGoldctlKey = 'GOLDCTL';
const String _kPresubmitEnvName = 'GOLD_TRYJOB';
const String _kLuciEnvName = 'LUCI_CONTEXT';

const String _skiaGoldHost = 'https://flutter-engine-gold.skia.org';
const String _instance = 'flutter-engine';

/// Uploads images and makes baseline requests to Skia Gold.
///
/// For an example of how to use this class, see `tool/e2e_test.dart`.
interface class SkiaGoldClient {
  /// Creates a [SkiaGoldClient] with the given [workDirectory].
  ///
  /// A set of [dimensions] can be provided to add attributes about the
  /// environment used to generate the screenshots, which are treated as keys
  /// for the image:
  ///
  /// ```dart
  /// final SkiaGoldClient skiaGoldClient = SkiaGoldClient(
  ///   someDir,
  ///   dimensions: <String, String>{
  ///     'platform': 'linux',
  ///   },
  /// );
  /// ```
  ///
  /// The [verbose] flag is intended for use in debugging CI issues, and
  /// produces more detailed output that some may find useful, but would be too
  /// spammy for regular use.
  factory SkiaGoldClient(
    io.Directory workDirectory, {
    Map<String, String>? dimensions,
    bool verbose = false,
  }) {
    return SkiaGoldClient.forTesting(workDirectory, dimensions: dimensions, verbose: verbose);
  }

  /// Creates a [SkiaGoldClient] for testing.
  ///
  /// Similar to the default constructor, but allows for dependency injection
  /// for testing purposes:
  ///
  /// - [httpClient] makes requests to Skia Gold to fetch expectations.
  /// - [processManager] launches sub-processes.
  /// - [stderr] is where output is written for diagnostics.
  /// - [environment] is the environment variables for the currently running
  ///   process, and is used to determine if Skia Gold is available, and whether
  ///   the current environment is CI, and if so, if it's pre-submit or
  ///   post-submit.
  /// - [engineRoot] is the root of the engine repository, which is used for
  ///   finding the current commit hash, as well as the location of the
  ///   `.engine-release.version` file.
  @visibleForTesting
  SkiaGoldClient.forTesting(
    this.workDirectory, {
    this.dimensions,
    this.verbose = false,
    io.HttpClient? httpClient,
    ProcessManager? processManager,
    StringSink? stderr,
    Map<String, String>? environment,
    Engine? engineRoot,
  }) : httpClient = httpClient ?? io.HttpClient(),
       process = processManager ?? const LocalProcessManager(),
       _stderr = stderr ?? io.stderr,
       _environment = environment ?? io.Platform.environment,
       _engineRoot = engineRoot ?? Engine.findWithin() {
    // Lookup the release version from the engine repository.
    final io.File releaseVersionFile = io.File(
      path.join(_engineRoot.flutterDir.path, '.engine-release.version'),
    );

    // If the file is not found or cannot be read, we are in an invalid state.
    try {
      _releaseVersion = ReleaseVersion.parse(releaseVersionFile.readAsStringSync());
    } on FormatException catch (error) {
      throw StateError('Failed to parse release version file: $error.');
    } on io.FileSystemException catch (error) {
      throw StateError('Failed to read release version file: $error.');
    }
  }

  /// The root of the engine repository.
  final Engine _engineRoot;
  ReleaseVersion? _releaseVersion;

  /// Whether the client is available and can be used in this environment.
  static bool isAvailable({Map<String, String>? environment}) {
    final String? result = (environment ?? io.Platform.environment)[_kGoldctlKey];
    return result != null && result.isNotEmpty;
  }

  /// Returns true if the current environment is a LUCI builder.
  static bool isLuciEnv({Map<String, String>? environment}) {
    return (environment ?? io.Platform.environment).containsKey(_kLuciEnvName);
  }

  /// Whether the current environment is a presubmit job.
  bool get _isPresubmit {
    return isLuciEnv(environment: _environment) &&
        isAvailable(environment: _environment) &&
        _environment.containsKey(_kPresubmitEnvName);
  }

  /// Whether the current environment is a postsubmit job.
  bool get _isPostsubmit {
    return isLuciEnv(environment: _environment) &&
        isAvailable(environment: _environment) &&
        !_environment.containsKey(_kPresubmitEnvName);
  }

  /// Whether to print verbose output from goldctl.
  ///
  /// This flag is intended for use in debugging CI issues, and should not
  /// ordinarily be set to true.
  final bool verbose;

  /// Environment variables for the currently running process.
  final Map<String, String> _environment;

  /// Where output is written for diagnostics.
  final StringSink _stderr;

  /// Allows to add attributes about the environment used to generate the screenshots.
  final Map<String, String>? dimensions;

  /// A controller for launching sub-processes.
  final ProcessManager process;

  /// A client for making Http requests to the Flutter Gold dashboard.
  final io.HttpClient httpClient;

  /// The local [Directory] for the current test context. In this directory, the
  /// client will create image and JSON files for the `goldctl` tool to use.
  final io.Directory workDirectory;

  String get _tempPath => path.join(workDirectory.path, 'temp');
  String get _keysPath => path.join(workDirectory.path, 'keys.json');
  String get _failuresPath => path.join(workDirectory.path, 'failures.json');

  Future<void>? _initResult;
  Future<void> _initOnce(Future<void> Function() callback) {
    // If a call has already been made, return the result of that call.
    _initResult ??= callback();
    return _initResult!;
  }

  /// Indicates whether the client has already been authorized to communicate
  /// with the Skia Gold backend.
  bool get _isAuthorized {
    final io.File authFile = io.File(path.join(_tempPath, 'auth_opt.json'));

    if (authFile.existsSync()) {
      final String contents = authFile.readAsStringSync();
      final Map<String, dynamic> decoded = json.decode(contents) as Map<String, dynamic>;
      return !(decoded['GSUtil'] as bool);
    }
    return false;
  }

  /// The path to the local [Directory] where the `goldctl` tool is hosted.
  String get _goldctl {
    final String? result = _environment[_kGoldctlKey];
    if (result == null || result.isEmpty) {
      throw StateError('The environment variable $_kGoldctlKey is not set.');
    }
    return result;
  }

  /// Prepares the local work space for golden file testing and calls the
  /// `goldctl auth` command.
  ///
  /// This ensures that the `goldctl` tool is authorized and ready for testing.
  Future<void> auth() async {
    if (_isAuthorized) {
      return;
    }
    final List<String> authCommand = <String>[
      _goldctl,
      'auth',
      if (verbose) '--verbose',
      '--work-dir',
      _tempPath,
      '--luci',
    ];

    final io.ProcessResult result = await _runCommand(authCommand);

    if (result.exitCode != 0) {
      final StringBuffer buf =
          StringBuffer()
            ..writeln('Skia Gold authorization failed.')
            ..writeln(
              'Luci environments authenticate using the file provided '
              'by LUCI_CONTEXT. There may be an error with this file or Gold '
              'authentication.',
            );
      throw SkiaGoldProcessError(
        command: authCommand,
        stdout: result.stdout.toString(),
        stderr: result.stderr.toString(),
        message: buf.toString(),
      );
    } else if (verbose) {
      _stderr.writeln('stdout:\n${result.stdout}');
      _stderr.writeln('stderr:\n${result.stderr}');
    }
  }

  Future<io.ProcessResult> _runCommand(List<String> command) {
    return process.run(command);
  }

  /// Executes the `imgtest init` command in the `goldctl` tool.
  ///
  /// The `imgtest` command collects and uploads test results to the Skia Gold
  /// backend, the `init` argument initializes the current test.
  Future<void> _imgtestInit() async {
    final io.File keys = io.File(_keysPath);
    final io.File failures = io.File(_failuresPath);

    await keys.writeAsString(_getKeysJSON());
    await failures.create();
    final String commitHash = await _getCurrentCommit();

    final List<String> imgtestInitCommand = <String>[
      _goldctl,
      'imgtest',
      'init',
      if (verbose) '--verbose',
      '--instance',
      _instance,
      '--work-dir',
      _tempPath,
      '--commit',
      commitHash,
      '--keys-file',
      keys.path,
      '--failure-file',
      failures.path,
      '--passfail',
    ];

    final io.ProcessResult result = await _runCommand(imgtestInitCommand);

    if (result.exitCode != 0) {
      final StringBuffer buf =
          StringBuffer()
            ..writeln('Skia Gold imgtest init failed.')
            ..writeln('An error occurred when initializing golden file test with ')
            ..writeln('goldctl.');
      throw SkiaGoldProcessError(
        command: imgtestInitCommand,
        stdout: result.stdout.toString(),
        stderr: result.stderr.toString(),
        message: buf.toString(),
      );
    } else if (verbose) {
      _stderr.writeln('stdout:\n${result.stdout}');
      _stderr.writeln('stderr:\n${result.stderr}');
    }
  }

  /// Executes the `imgtest add` command in the `goldctl` tool.
  ///
  /// The `imgtest` command collects and uploads test results to the Skia Gold
  /// backend, the `add` argument uploads the current image test.
  ///
  /// Throws an exception for try jobs that failed to pass the pixel comparison.
  ///
  /// The [testName] and [goldenFile] parameters reference the current
  /// comparison being evaluated.
  ///
  /// [pixelColorDelta] defines maximum acceptable difference in RGB channels of
  /// each pixel, such that:
  ///
  /// ```dart
  /// bool isSame(Color image, Color golden, int pixelDeltaThreshold) {
  ///   return abs(image.r - golden.r)
  ///     + abs(image.g - golden.g)
  ///     + abs(image.b - golden.b) <= pixelDeltaThreshold;
  /// }
  /// ```
  ///
  /// [differentPixelsRate] is the fraction of pixels that can differ, as
  /// determined by the [pixelColorDelta] parameter. It's in the range [0.0,
  /// 1.0] and defaults to 0.01. A value of 0.01 means that 1% of the pixels are
  /// allowed to be different.
  ///
  /// ## Release Testing
  ///
  /// In release branches, we add a unique test suffix to the test name. For
  /// example "testName" -> "testName_Release_3_21", based on the version in the
  /// `.engine-release.version` file at the root of the engine repository.
  ///
  /// See <../README.md#release-testing> for more information.
  Future<void> addImg(
    String testName,
    io.File goldenFile, {
    double differentPixelsRate = 0.01,
    int pixelColorDelta = 0,
    required int screenshotSize,
  }) async {
    assert(_isPresubmit || _isPostsubmit);

    // Clean the test name to remove the file extension.
    testName = path.basenameWithoutExtension(testName);

    // In release branches, we add a unique test suffix to the test name.
    // For example "testName" -> "testName_Release_3_21".
    // See ../README.md#release-testing for more information.
    if (_releaseVersion case final ReleaseVersion v) {
      testName = '${testName}_Release_${v.major}_${v.minor}';
    }

    if (_isPresubmit) {
      await _tryjobAdd(testName, goldenFile, screenshotSize, pixelColorDelta, differentPixelsRate);
    }
    if (_isPostsubmit) {
      await _imgtestAdd(testName, goldenFile, screenshotSize, pixelColorDelta, differentPixelsRate);
    }
  }

  /// Executes the `imgtest add` command in the `goldctl` tool.
  ///
  /// The `imgtest` command collects and uploads test results to the Skia Gold
  /// backend, the `add` argument uploads the current image test. A response is
  /// returned from the invocation of this command that indicates a pass or fail
  /// result.
  ///
  /// The [testName] and [goldenFile] parameters reference the current
  /// comparison being evaluated.
  Future<void> _imgtestAdd(
    String testName,
    io.File goldenFile,
    int screenshotSize,
    int pixelDeltaThreshold,
    double maxDifferentPixelsRate,
  ) async {
    await _initOnce(_imgtestInit);

    final List<String> imgtestCommand = <String>[
      _goldctl,
      'imgtest',
      'add',
      if (verbose) '--verbose',
      '--work-dir',
      _tempPath,
      '--test-name',
      testName,
      '--png-file',
      goldenFile.path,
      // Otherwise post submit will not fail.
      '--passfail',
      ..._getMatchingArguments(
        testName,
        screenshotSize,
        pixelDeltaThreshold,
        maxDifferentPixelsRate,
      ),
    ];

    final io.ProcessResult result = await _runCommand(imgtestCommand);

    if (result.exitCode != 0) {
      final StringBuffer buf =
          StringBuffer()
            ..writeln('Skia Gold received an unapproved image in post-submit ')
            ..writeln('testing. Golden file images in flutter/engine are triaged ')
            ..writeln('in pre-submit during code review for the given PR.')
            ..writeln()
            ..writeln('Visit https://flutter-engine-gold.skia.org/ to view and approve ')
            ..writeln('the image(s), or revert the associated change. For more ')
            ..writeln('information, visit the wiki: ')
            ..writeln(
              'https://github.com/flutter/flutter/wiki/Writing-a-golden-file-test-for-package:flutter',
            );
      throw SkiaGoldProcessError(
        command: imgtestCommand,
        stdout: result.stdout.toString(),
        stderr: result.stderr.toString(),
        message: buf.toString(),
      );
    } else if (verbose) {
      _stderr.writeln('stdout:\n${result.stdout}');
      _stderr.writeln('stderr:\n${result.stderr}');
    }
  }

  /// Executes the `imgtest init` command in the `goldctl` tool for tryjobs.
  ///
  /// The `imgtest` command collects and uploads test results to the Skia Gold
  /// backend, the `init` argument initializes the current tryjob.
  Future<void> _tryjobInit() async {
    final io.File keys = io.File(_keysPath);
    final io.File failures = io.File(_failuresPath);

    await keys.writeAsString(_getKeysJSON());
    await failures.create();
    final String commitHash = await _getCurrentCommit();

    final List<String> tryjobInitCommand = <String>[
      _goldctl,
      'imgtest',
      'init',
      if (verbose) '--verbose',
      '--instance',
      _instance,
      '--work-dir',
      _tempPath,
      '--commit',
      commitHash,
      '--keys-file',
      keys.path,
      '--failure-file',
      failures.path,
      '--passfail',
      '--crs',
      'github',
      '--patchset_id',
      commitHash,
      ..._getCIArguments(),
    ];

    final io.ProcessResult result = await _runCommand(tryjobInitCommand);

    if (result.exitCode != 0) {
      final StringBuffer buf =
          StringBuffer()
            ..writeln('Skia Gold tryjobInit failure.')
            ..writeln('An error occurred when initializing golden file tryjob with ')
            ..writeln('goldctl.');
      throw SkiaGoldProcessError(
        command: tryjobInitCommand,
        stdout: result.stdout.toString(),
        stderr: result.stderr.toString(),
        message: buf.toString(),
      );
    } else if (verbose) {
      _stderr.writeln('stdout:\n${result.stdout}');
      _stderr.writeln('stderr:\n${result.stderr}');
    }
  }

  /// Executes the `imgtest add` command in the `goldctl` tool for tryjobs.
  ///
  /// The `imgtest` command collects and uploads test results to the Skia Gold
  /// backend, the `add` argument uploads the current image test. A response is
  /// returned from the invocation of this command that indicates a pass or fail
  /// result for the tryjob.
  ///
  /// The [testName] and [goldenFile] parameters reference the current
  /// comparison being evaluated.
  Future<void> _tryjobAdd(
    String testName,
    io.File goldenFile,
    int screenshotSize,
    int pixelDeltaThreshold,
    double differentPixelsRate,
  ) async {
    await _initOnce(_tryjobInit);

    final List<String> tryjobCommand = <String>[
      _goldctl,
      'imgtest',
      'add',
      if (verbose) '--verbose',
      '--work-dir',
      _tempPath,
      '--test-name',
      testName,
      '--png-file',
      goldenFile.path,
      ..._getMatchingArguments(testName, screenshotSize, pixelDeltaThreshold, differentPixelsRate),
    ];

    final io.ProcessResult result = await _runCommand(tryjobCommand);
    final String resultStdout = result.stdout.toString();
    if (result.exitCode == 0) {
      // In "verbose" (debugging) mode, print the output of the tryjob anyway.
      if (verbose) {
        _stderr.writeln('stdout:\n${result.stdout}');
        _stderr.writeln('stderr:\n${result.stderr}');
      }
    } else {
      // Neither of these conditions are considered failures during tryjobs.
      final bool isUntriaged = resultStdout.contains('Untriaged');
      final bool isNegative = resultStdout.contains('negative image');
      if (!isUntriaged && !isNegative) {
        final StringBuffer buf =
            StringBuffer()
              ..writeln('Unexpected Gold tryjobAdd failure.')
              ..writeln('Tryjob execution for golden file test $testName failed for')
              ..writeln('a reason unrelated to pixel comparison.');
        throw SkiaGoldProcessError(
          command: tryjobCommand,
          stdout: resultStdout,
          stderr: result.stderr.toString(),
          message: buf.toString(),
        );
      }
      // ... but we want to know about them anyway.
      // See https://github.com/flutter/flutter/issues/145219.
      // TODO(matanlurey): Update the documentation to reflect the new behavior.
      if (isUntriaged) {
        _stderr
          ..writeln('NOTE: Untriaged image detected in tryjob.')
          ..writeln('Triage should be required by the "Flutter Gold" check')
          ..writeln('stdout:\n$resultStdout');
      }
    }
  }

  List<String> _getMatchingArguments(
    String testName,
    int screenshotSize,
    int pixelDeltaThreshold,
    double differentPixelsRate,
  ) {
    // The algorithm to be used when matching images. The available options are:
    // - "fuzzy": Allows for customizing the thresholds of pixel differences.
    // - "sobel": Same as "fuzzy" but performs edge detection before performing
    //            a fuzzy match.
    const String algorithm = 'fuzzy';

    // The number of pixels in this image that are allowed to differ from the
    // baseline. It's okay for this to be a slightly high number like 10% of the
    // image size because those wrong pixels are constrained by
    // `pixelDeltaThreshold` below.
    final int maxDifferentPixels = (screenshotSize * differentPixelsRate).toInt();
    return <String>[
      '--add-test-optional-key',
      'image_matching_algorithm:$algorithm',
      '--add-test-optional-key',
      'fuzzy_max_different_pixels:$maxDifferentPixels',
      '--add-test-optional-key',
      'fuzzy_pixel_delta_threshold:$pixelDeltaThreshold',
    ];
  }

  /// Returns the latest positive digest for the given test known to Skia Gold
  /// at head.
  Future<String?> getExpectationForTest(String testName) async {
    late String? expectation;
    final String traceID = getTraceID(testName);
    final Uri requestForExpectations = Uri.parse(
      '$_skiaGoldHost/json/v2/latestpositivedigest/$traceID',
    );
    late String rawResponse;
    try {
      final io.HttpClientRequest request = await httpClient.getUrl(requestForExpectations);
      final io.HttpClientResponse response = await request.close();
      rawResponse = await utf8.decodeStream(response);
      final dynamic jsonResponse = json.decode(rawResponse);
      if (jsonResponse is! Map<String, dynamic>) {
        throw const FormatException('Skia gold expectations do not match expected format.');
      }
      expectation = jsonResponse['digest'] as String?;
    } on FormatException catch (error) {
      _stderr.writeln(
        'Formatting error detected requesting expectations from Flutter Gold.\n'
        'error: $error\n'
        'url: $requestForExpectations\n'
        'response: $rawResponse',
      );
      rethrow;
    }
    return expectation;
  }

  /// Returns the current commit hash of the engine repository.
  Future<String> _getCurrentCommit() async {
    final String engineCheckout = _engineRoot.flutterDir.path;
    final io.ProcessResult revParse = await process.run(<String>[
      'git',
      'rev-parse',
      'HEAD',
    ], workingDirectory: engineCheckout);
    if (revParse.exitCode != 0) {
      throw StateError('Current commit of the engine can not be found from path $engineCheckout.');
    }
    return (revParse.stdout as String).trim();
  }

  /// Returns a Map of key value pairs used to uniquely identify the
  /// configuration that generated the given golden file.
  ///
  /// Currently, the only key value pairs being tracked are the platform and
  /// browser the image was rendered on.
  Map<String, dynamic> _getKeys() {
    final Map<String, dynamic> initialKeys = <String, dynamic>{
      'CI': 'luci',
      'Platform': io.Platform.operatingSystem,
    };
    if (dimensions != null) {
      initialKeys.addAll(dimensions!);
    }
    return initialKeys;
  }

  /// Same as [_getKeys] but encodes it in a JSON string.
  String _getKeysJSON() {
    return json.encode(_getKeys());
  }

  /// Returns a list of arguments for initializing a tryjob based on the testing
  /// environment.
  List<String> _getCIArguments() {
    final String jobId = _environment['LOGDOG_STREAM_PREFIX']!.split('/').last;
    final List<String> refs = _environment['GOLD_TRYJOB']!.split('/');
    final String pullRequest = refs[refs.length - 2];

    return <String>['--changelist', pullRequest, '--cis', 'buildbucket', '--jobid', jobId];
  }

  /// Returns a trace id based on the current testing environment to lookup
  /// the latest positive digest on Skia Gold with a hex-encoded md5 hash of
  /// the image keys.
  @visibleForTesting
  String getTraceID(String testName) {
    final Map<String, dynamic> keys = <String, dynamic>{
      ..._getKeys(),
      'name': testName,
      'source_type': _instance,
    };
    final String jsonTrace = json.encode(keys);
    final String md5Sum = md5.convert(utf8.encode(jsonTrace)).toString();
    return md5Sum;
  }
}
