// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'flutter_goldens.dart';
library;

import 'dart:convert';
import 'dart:io' as io;

import 'package:crypto/crypto.dart';
import 'package:file/file.dart';
import 'package:path/path.dart' as path;
import 'package:platform/platform.dart';
import 'package:process/process.dart';

// If you are here trying to figure out how to use golden files in the Flutter
// repo itself, consider reading this wiki page:
// https://github.com/flutter/flutter/blob/main/docs/contributing/testing/Writing-a-golden-file-test-for-package-flutter.md

const String _kFlutterRootKey = 'FLUTTER_ROOT';
const String _kGoldctlKey = 'GOLDCTL';
const String _kTestBrowserKey = 'FLUTTER_TEST_BROWSER';
const String _kWebRendererKey = 'FLUTTER_WEB_RENDERER';
const String _kImpellerKey = 'FLUTTER_TEST_IMPELLER';

/// Signature of callbacks used to inject [print] replacements.
typedef LogCallback = void Function(String);

/// Exception thrown when an error is returned from the [SkiaGoldClient].
class SkiaException implements Exception {
  /// Creates a new `SkiaException` with a required error [message].
  const SkiaException(this.message);

  /// A message describing the error.
  final String message;

  /// Returns a description of the Skia exception.
  ///
  /// The description always contains the [message].
  @override
  String toString() => 'SkiaException: $message';
}

/// A client for uploading image tests and making baseline requests to the
/// Flutter Gold Dashboard.
class SkiaGoldClient {
  /// Creates a [SkiaGoldClient] with the given [workDirectory] and [Platform].
  ///
  /// All other parameters are optional. They may be provided in tests to
  /// override the defaults for [fs], [process], and [httpClient].
  SkiaGoldClient(
    this.workDirectory, {
    required this.fs,
    required this.process,
    required this.platform,
    required this.httpClient,
    required this.log,
  });

  /// The file system to use for storing the local clone of the repository.
  ///
  /// This is useful in tests, where a local file system (the default) can be
  /// replaced by a memory file system.
  final FileSystem fs;

  /// The environment (current working directory, identity of the OS,
  /// environment variables, etc).
  final Platform platform;

  /// A controller for launching sub-processes.
  ///
  /// This is useful in tests, where the real process manager (the default) can
  /// be replaced by a mock process manager that doesn't really create
  /// sub-processes.
  final ProcessManager process;

  /// A client for making Http requests to the Flutter Gold dashboard.
  final io.HttpClient httpClient;

  /// The local [Directory] within the comparison root for the current test
  /// context. In this directory, the client will create image and JSON files
  /// for the goldctl tool to use.
  ///
  /// This is informed by [FlutterGoldenFileComparator.basedir]. It cannot be
  /// null.
  final Directory workDirectory;

  /// The logging function to use when reporting messages to the console.
  final LogCallback log;

  /// The local [Directory] where the Flutter repository is hosted.
  ///
  /// Uses the [fs] file system.
  Directory get _flutterRoot => fs.directory(platform.environment[_kFlutterRootKey]);

  /// The path to the local [Directory] where the goldctl tool is hosted.
  ///
  /// Uses the [platform] environment in this implementation.
  String get _goldctl => platform.environment[_kGoldctlKey]!;

  /// Prepares the local work space for golden file testing and calls the
  /// goldctl `auth` command.
  ///
  /// This ensures that the goldctl tool is authorized and ready for testing.
  /// Used by the [FlutterPostSubmitFileComparator] and the
  /// [FlutterPreSubmitFileComparator].
  Future<void> auth() async {
    if (await clientIsAuthorized()) {
      return;
    }
    final List<String> authCommand = <String>[
      _goldctl,
      'auth',
      '--work-dir',
      workDirectory.childDirectory('temp').path,
      '--luci',
    ];

    final io.ProcessResult result = await process.run(authCommand);

    if (result.exitCode != 0) {
      final StringBuffer buf =
          StringBuffer()
            ..writeln('Skia Gold authorization failed.')
            ..writeln(
              'Luci environments authenticate using the file provided '
              'by LUCI_CONTEXT. There may be an error with this file or Gold '
              'authentication.',
            )
            ..writeln('Debug information for Gold --------------------------------')
            ..writeln('stdout: ${result.stdout}')
            ..writeln('stderr: ${result.stderr}');
      throw SkiaException(buf.toString());
    }
  }

  /// Signals if this client is initialized for uploading images to the Gold
  /// service.
  ///
  /// Since Flutter framework tests are executed in parallel, and in random
  /// order, this will signal is this instance of the Gold client has been
  /// initialized.
  bool _initialized = false;

  /// Executes the `imgtest init` command in the goldctl tool.
  ///
  /// The `imgtest` command collects and uploads test results to the Skia Gold
  /// backend, the `init` argument initializes the current test. Used by the
  /// [FlutterPostSubmitFileComparator].
  Future<void> imgtestInit() async {
    // This client has already been initialized
    if (_initialized) {
      return;
    }

    final File keys = workDirectory.childFile('keys.json');
    final File failures = workDirectory.childFile('failures.json');

    await keys.writeAsString(_getKeysJSON());
    await failures.create();
    final String commitHash = await _getCurrentCommit();

    final List<String> imgtestInitCommand = <String>[
      _goldctl,
      'imgtest',
      'init',
      '--instance',
      'flutter',
      '--work-dir',
      workDirectory.childDirectory('temp').path,
      '--commit',
      commitHash,
      '--keys-file',
      keys.path,
      '--failure-file',
      failures.path,
      '--passfail',
    ];

    if (imgtestInitCommand.contains(null)) {
      final StringBuffer buf =
          StringBuffer()
            ..writeln('A null argument was provided for Skia Gold imgtest init.')
            ..writeln('Please confirm the settings of your golden file test.')
            ..writeln('Arguments provided:');
      imgtestInitCommand.forEach(buf.writeln);
      throw SkiaException(buf.toString());
    }

    final io.ProcessResult result = await process.run(imgtestInitCommand);

    if (result.exitCode != 0) {
      _initialized = false;
      final StringBuffer buf =
          StringBuffer()
            ..writeln('Skia Gold imgtest init failed.')
            ..writeln('An error occurred when initializing golden file test with ')
            ..writeln('goldctl.')
            ..writeln()
            ..writeln('Debug information for Gold --------------------------------')
            ..writeln('stdout: ${result.stdout}')
            ..writeln('stderr: ${result.stderr}');
      throw SkiaException(buf.toString());
    }
    _initialized = true;
  }

  /// Executes the `imgtest add` command in the goldctl tool.
  ///
  /// The `imgtest` command collects and uploads test results to the Skia Gold
  /// backend, the `add` argument uploads the current image test. A response is
  /// returned from the invocation of this command that indicates a pass or fail
  /// result.
  ///
  /// The [testName] and [goldenFile] parameters reference the current
  /// comparison being evaluated by the [FlutterPostSubmitFileComparator].
  Future<bool> imgtestAdd(String testName, File goldenFile) async {
    final List<String> imgtestCommand = <String>[
      _goldctl,
      'imgtest',
      'add',
      '--work-dir',
      workDirectory.childDirectory('temp').path,
      '--test-name',
      cleanTestName(testName),
      '--png-file',
      goldenFile.path,
      '--passfail',
      ..._getPixelMatchingArguments(),
    ];

    final io.ProcessResult result = await process.run(imgtestCommand);

    if (result.exitCode != 0) {
      // If an unapproved image has made it to post-submit, throw to close the
      // tree.
      String? resultContents;
      final File resultFile = workDirectory.childFile(fs.path.join('result-state.json'));
      if (await resultFile.exists()) {
        resultContents = await resultFile.readAsString();
      }

      final StringBuffer buf =
          StringBuffer()
            ..writeln('Skia Gold received an unapproved image in post-submit ')
            ..writeln('testing. Golden file images in flutter/flutter are triaged ')
            ..writeln('in pre-submit during code review for the given PR.')
            ..writeln()
            ..writeln('Visit https://flutter-gold.skia.org/ to view and approve ')
            ..writeln('the image(s), or revert the associated change. For more ')
            ..writeln('information, visit the wiki: ')
            ..writeln(
              'https://github.com/flutter/flutter/blob/main/docs/contributing/testing/Writing-a-golden-file-test-for-package-flutter.md',
            )
            ..writeln()
            ..writeln('Debug information for Gold --------------------------------')
            ..writeln('stdout: ${result.stdout}')
            ..writeln('stderr: ${result.stderr}')
            ..writeln()
            ..writeln('result-state.json: ${resultContents ?? 'No result file found.'}');
      throw SkiaException(buf.toString());
    }

    return true;
  }

  /// Signals if this client is initialized for uploading tryjobs to the Gold
  /// service.
  ///
  /// Since Flutter framework tests are executed in parallel, and in random
  /// order, this will signal is this instance of the Gold client has been
  /// initialized for tryjobs.
  bool _tryjobInitialized = false;

  /// Executes the `imgtest init` command in the goldctl tool for tryjobs.
  ///
  /// The `imgtest` command collects and uploads test results to the Skia Gold
  /// backend, the `init` argument initializes the current tryjob. Used by the
  /// [FlutterPreSubmitFileComparator].
  Future<void> tryjobInit() async {
    // This client has already been initialized
    if (_tryjobInitialized) {
      return;
    }

    final File keys = workDirectory.childFile('keys.json');
    final File failures = workDirectory.childFile('failures.json');

    await keys.writeAsString(_getKeysJSON());
    await failures.create();
    final String commitHash = await _getCurrentCommit();

    final List<String> imgtestInitCommand = <String>[
      _goldctl,
      'imgtest',
      'init',
      '--instance',
      'flutter',
      '--work-dir',
      workDirectory.childDirectory('temp').path,
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
      ...getCIArguments(),
    ];

    if (imgtestInitCommand.contains(null)) {
      final StringBuffer buf =
          StringBuffer()
            ..writeln('A null argument was provided for Skia Gold tryjob init.')
            ..writeln('Please confirm the settings of your golden file test.')
            ..writeln('Arguments provided:');
      imgtestInitCommand.forEach(buf.writeln);
      throw SkiaException(buf.toString());
    }

    final io.ProcessResult result = await process.run(imgtestInitCommand);

    if (result.exitCode != 0) {
      _tryjobInitialized = false;
      final StringBuffer buf =
          StringBuffer()
            ..writeln('Skia Gold tryjobInit failure.')
            ..writeln('An error occurred when initializing golden file tryjob with ')
            ..writeln('goldctl.')
            ..writeln()
            ..writeln('Debug information for Gold --------------------------------')
            ..writeln('stdout: ${result.stdout}')
            ..writeln('stderr: ${result.stderr}');
      throw SkiaException(buf.toString());
    }
    _tryjobInitialized = true;
  }

  /// Executes the `imgtest add` command in the goldctl tool for tryjobs.
  ///
  /// The `imgtest` command collects and uploads test results to the Skia Gold
  /// backend, the `add` argument uploads the current image test. A response is
  /// returned from the invocation of this command that indicates a pass or fail
  /// result for the tryjob.
  ///
  /// The [testName] and [goldenFile] parameters reference the current
  /// comparison being evaluated by the [FlutterPreSubmitFileComparator].
  ///
  /// If the tryjob fails due to pixel differences, the method will succeed
  /// as the failure will be triaged in the 'Flutter Gold' dashboard, and the
  /// `stdout` will contain the failure message; otherwise will return `null`.
  Future<String?> tryjobAdd(String testName, File goldenFile) async {
    final List<String> imgtestCommand = <String>[
      _goldctl,
      'imgtest',
      'add',
      '--work-dir',
      workDirectory.childDirectory('temp').path,
      '--test-name',
      cleanTestName(testName),
      '--png-file',
      goldenFile.path,
      ..._getPixelMatchingArguments(),
    ];

    final io.ProcessResult result = await process.run(imgtestCommand);

    final String resultStdout = result.stdout.toString();
    if (result.exitCode != 0 &&
        !(resultStdout.contains('Untriaged') || resultStdout.contains('negative image'))) {
      String? resultContents;
      final File resultFile = workDirectory.childFile(fs.path.join('result-state.json'));
      if (await resultFile.exists()) {
        resultContents = await resultFile.readAsString();
      }
      final StringBuffer buf =
          StringBuffer()
            ..writeln('Unexpected Gold tryjobAdd failure.')
            ..writeln('Tryjob execution for golden file test $testName failed for')
            ..writeln('a reason unrelated to pixel comparison.')
            ..writeln()
            ..writeln('Debug information for Gold --------------------------------')
            ..writeln('stdout: ${result.stdout}')
            ..writeln('stderr: ${result.stderr}')
            ..writeln()
            ..writeln()
            ..writeln('result-state.json: ${resultContents ?? 'No result file found.'}');
      throw SkiaException(buf.toString());
    }
    return result.exitCode == 0 ? null : resultStdout;
  }

  // Constructs arguments for `goldctl` for controlling how pixels are compared.
  //
  // For AOT and CanvasKit exact pixel matching is used. For the HTML renderer
  // on the web a fuzzy matching algorithm is used that allows very small deltas
  // because Chromium cannot exactly reproduce the same golden on all computers.
  // It seems to depend on the hardware/OS/driver combination. However, those
  // differences are very small (typically not noticeable to human eye).
  List<String> _getPixelMatchingArguments() {
    // Only use fuzzy pixel matching in the HTML renderer.
    if (!_isBrowserTest || _isBrowserSkiaTest) {
      return const <String>[];
    }

    // The algorithm to be used when matching images. The available options are:
    // - "fuzzy": Allows for customizing the thresholds of pixel differences.
    // - "sobel": Same as "fuzzy" but performs edge detection before performing
    //            a fuzzy match.
    const String algorithm = 'fuzzy';

    // The number of pixels in this image that are allowed to differ from the
    // baseline.
    //
    // The chosen number - 20 - is arbitrary. Even for a small golden file, say
    // 50 x 50, it would be less than 1% of the total number of pixels. This
    // number should not grow too much. If it's growing, it is probably due to a
    // larger issue that needs to be addressed at the infra level.
    const int maxDifferentPixels = 20;

    // The maximum acceptable difference per pixel.
    //
    // Uses the Manhattan distance using the RGBA color components as
    // coordinates. The chosen number - 4 - is arbitrary. It's small enough to
    // both not be noticeable and not trigger test flakes due to sub-pixel
    // golden deltas. This number should not grow too much. If it's growing, it
    // is probably due to a larger issue that needs to be addressed at the infra
    // level.
    const int pixelDeltaThreshold = 4;

    return <String>[
      '--add-test-optional-key',
      'image_matching_algorithm:$algorithm',
      '--add-test-optional-key',
      'fuzzy_max_different_pixels:$maxDifferentPixels',
      '--add-test-optional-key',
      'fuzzy_pixel_delta_threshold:$pixelDeltaThreshold',
    ];
  }

  /// Returns the latest positive digest for the given test known to Flutter
  /// Gold at head.
  Future<String?> getExpectationForTest(String testName) async {
    late String? expectation;
    final String traceID = getTraceID(testName);
    final Uri requestForExpectations = Uri.parse(
      'https://flutter-gold.skia.org/json/v2/latestpositivedigest/$traceID',
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
      log(
        'Formatting error detected requesting expectations from Flutter Gold.\n'
        'error: $error\n'
        'url: $requestForExpectations\n'
        'response: $rawResponse',
      );
      rethrow;
    }
    return expectation;
  }

  /// Returns a list of bytes representing the golden image retrieved from the
  /// Flutter Gold dashboard.
  ///
  /// The provided image hash represents an expectation from Flutter Gold.
  Future<List<int>> getImageBytes(String imageHash) async {
    final List<int> imageBytes = <int>[];
    final Uri requestForImage = Uri.parse(
      'https://flutter-gold.skia.org/img/images/$imageHash.png',
    );
    final io.HttpClientRequest request = await httpClient.getUrl(requestForImage);
    final io.HttpClientResponse response = await request.close();
    await response.forEach((List<int> bytes) => imageBytes.addAll(bytes));
    return imageBytes;
  }

  /// Returns the current commit hash of the Flutter repository.
  Future<String> _getCurrentCommit() async {
    if (!_flutterRoot.existsSync()) {
      throw SkiaException('Flutter root could not be found: $_flutterRoot\n');
    } else {
      final io.ProcessResult revParse = await process.run(<String>[
        'git',
        'rev-parse',
        'HEAD',
      ], workingDirectory: _flutterRoot.path);
      if (revParse.exitCode != 0) {
        throw const SkiaException('Current commit of Flutter can not be found.');
      }
      return (revParse.stdout as String).trim();
    }
  }

  /// Returns a JSON String with keys value pairs used to uniquely identify the
  /// configuration that generated the given golden file.
  ///
  /// Currently, the only key value pairs being tracked is the platform the
  /// image was rendered on, and for web tests, the browser the image was
  /// rendered on.
  String _getKeysJSON() {
    final String? webRenderer = _webRendererValue;
    final Map<String, dynamic> keys = <String, dynamic>{
      'Platform': platform.operatingSystem,
      'CI': 'luci',
      if (_isImpeller) 'impeller': 'swiftshader',
    };
    if (_isBrowserTest) {
      keys['Browser'] = _browserKey;
      keys['Platform'] = '${keys['Platform']}-browser';
      if (webRenderer != null) {
        keys['WebRenderer'] = webRenderer;
      }
    }
    return json.encode(keys);
  }

  /// Removes the file extension from the [fileName] to represent the test name
  /// properly.
  String cleanTestName(String fileName) {
    return fileName.split(path.extension(fileName))[0];
  }

  /// Returns a boolean value to prevent the client from re-authorizing itself
  /// for multiple tests.
  Future<bool> clientIsAuthorized() async {
    final File authFile = workDirectory.childFile(fs.path.join('temp', 'auth_opt.json'));

    if (await authFile.exists()) {
      final String contents = await authFile.readAsString();
      final Map<String, dynamic> decoded = json.decode(contents) as Map<String, dynamic>;
      return !(decoded['GSUtil'] as bool);
    }
    return false;
  }

  /// Returns a list of arguments for initializing a tryjob based on the testing
  /// environment.
  List<String> getCIArguments() {
    final String jobId = platform.environment['LOGDOG_STREAM_PREFIX']!.split('/').last;
    final List<String> refs = platform.environment['GOLD_TRYJOB']!.split('/');
    final String pullRequest = refs[refs.length - 2];

    return <String>['--changelist', pullRequest, '--cis', 'buildbucket', '--jobid', jobId];
  }

  bool get _isBrowserTest {
    return platform.environment[_kTestBrowserKey] != null;
  }

  bool get _isBrowserSkiaTest {
    return _isBrowserTest &&
        switch (platform.environment[_kWebRendererKey]) {
          'canvaskit' || 'skwasm' => true,
          _ => false,
        };
  }

  String? get _webRendererValue {
    return _isBrowserSkiaTest ? platform.environment[_kWebRendererKey] : null;
  }

  bool get _isImpeller {
    return (platform.environment[_kImpellerKey] != null);
  }

  String get _browserKey {
    assert(_isBrowserTest);
    return platform.environment[_kTestBrowserKey]!;
  }

  /// Returns a trace id based on the current testing environment to lookup
  /// the latest positive digest on Flutter Gold with a hex-encoded md5 hash of
  /// the image keys.
  String getTraceID(String testName) {
    final String? webRenderer = _webRendererValue;
    final Map<String, Object?> parameters = <String, Object?>{
      if (_isBrowserTest) 'Browser': _browserKey,
      'CI': 'luci',
      'Platform': platform.operatingSystem,
      if (webRenderer != null) 'WebRenderer': webRenderer,
      if (_isImpeller) 'impeller': 'swiftshader',
      'name': testName,
      'source_type': 'flutter',
    };
    final Map<String, Object?> sorted = <String, Object?>{};
    for (final String key in parameters.keys.toList()..sort()) {
      sorted[key] = parameters[key];
    }
    final String jsonTrace = json.encode(sorted);
    final String md5Sum = md5.convert(utf8.encode(jsonTrace)).toString();
    return md5Sum;
  }
}
