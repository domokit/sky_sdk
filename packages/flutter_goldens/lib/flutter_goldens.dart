// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'dart:io';
library;

import 'dart:async' show FutureOr;
import 'dart:io' as io show HttpClient, OSError, SocketException;

import 'package:file/file.dart';
import 'package:file/local.dart';
import 'package:flutter/foundation.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:platform/platform.dart';
import 'package:process/process.dart';

import 'skia_client.dart';
export 'skia_client.dart';

// If you are here trying to figure out how to use golden files in the Flutter
// repo itself, consider reading this wiki page:
// https://github.com/flutter/flutter/blob/main/docs/contributing/testing/Writing-a-golden-file-test-for-package-flutter.md

// If you are trying to debug this package, you may like to use the golden test
// titled "Inconsequential golden test" in this file:
//   /packages/flutter/test/widgets/basic_test.dart

// TODO(ianh): sort the parameters and arguments in this file so they use a consistent order throughout.

const String _kFlutterRootKey = 'FLUTTER_ROOT';

bool _isMainBranch(String? branch) {
  return branch == 'main' || branch == 'master';
}

/// Main method that can be used in a `flutter_test_config.dart` file to set
/// [goldenFileComparator] to an instance of [FlutterGoldenFileComparator] that
/// works for the current test. _Which_ [FlutterGoldenFileComparator] is
/// instantiated is based on the current testing environment.
///
/// When set, the `namePrefix` is prepended to the names of all gold images.
///
/// This function assumes the [goldenFileComparator] has been set to a
/// [LocalFileComparator], which happens in the bootstrap code used when running
/// tests using `flutter test`. This should not be called when running a test
/// using `flutter run`, as in that environment, the [goldenFileComparator] is a
/// [TrivialComparator].
///
/// An [HttpClient] is created when this method is called. That client is used
/// to communicate with the Skia Gold servers. Any [HttpOverrides] set in this
/// will affect whether this is effective or not. For example, if the current
/// override provides a mock client that always fails, then all calls to gold
/// comparison functions will fail.
Future<void> testExecutable(FutureOr<void> Function() testMain, {String? namePrefix}) async {
  assert(
    goldenFileComparator is LocalFileComparator,
    'The flutter_goldens package should be used from a flutter_test_config.dart '
    'file, which is only invoked when using "flutter test". The "flutter test" '
    'bootstrap logic sets "goldenFileComparator" to a LocalFileComparator. It '
    'appears in this instance however that the "goldenFileComparator" is a '
    '${goldenFileComparator.runtimeType}.\n'
    'See also: https://flutter.dev/to/flutter-test-docs',
  );
  const Platform platform = LocalPlatform();
  const FileSystem fs = LocalFileSystem();
  const ProcessManager process = LocalProcessManager();
  final io.HttpClient httpClient = io.HttpClient();
  if (FlutterPostSubmitFileComparator.isForEnvironment(platform)) {
    goldenFileComparator = await FlutterPostSubmitFileComparator.fromLocalFileComparator(
      localFileComparator: goldenFileComparator as LocalFileComparator,
      platform: platform,
      namePrefix: namePrefix,
      log: print,
      fs: fs,
      process: process,
      httpClient: httpClient,
    );
  } else if (FlutterPreSubmitFileComparator.isForEnvironment(platform)) {
    goldenFileComparator = await FlutterPreSubmitFileComparator.fromLocalFileComparator(
      localFileComparator: goldenFileComparator as LocalFileComparator,
      platform: platform,
      namePrefix: namePrefix,
      log: print,
      fs: fs,
      process: process,
      httpClient: httpClient,
    );
  } else if (FlutterSkippingFileComparator.isForEnvironment(platform)) {
    goldenFileComparator = FlutterSkippingFileComparator.fromLocalFileComparator(
      localFileComparator: goldenFileComparator as LocalFileComparator,
      'Golden file testing is not executed on Cirrus, or LUCI environments '
      'outside of flutter/flutter, or in test shards that are not configured '
      'for using goldctl.',
      platform: platform,
      namePrefix: namePrefix,
      log: print,
      fs: fs,
      process: process,
      httpClient: httpClient,
    );
  } else {
    goldenFileComparator = await FlutterLocalFileComparator.fromLocalFileComparator(
      localFileComparator: goldenFileComparator as LocalFileComparator,
      platform: platform,
      log: print,
      fs: fs,
      process: process,
      httpClient: httpClient,
    );
  }
  await testMain();
}

/// Abstract base class golden file comparator specific to the `flutter/flutter`
/// repository.
///
/// Golden file testing for the `flutter/flutter` repository is handled by three
/// different [FlutterGoldenFileComparator]s, depending on the current testing
/// environment.
///
///   * The [FlutterPostSubmitFileComparator] is utilized during post-submit
///     testing, after a pull request has landed on the master branch. This
///     comparator uses the [SkiaGoldClient] and the `goldctl` tool to upload
///     tests to the [Flutter Gold dashboard](https://flutter-gold.skia.org).
///     Flutter Gold manages the master golden files for the `flutter/flutter`
///     repository.
///
///   * The [FlutterPreSubmitFileComparator] is utilized in pre-submit testing,
///     before a pull request lands on the master branch. This
///     comparator uses the [SkiaGoldClient] to execute tryjobs, allowing
///     contributors to view and check in visual differences before landing the
///     change.
///
///   * The [FlutterLocalFileComparator] is used for local development testing.
///     This comparator will use the [SkiaGoldClient] to request baseline images
///     from [Flutter Gold](https://flutter-gold.skia.org) and manually compare
///     pixels. If a difference is detected, this comparator will
///     generate failure output illustrating the found difference. If a baseline
///     is not found for a given test image, it will consider it a new test and
///     output the new image for verification.
///
///  The [FlutterSkippingFileComparator] is utilized to skip tests outside
///  of the appropriate environments described above. Currently, some Luci
///  environments do not execute golden file testing, and as such do not require
///  a comparator. This comparator is also used when an internet connection is
///  unavailable.
abstract class FlutterGoldenFileComparator extends GoldenFileComparator {
  /// Creates a [FlutterGoldenFileComparator] that will resolve golden file
  /// URIs relative to the specified [basedir], and retrieve golden baselines
  /// using the [skiaClient]. The [basedir] is used for writing and accessing
  /// information and files for interacting with the [skiaClient]. When testing
  /// locally, the [basedir] will also contain any diffs from failed tests, or
  /// goldens generated from newly introduced tests.
  @visibleForTesting
  FlutterGoldenFileComparator(
    this.basedir,
    this.skiaClient, {
    required this.fs,
    required this.platform,
    this.namePrefix,
    required this.log,
  });

  /// The directory to which golden file URIs will be resolved in [compare] and
  /// [update].
  final Uri basedir;

  /// A client for uploading image tests and making baseline requests to the
  /// Flutter Gold Dashboard.
  final SkiaGoldClient skiaClient;

  /// The file system used to perform file access.
  final FileSystem fs;

  /// The environment (current working directory, identity of the OS,
  /// environment variables, etc).
  final Platform platform;

  /// The prefix that is added to all golden names.
  final String? namePrefix;

  /// The logging function to use when reporting messages to the console.
  final LogCallback log;

  @override
  Future<void> update(Uri golden, Uint8List imageBytes) async {
    final File goldenFile = getGoldenFile(golden);
    await goldenFile.parent.create(recursive: true);
    await goldenFile.writeAsBytes(imageBytes, flush: true);
  }

  @override
  Uri getTestUri(Uri key, int? version) => key;

  /// Calculate the appropriate basedir for the current test context.
  ///
  /// The optional [suffix] argument is used by the
  /// [FlutterPostSubmitFileComparator] and the [FlutterPreSubmitFileComparator].
  /// These [FlutterGoldenFileComparator]s randomize their base directories to
  /// maintain thread safety while using the `goldctl` tool.
  @protected
  @visibleForTesting
  static Directory getBaseDirectory(
    LocalFileComparator defaultComparator, {
    required Platform platform,
    String? suffix,
    required FileSystem fs,
  }) {
    final Directory flutterRoot = fs.directory(platform.environment[_kFlutterRootKey]);
    final Directory comparisonRoot = switch (suffix) {
      null => flutterRoot.childDirectory(fs.path.join('bin', 'cache', 'pkg', 'skia_goldens')),
      _ => fs.systemTempDirectory.createTempSync(suffix),
    };

    final String testPath = fs.directory(defaultComparator.basedir).path;
    return comparisonRoot.childDirectory(fs.path.relative(testPath, from: flutterRoot.path));
  }

  /// Returns the golden [File] identified by the given [Uri].
  @protected
  File getGoldenFile(Uri uri) {
    final File goldenFile = fs.directory(basedir).childFile(fs.file(uri).path);
    return goldenFile;
  }

  /// Prepends the golden URL with the library name that encloses the current
  /// test.
  Uri _addPrefix(Uri golden) {
    // Ensure the Uri ends in .png as the SkiaClient expects
    assert(
      golden.toString().split('.').last == 'png',
      'Golden files in the Flutter framework must end with the file extension '
      '.png.',
    );
    return Uri.parse(
      <String>[
        if (namePrefix != null) namePrefix!,
        basedir.pathSegments[basedir.pathSegments.length - 2],
        golden.toString(),
      ].join('.'),
    );
  }
}

/// A [FlutterGoldenFileComparator] for testing golden images with Skia Gold in
/// post-submit.
///
/// For testing across all platforms, the [SkiaGoldClient] is used to upload
/// images for framework-related golden tests and process results.
///
/// See also:
///
///  * [GoldenFileComparator], the abstract class that
///    [FlutterGoldenFileComparator] implements.
///  * [FlutterPreSubmitFileComparator], another
///    [FlutterGoldenFileComparator] that tests golden images before changes are
///    merged into the master branch.
///  * [FlutterLocalFileComparator], another
///    [FlutterGoldenFileComparator] that tests golden images locally on your
///    current machine.
class FlutterPostSubmitFileComparator extends FlutterGoldenFileComparator {
  /// Creates a [FlutterPostSubmitFileComparator] that will test golden file
  /// images against Skia Gold.
  ///
  /// The [fs] parameter is useful in tests, where the default
  /// file system can be replaced by mock instances.
  FlutterPostSubmitFileComparator(
    super.basedir,
    super.skiaClient, {
    required super.fs,
    required super.platform,
    super.namePrefix,
    required super.log,
  });

  /// Creates a new [FlutterPostSubmitFileComparator] that mirrors the relative
  /// path resolution of the provided `localFileComparator`.
  ///
  /// The [goldens] parameter is visible for testing purposes only.
  static Future<FlutterPostSubmitFileComparator> fromLocalFileComparator({
    SkiaGoldClient? goldens,
    required LocalFileComparator localFileComparator,
    required Platform platform,
    String? namePrefix,
    required LogCallback log,
    required FileSystem fs,
    required ProcessManager process,
    required io.HttpClient httpClient,
  }) async {
    final Directory baseDirectory = FlutterGoldenFileComparator.getBaseDirectory(
      localFileComparator,
      platform: platform,
      suffix: 'flutter_goldens_postsubmit.',
      fs: fs,
    );
    baseDirectory.createSync(recursive: true);

    goldens ??= SkiaGoldClient(
      baseDirectory,
      log: log,
      platform: platform,
      fs: fs,
      process: process,
      httpClient: httpClient,
    );
    await goldens.auth();
    return FlutterPostSubmitFileComparator(
      baseDirectory.uri,
      goldens,
      platform: platform,
      namePrefix: namePrefix,
      log: log,
      fs: fs,
    );
  }

  @override
  Future<bool> compare(Uint8List imageBytes, Uri golden) async {
    await skiaClient.imgtestInit();
    golden = _addPrefix(golden);
    await update(golden, imageBytes);
    final File goldenFile = getGoldenFile(golden);
    return skiaClient.imgtestAdd(golden.path, goldenFile);
  }

  /// Decides based on the current environment if goldens tests should be
  /// executed through Skia Gold.
  static bool isForEnvironment(Platform platform) {
    final bool luciPostSubmit =
        platform.environment.containsKey('SWARMING_TASK_ID') &&
        platform.environment.containsKey('GOLDCTL')
        // Luci tryjob environments contain this value to inform the [FlutterPreSubmitComparator].
        &&
        !platform.environment.containsKey('GOLD_TRYJOB')
        // Only run on main branch.
        &&
        _isMainBranch(platform.environment['GIT_BRANCH']);
    return luciPostSubmit;
  }
}

/// A [FlutterGoldenFileComparator] for testing golden images before changes are
/// merged into the master branch. The comparator executes tryjobs using the
/// [SkiaGoldClient].
///
/// See also:
///
///  * [GoldenFileComparator], the abstract class that
///    [FlutterGoldenFileComparator] implements.
///  * [FlutterPostSubmitFileComparator], another
///    [FlutterGoldenFileComparator] that uploads tests to the Skia Gold
///    dashboard in post-submit.
///  * [FlutterLocalFileComparator], another
///    [FlutterGoldenFileComparator] that tests golden images locally on your
///    current machine.
class FlutterPreSubmitFileComparator extends FlutterGoldenFileComparator {
  /// Creates a [FlutterPreSubmitFileComparator] that will test golden file
  /// images against baselines requested from Flutter Gold.
  ///
  /// The [fs] parameter is useful in tests, where the default
  /// file system can be replaced by mock instances.
  FlutterPreSubmitFileComparator(
    super.basedir,
    super.skiaClient, {
    required super.fs,
    required super.platform,
    super.namePrefix,
    required super.log,
  });

  /// Creates a new [FlutterPreSubmitFileComparator] that mirrors the
  /// relative path resolution of the default [goldenFileComparator].
  ///
  /// The [goldens] parameter is visible for testing purposes only.
  static Future<FlutterGoldenFileComparator> fromLocalFileComparator({
    SkiaGoldClient? goldens,
    required LocalFileComparator localFileComparator,
    required Platform platform,
    Directory? testBasedir,
    String? namePrefix,
    required LogCallback log,
    required FileSystem fs,
    required ProcessManager process,
    required io.HttpClient httpClient,
  }) async {
    final Directory baseDirectory =
        testBasedir ??
        FlutterGoldenFileComparator.getBaseDirectory(
          localFileComparator,
          platform: platform,
          suffix: 'flutter_goldens_presubmit.',
          fs: fs,
        );

    if (!baseDirectory.existsSync()) {
      baseDirectory.createSync(recursive: true);
    }

    goldens ??= SkiaGoldClient(
      baseDirectory,
      platform: platform,
      log: log,
      fs: fs,
      process: process,
      httpClient: httpClient,
    );

    await goldens.auth();
    return FlutterPreSubmitFileComparator(
      baseDirectory.uri,
      goldens,
      platform: platform,
      namePrefix: namePrefix,
      log: log,
      fs: fs,
    );
  }

  @override
  Future<bool> compare(Uint8List imageBytes, Uri golden) async {
    await skiaClient.tryjobInit();
    golden = _addPrefix(golden);
    await update(golden, imageBytes);
    final File goldenFile = getGoldenFile(golden);

    await skiaClient.tryjobAdd(golden.path, goldenFile);

    // This will always return true since golden file test failures are managed
    // in pre-submit checks by the flutter-gold status check.
    return true;
  }

  /// Decides based on the current environment if goldens tests should be
  /// executed as pre-submit tests with Skia Gold.
  static bool isForEnvironment(Platform platform) {
    final bool luciPreSubmit =
        platform.environment.containsKey('SWARMING_TASK_ID') &&
        platform.environment.containsKey('GOLDCTL') &&
        platform.environment.containsKey('GOLD_TRYJOB')
        // Only run on the main branch
        &&
        _isMainBranch(platform.environment['GIT_BRANCH']);
    return luciPreSubmit;
  }
}

/// A [FlutterGoldenFileComparator] for testing conditions that do not execute
/// golden file tests.
///
/// Currently, this comparator is used on Cirrus, or in Luci environments when executing tests
/// outside of the flutter/flutter repository.
///
/// See also:
///
///  * [FlutterPostSubmitFileComparator], another [FlutterGoldenFileComparator]
///    that tests golden images through Skia Gold.
///  * [FlutterPreSubmitFileComparator], another
///    [FlutterGoldenFileComparator] that tests golden images before changes are
///    merged into the master branch.
///  * [FlutterLocalFileComparator], another
///    [FlutterGoldenFileComparator] that tests golden images locally on your
///    current machine.
class FlutterSkippingFileComparator extends FlutterGoldenFileComparator {
  /// Creates a [FlutterSkippingFileComparator] that will skip tests that
  /// are not in the right environment for golden file testing.
  FlutterSkippingFileComparator(
    super.basedir,
    super.skiaClient,
    this.reason, {
    super.namePrefix,
    required super.platform,
    required super.log,
    required super.fs,
  });

  /// Describes the reason for using the [FlutterSkippingFileComparator].
  final String reason;

  /// Creates a new [FlutterSkippingFileComparator] that mirrors the
  /// relative path resolution of the given [localFileComparator].
  static FlutterSkippingFileComparator fromLocalFileComparator(
    String reason, {
    required LocalFileComparator localFileComparator,
    String? namePrefix,
    required Platform platform,
    required LogCallback log,
    required FileSystem fs,
    required ProcessManager process,
    required io.HttpClient httpClient,
  }) {
    final Uri basedir = localFileComparator.basedir;
    final SkiaGoldClient skiaClient = SkiaGoldClient(
      fs.directory(basedir),
      platform: platform,
      log: log,
      fs: fs,
      process: process,
      httpClient: httpClient,
    );
    return FlutterSkippingFileComparator(
      basedir,
      skiaClient,
      reason,
      namePrefix: namePrefix,
      platform: platform,
      log: log,
      fs: fs,
    );
  }

  @override
  Future<bool> compare(Uint8List imageBytes, Uri golden) async {
    log('Skipping "$golden" test: $reason');
    return true;
  }

  @override
  Future<void> update(Uri golden, Uint8List imageBytes) async {}

  /// Decides, based on the current environment, if this comparator should be
  /// used.
  ///
  /// If we are in a CI environment, LUCI or Cirrus, but are not using the other
  /// comparators, we skip. Otherwise we would fallback to the local comparator,
  /// for which failures cannot be resolved in a CI environment.
  static bool isForEnvironment(Platform platform) {
    return platform.environment.containsKey('SWARMING_TASK_ID')
        // Some builds are still being run on Cirrus, we should skip these.
        ||
        platform.environment.containsKey('CIRRUS_CI');
  }
}

/// A [FlutterGoldenFileComparator] for testing golden images locally on your
/// current machine.
///
/// This comparator utilizes the [SkiaGoldClient] to request baseline images for
/// the given device under test for comparison. This comparator is initialized
/// when conditions for all other [FlutterGoldenFileComparator]s have not been
/// met, see the `isForEnvironment` method for each one listed below.
///
/// The [FlutterLocalFileComparator] is intended to run on local machines and
/// serve as a smoke test during development. As such, it will not be able to
/// detect unintended changes on environments other than the currently executing
/// machine, until they are tested using the [FlutterPreSubmitFileComparator].
///
/// See also:
///
///  * [GoldenFileComparator], the abstract class that
///    [FlutterGoldenFileComparator] implements.
///  * [FlutterPostSubmitFileComparator], another
///    [FlutterGoldenFileComparator] that uploads tests to the Skia Gold
///    dashboard.
///  * [FlutterPreSubmitFileComparator], another
///    [FlutterGoldenFileComparator] that tests golden images before changes are
///    merged into the master branch.
///  * [FlutterSkippingFileComparator], another
///    [FlutterGoldenFileComparator] that controls post-submit testing
///    conditions that do not execute golden file tests.
class FlutterLocalFileComparator extends FlutterGoldenFileComparator with LocalComparisonOutput {
  /// Creates a [FlutterLocalFileComparator] that will test golden file
  /// images against baselines requested from Flutter Gold.
  ///
  /// The [fs] parameter is useful in tests, where the default
  /// file system can be replaced by mock instances.
  FlutterLocalFileComparator(
    super.basedir,
    super.skiaClient, {
    required super.fs,
    required super.platform,
    required super.log,
  });

  /// Creates a new [FlutterLocalFileComparator] that mirrors the
  /// relative path resolution of the given [localFileComparator].
  ///
  /// The [goldens] and [baseDirectory] parameters are
  /// visible for testing purposes only.
  static Future<FlutterGoldenFileComparator> fromLocalFileComparator({
    SkiaGoldClient? goldens,
    required LocalFileComparator localFileComparator,
    required Platform platform,
    Directory? baseDirectory,
    required LogCallback log,
    required FileSystem fs,
    required ProcessManager process,
    required io.HttpClient httpClient,
  }) async {
    baseDirectory ??= FlutterGoldenFileComparator.getBaseDirectory(
      localFileComparator,
      platform: platform,
      fs: fs,
    );

    if (!baseDirectory.existsSync()) {
      baseDirectory.createSync(recursive: true);
    }

    goldens ??= SkiaGoldClient(
      baseDirectory,
      platform: platform,
      log: log,
      fs: fs,
      process: process,
      httpClient: httpClient,
    );
    try {
      // Check if we can reach Gold.
      await goldens.getExpectationForTest('');
    } on io.OSError catch (_) {
      return FlutterSkippingFileComparator(
        baseDirectory.uri,
        goldens,
        'OSError occurred, could not reach Gold. '
        'Switching to FlutterSkippingGoldenFileComparator.',
        platform: platform,
        log: log,
        fs: fs,
      );
    } on io.SocketException catch (_) {
      return FlutterSkippingFileComparator(
        baseDirectory.uri,
        goldens,
        'SocketException occurred, could not reach Gold. '
        'Switching to FlutterSkippingGoldenFileComparator.',
        platform: platform,
        log: log,
        fs: fs,
      );
    } on FormatException catch (_) {
      return FlutterSkippingFileComparator(
        baseDirectory.uri,
        goldens,
        'FormatException occurred, could not reach Gold. '
        'Switching to FlutterSkippingGoldenFileComparator.',
        platform: platform,
        log: log,
        fs: fs,
      );
    }

    return FlutterLocalFileComparator(
      baseDirectory.uri,
      goldens,
      platform: platform,
      log: log,
      fs: fs,
    );
  }

  @override
  Future<bool> compare(Uint8List imageBytes, Uri golden) async {
    golden = _addPrefix(golden);
    final String testName = skiaClient.cleanTestName(golden.path);
    late String? testExpectation;
    testExpectation = await skiaClient.getExpectationForTest(testName);

    if (testExpectation == null || testExpectation.isEmpty) {
      log(
        'No expectations provided by Skia Gold for test: $golden. '
        'This may be a new test. If this is an unexpected result, check '
        'https://flutter-gold.skia.org.\n'
        'Validate image output found at $basedir',
      );
      update(golden, imageBytes);
      return true;
    }

    ComparisonResult result;
    final List<int> goldenBytes = await skiaClient.getImageBytes(testExpectation);

    result = await GoldenFileComparator.compareLists(imageBytes, goldenBytes);

    if (result.passed) {
      result.dispose();
      return true;
    }

    final String error = await generateFailureOutput(result, golden, basedir);
    result.dispose();
    throw FlutterError(error);
  }
}
