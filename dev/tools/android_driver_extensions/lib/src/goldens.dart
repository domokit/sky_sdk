// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// A partial copy of [flutter_test.matchesGoldenFile] and supporting code.
//
// Flutter driver runs in the standalone Dart VM, which does not have access to
// the Flutter test library or `dart:ui`. This file provides a subset of the
// functionality of `flutter_test`'s `matchesGoldenFile` function, and we can
// consider refactoring this code to be shared between the two libraries
// (https://github.com/flutter/flutter/issues/152257).
part of '../native_driver.dart';

/// Whether golden files should be automatically updated during tests rather
/// than compared to the image bytes recorded by the tests.
///
/// When this is `true`, [matchesGoldenFile] will always report a successful
/// match, because the bytes being tested implicitly become the new golden.
bool autoUpdateGoldenFiles = false;

/// Compares pixels against those of a golden image file.
///
/// This comparator is used as the backend for [matchesGoldenFile].
///
/// By default, an exact pixel match to a local golden file is used.
GoldenFileComparator goldenFileComparator = const NaiveLocalFileComparator._();

/// Compares image pixels against a golden image file.
///
/// Instances of this comparator will be used as the backend for
/// [matchesGoldenFile].
abstract class GoldenFileComparator {
  /// @nodoc
  const GoldenFileComparator();

  /// Compares the pixels of decoded png [imageBytes] against the golden file
  /// identified by [golden].
  ///
  /// The returned future completes with a boolean value that indicates whether
  /// the pixels decoded from [imageBytes] match the golden file's pixels.
  ///
  /// In the case of comparison mismatch, the comparator may choose to throw a
  /// [TestFailure] if it wants to control the failure message, often in the
  /// form of a [ComparisonResult] that provides detailed information about the
  /// mismatch.
  ///
  /// The method by which [golden] is located and by which its bytes are loaded
  /// is left up to the implementation class. For instance, some implementations
  /// may load files from the local file system, whereas others may load files
  /// over the network or from a remote repository.
  Future<bool> compare(Uint8List imageBytes, Uri golden);

  /// Updates the golden file identified by [golden] with [imageBytes].
  ///
  /// This will be invoked in lieu of [compare] when [autoUpdateGoldenFiles]
  /// is `true` (which gets set automatically by the test framework when the
  /// user runs `flutter drive --update-goldens`).
  ///
  /// The method by which [golden] is located and by which its bytes are written
  /// is left up to the implementation class.
  Future<void> update(Uri golden, Uint8List imageBytes);

  /// Returns a new golden file [Uri] to incorporate any [version] number with
  /// the [key].
  ///
  /// The [version] is an optional int that can be used to differentiate
  /// historical golden files.
  ///
  /// Version numbers are used in golden file tests for package:flutter. You can
  /// learn more about these tests [here](https://github.com/flutter/flutter/blob/main/docs/contributing/testing/Writing-a-golden-file-test-for-package-flutter.md).
  Uri getTestUri(Uri key, int? version) {
    if (version == null) {
      return key;
    }
    final String keyString = key.toString();
    final String extension = path.extension(keyString);
    return Uri.parse('${keyString.split(extension).join()}.$version$extension');
  }
}

/// The default [GoldenFileComparator] implementation for `flutter drive`.
///
/// This comparator performs a pixel-for-pixel comparison of the decoded PNGs,
/// returning true only if there's an exact match. In cases where the captured
/// test image does not match the golden file, this comparator will provide a
/// fairly unhelpful error message, which could be improved in the future.
final class NaiveLocalFileComparator extends GoldenFileComparator {
  const NaiveLocalFileComparator._();

  @override
  Future<bool> compare(Uint8List imageBytes, Uri golden) async {
    final io.File goldenFile = _getTestFilePath(golden);
    final Uint8List goldenBytes;
    try {
      goldenBytes = await goldenFile.readAsBytes();
    } on io.PathNotFoundException {
      throw TestFailure('Golden file not found: ${goldenFile.path}');
    }

    if (goldenBytes.length != imageBytes.length) {
      return false;
    }

    for (int i = 0; i < goldenBytes.length; i++) {
      if (goldenBytes[i] != imageBytes[i]) {
        return false;
      }
    }

    return true;
  }

  @override
  Future<void> update(Uri golden, Uint8List imageBytes) async {
    final io.File goldenFile = _getTestFilePath(golden);
    await goldenFile.parent.create(recursive: true);
    await goldenFile.writeAsBytes(imageBytes);
  }

  /// Returns a path relative to the test script.
  ///
  /// This is hacky and unreliable, but it's the best we can do until we have
  /// more integration with the `flutter` CLI (which does all the heavy lifting
  /// for us in `flutter_test`).
  io.File _getTestFilePath(Uri golden) {
    final String testScriptPath = io.Platform.script.toFilePath();
    final String testScriptDir = path.dirname(testScriptPath);
    return io.File(path.join(testScriptDir, golden.path));
  }
}

// Examples can assume:
// import 'package:flutter_driver/src/native/driver.dart';
// import 'package:flutter_driver/src/native/goldens.dart';
// import 'package:test/test.dart';
// late NativeDriver nativeDriver;

/// Asserts that a [NativeScreenshot], [Future<NativeScreenshot>], or
/// [List<int>] matches the golden image file identified by [key], with an
/// optional [version] number].
///
/// The [key] may be either a [Uri] or a [String] representation of a URL.
///
/// The [version] is a number that can be used to differentiate historical
/// golden files. This parameter is optional.
///
/// This is an asynchronous matcher, meaning that callers should use
/// [flutter_test.expectLater] when using this matcher and await the future
/// returned by [flutter_test.expectLater].
///
/// ## Golden File Testing
///
/// The term __golden file__ refers to a master image that is considered the
/// true rendering of a given widget, state, application, or other visual
/// representation you have chosen to capture.
///
/// The master golden image files are tested against can be created or updated
/// by running `flutter drive --update-goldens` on the test.
///
/// {@tool snippet}
/// Sample invocations of [matchesGoldenFile].
///
/// ```dart
/// await expectLater(
///   nativeDriver.screenshot(),
///   matchesGoldenFile('save.png'),
/// );
/// ```
/// {@end-tool}
AsyncMatcher matchesGoldenFile(Object key, {int? version}) {
  return switch (key) {
    Uri() => _MatchesGoldenFile(key, version),
    String() => _MatchesGoldenFile.forStringPath(key, version),
    _ => throw ArgumentError('Unexpected type for golden file: ${key.runtimeType}'),
  };
}

/// The matcher created by [matchesGoldenFile].
final class _MatchesGoldenFile extends AsyncMatcher {
  /// Creates an instance of [MatchesGoldenFile].
  const _MatchesGoldenFile(this.key, this.version);

  /// Creates an instance of [MatchesGoldenFile] from a [String] path.
  _MatchesGoldenFile.forStringPath(String path, this.version) : key = Uri.parse(path);

  /// The [key] to the golden image.
  final Uri key;

  /// The [version] of the golden image.
  final int? version;

  @override
  Future<String?> matchAsync(Object? item) async {
    final Uri testNameUri = goldenFileComparator.getTestUri(key, version);

    final Uint8List buffer;
    if (item is FutureOr<List<int>>) {
      buffer = Uint8List.fromList(await item);
    } else if (item is FutureOr<NativeScreenshot>) {
      buffer = await (await item).readAsBytes();
    } else {
      throw ArgumentError('Unexpected type for golden file: ${item.runtimeType}');
    }

    if (autoUpdateGoldenFiles) {
      await goldenFileComparator.update(testNameUri, buffer);
      return null;
    }
    try {
      final bool success = await goldenFileComparator.compare(buffer, testNameUri);
      return success ? null : 'does not match';
    } on TestFailure catch (e) {
      return e.message;
    }
  }

  @override
  Description describe(Description description) {
    return description.add('app screenshot image matches golden file "$key"');
  }
}
