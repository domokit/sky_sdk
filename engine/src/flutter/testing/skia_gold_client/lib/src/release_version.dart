// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:convert';

import 'package:meta/meta.dart';

/// Signals that the output of golden files may diverge from `main`.
///
/// By default, Skia Gold will compare the output of a test to the golden file
/// in the `main` branch. During a release branch, the output of the test may
/// diverge (intentionally or accidentally, i.e. a bug) from the output in
/// `main`.
///
/// This class signals to Skia Gold to use a unique test name for the release.
@immutable
final class ReleaseVersion {
  /// Creates a [ReleaseVersion] with the given [major] and [minor] version.
  ///
  /// For example, for the `3.21` release:
  /// ```dart
  /// ReleaseVersion(
  ///   major: 3,
  ///   minor: 21,
  /// )
  /// ```
  ///
  /// Each number must be non-negative.
  ReleaseVersion({required this.major, required this.minor}) {
    RangeError.checkNotNegative(major, 'major');
    RangeError.checkNotNegative(minor, 'minor');
  }

  /// Parses a [ReleaseVersion] from the contents of a `release.version` file.
  ///
  /// Returns `null` if and only if the version is parsed as the string `none`.
  ///
  /// The format of the file is plaintext, and any lines that are empty
  /// (newline characters only) or start with `#` are ignored. The first line
  /// that is not ignored must be in the format `major.minor`, where `major` and
  /// `minor` are non-negative integers.
  ///
  /// For example, the following file contents:
  /// ```txt
  /// # This is a comment
  ///
  /// 3.21
  /// ```
  ///
  /// ... would parse to `ReleaseVersion(major: 3, minior: 21)`.
  ///
  /// Throws a [FormatException] if the file contents are not in the expected
  /// format (either empty/comments only, or missing a `major.minor` line in
  /// the format described in [ReleaseVersion.new]).
  static ReleaseVersion? parse(String fileContents) {
    bool parsedNone = false;
    ReleaseVersion? parsed;
    for (final String line in LineSplitter.split(fileContents)) {
      final String trimmed = line.trim();
      if (trimmed.isEmpty || trimmed.startsWith('#')) {
        continue;
      }
      if (trimmed == 'none') {
        parsedNone = true;
        continue;
      }
      final List<String> parts = trimmed.split('.');
      if (parts.length != 2) {
        throw FormatException('Expected "major.minor" format, got "$trimmed"', fileContents);
      }
      final int? major = int.tryParse(parts[0]);
      final int? minor = int.tryParse(parts[1]);
      if (major == null || minor == null) {
        throw FormatException(
          'Expected "major.minor", each as an integer, got "$trimmed"',
          fileContents,
        );
      }
      if (parsed != null) {
        throw FormatException('Multiple "major.minor" versions found', fileContents);
      }
      parsed = ReleaseVersion(major: major, minor: minor);
    }
    if (parsed != null && parsedNone) {
      throw FormatException('Both "none" and a "major.minor" version found', fileContents);
    }
    if (parsed != null) {
      return parsed;
    }
    if (parsedNone) {
      return null;
    }
    throw FormatException('No "major.minor" version found', fileContents);
  }

  /// The major version number.
  final int major;

  /// The minor version number.
  final int minor;

  @override
  bool operator ==(Object other) {
    return other is ReleaseVersion && other.major == major && other.minor == minor;
  }

  @override
  int get hashCode => Object.hash(major, minor);

  @override
  String toString() => '$major.$minor';
}
