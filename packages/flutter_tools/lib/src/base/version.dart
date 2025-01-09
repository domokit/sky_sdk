// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:meta/meta.dart';

/// Represents the version of some piece of software.
///
/// While a [Version] object has fields resembling semver, it does not
/// necessarily represent a semver version.
@immutable
class Version implements Comparable<Version> {
  /// Creates a new [Version] object.
  ///
  /// A null [minor] or [patch] version is logically equivalent to 0. Using null
  /// for these parameters only affects the generation of [text], if no value
  /// for it is provided.
  factory Version(int major, int? minor, int? patch, {String? text}) {
    if (text == null) {
      text = '$major';
      if (minor != null) {
        text = '$text.$minor';
      }
      if (patch != null) {
        text = '$text.$patch';
      }
    }

    return Version._(major, minor ?? 0, patch ?? 0, text);
  }

  /// Public constant constructor when all fields are non-null, without default value fallbacks.
  const Version.withText(this.major, this.minor, this.patch, this._text);

  Version._(this.major, this.minor, this.patch, this._text) {
    if (major < 0) {
      throw ArgumentError('Major version must be non-negative.');
    }
    if (minor < 0) {
      throw ArgumentError('Minor version must be non-negative.');
    }
    if (patch < 0) {
      throw ArgumentError('Patch version must be non-negative.');
    }
  }

  /// Creates a new [Version] by parsing [text].
  static Version? parse(String? text) {
    final Match? match = versionPattern.firstMatch(text ?? '');
    if (match == null) {
      return null;
    }

    try {
      final int major = int.parse(match[1] ?? '0');
      final int minor = int.parse(match[3] ?? '0');
      final int patch = int.parse(match[5] ?? '0');
      return Version._(major, minor, patch, text ?? '');
    } on FormatException {
      return null;
    }
  }

  /// Returns the primary version out of a list of candidates.
  ///
  /// This is the highest-numbered stable version.
  static Version? primary(List<Version> versions) {
    Version? primary;
    for (final Version version in versions) {
      if (primary == null || (version > primary)) {
        primary = version;
      }
    }
    return primary;
  }

  /// The major version number: "1" in "1.2.3".
  final int major;

  /// The minor version number: "2" in "1.2.3".
  final int minor;

  /// The patch version number: "3" in "1.2.3".
  final int patch;

  /// The original string representation of the version number.
  ///
  /// This preserves textual artifacts like leading zeros that may be left out
  /// of the parsed version.
  final String _text;

  static final RegExp versionPattern = RegExp(r'^(\d+)(\.(\d+)(\.(\d+))?)?');

  /// Two [Version]s are equal if their version numbers are. The version text
  /// is ignored.
  @override
  bool operator ==(Object other) {
    return other is Version && other.major == major && other.minor == minor && other.patch == patch;
  }

  @override
  int get hashCode => Object.hash(major, minor, patch);

  bool operator <(Version other) => compareTo(other) < 0;
  bool operator >(Version other) => compareTo(other) > 0;
  bool operator <=(Version other) => compareTo(other) <= 0;
  bool operator >=(Version other) => compareTo(other) >= 0;

  @override
  int compareTo(Version other) {
    if (major != other.major) {
      return major.compareTo(other.major);
    }
    if (minor != other.minor) {
      return minor.compareTo(other.minor);
    }
    return patch.compareTo(other.patch);
  }

  @override
  String toString() => _text;
}

/// Returns true if [targetVersion] is within the range [min] and [max]
/// inclusive by default.
///
/// [min] and [max] are evaluated by [Version.parse(text)].
///
/// Pass [inclusiveMin] = false for greater than and not equal to min.
/// Pass [inclusiveMax] = false for less than and not equal to max.
bool isWithinVersionRange(
  String targetVersion, {
  required String min,
  required String max,
  bool inclusiveMax = true,
  bool inclusiveMin = true,
}) {
  final Version? parsedTargetVersion = Version.parse(targetVersion);
  final Version? minVersion = Version.parse(min);
  final Version? maxVersion = Version.parse(max);

  final bool withinMin =
      minVersion != null &&
      parsedTargetVersion != null &&
      (inclusiveMin ? parsedTargetVersion >= minVersion : parsedTargetVersion > minVersion);

  final bool withinMax =
      maxVersion != null &&
      parsedTargetVersion != null &&
      (inclusiveMax ? parsedTargetVersion <= maxVersion : parsedTargetVersion < maxVersion);
  return withinMin && withinMax;
}
