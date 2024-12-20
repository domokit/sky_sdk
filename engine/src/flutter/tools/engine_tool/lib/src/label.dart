// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:meta/meta.dart';

/// An identifier of a target that loosely follows [GN][] and [Bazel][] format.
///
/// [gn]: https://gn.googlesource.com/gn/+/master/docs/reference.md#labels
/// [bazel]: https://bazel.build/concepts/labels
///
/// A label in `engine_tool` are two string components separated by a colon in
/// the format `//path/to/package-name:target-name`, with constraints on the
/// characters that can be used in each component (see [package] and [target]).
///
/// Unlike counterparts in Bazel and GN:
/// - The package name is always a source-absolute path (i.e. starts with `//`).
/// - Valid identifier characters are `a-zA-Z0-9_-`, not starting with a digit.
/// - The target name is never empty, even when it is a default target.
@immutable
final class Label {
  /// Creates a new label with the given package and target name.
  ///
  /// If [target] is omitted, the final component of [package] is used as the
  /// target name (e.g. `//name` becomes `//name:name`).
  ///
  /// Throws a [FormatException] if the package or target name is invalid.
  factory Label(String package, [String? target]) {
    final FormatException? packageError = _checkPackage(package);
    if (packageError != null) {
      throw packageError;
    }

    target ??= package.substring(package.lastIndexOf('/') + 1);
    final FormatException? targetError = _checkTarget(target);
    if (targetError != null) {
      throw targetError;
    }

    return Label._(package, target);
  }

  const Label._(this.package, this.target);

  /// Parses a label from a string in the format `//package-name:target-name`.
  ///
  /// Throws a [FormatException] if the label is invalid.
  static Label parse(String label) {
    final int colon = label.indexOf(':');
    if (colon == -1) {
      return Label(label);
    }
    return Label(label.substring(0, colon), label.substring(colon + 1));
  }

  /// Parses a label from a string in the format `//package-name:target-name`.
  ///
  /// If a toolchain is present, it is removed.
  ///
  /// Throws a [FormatException] if the label is invalid.
  static Label parseGn(String label) {
    // Remove (//build/toolchain/...) at the end of the label.
    if (label.endsWith(')')) {
      final int start = label.lastIndexOf('(', label.length - 2);
      if (start != -1) {
        label = label.substring(0, start);
      }
    }

    return Label.parse(label);
  }

  /// A source-absolute package name, starting with `//`.
  ///
  /// The package name must be a valid identifier, i.e. it must be a sequence of
  /// identifier components separated by slashes and cannot end with a slash.
  final String package;

  /// A target name within the package.
  ///
  /// The target name must be a valid identifier.
  final String target;

  @override
  bool operator ==(Object other) {
    return other is Label && package == other.package && target == other.target;
  }

  @override
  int get hashCode => Object.hash(package, target);

  /// Returns a string representation of the label (i.e. used by [parse]).
  ///
  /// Use [toNinjaLabel] to remove the leading `//` from the package name.
  @override
  String toString() => '$package:$target';

  /// Returns a Ninja-compatible string representation of the label.
  ///
  /// `//package-name:target-name` becomes `package-name:target-name`.
  String toNinjaLabel() => '${package.substring(2)}:$target';

  static FormatException? _checkPackage(String package) {
    // Must start with a double slash.
    if (!package.startsWith('//')) {
      return FormatException('Package name must start with "//".', package);
    }

    // Cannot end with a slash.
    if (package.endsWith('/')) {
      return FormatException('Package name must not end with a slash.', package);
    }

    // Check each component of the package name for valid identifier characters.
    // We use a standard loop to give a better error message.
    int i = 2;
    while (true) {
      final int j = package.indexOf('/', i);
      final String component = j == -1 ? package.substring(i) : package.substring(i, j);
      if (!_identifier.hasMatch(component)) {
        return FormatException('Package name component must be a valid identifier.', package, i);
      }
      if (j == -1) {
        return null;
      }
      i = j + 1;
    }
  }

  static FormatException? _checkTarget(String target) {
    if (!_identifier.hasMatch(target)) {
      return FormatException('Target name must be a valid identifier.', target);
    }
    return null;
  }

  static final RegExp _identifier = RegExp(r'^[a-zA-Z_][a-zA-Z0-9_-]*$');
}

/// A generic target pattern that can be used to match multiple targets.
///
/// Similar to [Label], but supports wildcards in the package and target name:
/// - `//path/to/package/...` all targets in the package and its subpackages.
/// - `//path/to/package:all` all targets in the package.
///
/// Inspired by <https://bazel.build/run/build#specifying-build-targets>.
@immutable
final class TargetPattern {
  /// Creates a new pattern with the given package and target name.
  ///
  /// If [target] is omitted, the pattern matches all targets in the package.
  factory TargetPattern(String package, [String? target]) {
    // We are so close to a Label that we can reuse the same checks.
    // The only difference is that the package name can end with `...`.
    bool packageEndsWithWildcard = false;
    if (package.endsWith('/...')) {
      packageEndsWithWildcard = true;
      package = package.substring(0, package.length - 4);
    }

    // Edgecase: //... is a valid pattern that matches all targets.
    if (packageEndsWithWildcard && package == '/') {
      return const TargetPattern._('//...', null);
    }

    // Throws a FormatException if the package or target name is invalid.
    final Label label = Label(package, target);
    return TargetPattern._(
      packageEndsWithWildcard ? '${label.package}...' : label.package,
      packageEndsWithWildcard ? null : label.target,
    );
  }

  const TargetPattern._(this.package, this.target);

  /// Parses a pattern from a string in the format `//package-name:target-name`.
  ///
  /// Throws a [FormatException] if the pattern is invalid.
  static TargetPattern parse(String pattern) {
    final int colon = pattern.indexOf(':');
    if (colon == -1) {
      return TargetPattern(pattern);
    }
    return TargetPattern(pattern.substring(0, colon), pattern.substring(colon + 1));
  }

  /// A source-absolute package name optionally ending with `...`.
  final String package;

  /// A target name within the package, or `all` to match all targets.
  ///
  /// If [package] ends with `...`, [target] must be `null`.
  final String? target;

  @override
  bool operator ==(Object other) {
    return other is TargetPattern && package == other.package && target == other.target;
  }

  @override
  int get hashCode => Object.hash(package, target);

  /// Returns a string representation of the target pattern.
  ///
  /// The string representation is compatible with [parse].
  @override
  String toString() => target == null ? package : '$package:$target';

  /// Returns a string representation of the target pattern in [GN format][gn].
  ///
  /// [gn]: https://gn.googlesource.com/gn/+/master/docs/reference.md#labels
  ///
  /// Specifically:
  /// - `//package/to/package/...` is converted to `path/to/package/*`.
  /// - `//package/to/package:all` is converted to `path/to/package:*`.
  String toGnPattern() {
    // Remove the leading `//` and replace `...` and `all`.
    final String package = this.package.substring(2);
    if (target == null) {
      assert(package.endsWith('...'));

      // Edgecase: //... is a valid pattern that matches all targets.
      if (package == '...') {
        return '/*';
      }

      return package.replaceRange(package.length - 3, package.length, '/*');
    }
    return '$package:${target == 'all' ? '*' : target}';
  }
}
