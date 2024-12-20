// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:meta/meta.dart';

/// Platforms that are supported by the Flutter tool.
///
/// This is partially based on the `flutter_tools` `TargetPlatform` class:
/// <https://github.com/flutter/flutter/blob/master/packages/flutter_tools/lib/src/build_info.dart>
///
/// This class is used to represent the target platform of a device.
@immutable
final class TargetPlatform {
  const TargetPlatform._(this.identifier);

  /// Android, host architecture left unspecified.
  static const androidUnspecified = TargetPlatform._('android');

  /// Android ARM.
  static const androidArm = TargetPlatform._('android-arm');

  /// Android ARM64.
  static const androidArm64 = TargetPlatform._('android-arm64');

  /// Android x64.
  static const androidX64 = TargetPlatform._('android-x64');

  /// Android x86.
  static const androidX86 = TargetPlatform._('android-x86');

  /// Linux ARM64.
  static const linuxArm64 = TargetPlatform._('linux-arm64');

  /// Linux x64.
  static const linuxX64 = TargetPlatform._('linux-x64');

  /// Windows ARM64.
  static const windowsArm64 = TargetPlatform._('windows-arm64');

  /// Windows x64.
  static const windowsX64 = TargetPlatform._('windows-x64');

  /// Fuchsia ARM64.
  static const fuchsiaArm64 = TargetPlatform._('fuchsia-arm64');

  /// Fuchsia x64.
  static const fuchsiaX64 = TargetPlatform._('fuchsia-x64');

  /// Darwin, host architecture left unspecified.
  static const darwinUnspecified = TargetPlatform._('darwin');

  /// Darwin ARM64.
  static const darwinArm64 = TargetPlatform._('darwin-arm64');

  /// Darwin x64.
  static const darwinX64 = TargetPlatform._('darwin-x64');

  /// iOS, host architecture left unspecified.
  static const iOSUnspecified = TargetPlatform._('ios');

  /// iOS, ARM64.
  static const iOSArm64 = TargetPlatform._('ios-arm64');

  /// iOS, x64.
  static const iOSX64 = TargetPlatform._('ios-x64');

  /// Flutter tester.
  static const tester = TargetPlatform._('flutter-tester');

  /// Web/Javascript.
  static const webJavascript = TargetPlatform._('web-javascript');

  /// Platforms that are recognized by the Flutter tool.
  ///
  /// There is no reason to use or iterate this list in non-test code; to check
  /// if a platform is recognized, use [tryParse] and check for `null` instead:
  ///
  /// ```dart
  /// final platform = TargetPlatform.tryParse('android-arm');
  /// if (platform == null) {
  ///   // Handle unrecognized platform.
  /// }
  /// ```
  @visibleForTesting
  static const knownPlatforms = [
    androidUnspecified,
    androidArm,
    androidArm64,
    androidX64,
    androidX86,
    linuxArm64,
    linuxX64,
    windowsArm64,
    windowsX64,
    fuchsiaArm64,
    fuchsiaX64,
    darwinUnspecified,
    darwinArm64,
    darwinX64,
    iOSUnspecified,
    iOSArm64,
    iOSX64,
    tester,
    webJavascript,
  ];

  /// Parses the [TargetPlatform] for a given [identifier].
  ///
  /// Returns `null` if the [identifier] is not recognized.
  static TargetPlatform? tryParse(String identifier) {
    for (final platform in knownPlatforms) {
      if (platform.identifier == identifier) {
        return platform;
      }
    }
    return null;
  }

  /// Parses the [TargetPlatform] for a given [identifier].
  ///
  /// Throws a [FormatException] if the [identifier] is not recognized.
  static TargetPlatform parse(String identifier) {
    final platform = tryParse(identifier);
    if (platform == null) {
      throw FormatException(
        'Unrecognized TargetPlatform. It is possible that "$identifier" is '
        'a new platform that is recognized by the `flutter` tool, but has not '
        'been added to engine_tool, or, if this is a test, an intentionally '
        'unrecognized platform was used.',
        identifier,
      );
    }
    return platform;
  }

  /// String-based identifier that is returned by `flutter device --machine`.
  ///
  /// See:
  /// - <https://github.com/flutter/flutter/blob/9441f9d48fce1d0b425628731dd6ecab8c8b0826/packages/flutter_tools/lib/src/device.dart#L878>.
  /// - <https://github.com/flutter/flutter/blob/master/packages/flutter_tools/lib/src/build_info.dart#L736>.
  final String identifier;

  @override
  bool operator ==(Object other) {
    return other is TargetPlatform && other.identifier == identifier;
  }

  @override
  int get hashCode => identifier.hashCode;

  @override
  String toString() => 'TargetPlatform <$identifier>';
}
