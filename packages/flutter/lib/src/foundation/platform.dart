// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

/// @docImport 'package:flutter/material.dart';
/// @docImport 'package:flutter_test/flutter_test.dart';
library;

import '_platform_io.dart' if (dart.library.js_util) '_platform_web.dart' as platform;
import 'assertions.dart';
import 'constants.dart';

/// The [TargetPlatform] that matches the platform on which the framework is
/// currently executing.
///
/// This is the default value of [ThemeData.platform] (hence the name). Widgets
/// from the material library should use [Theme.of] to determine the current
/// platform for styling purposes, rather than using [defaultTargetPlatform].
/// Widgets and render objects at lower layers that try to emulate the
/// underlying platform can depend on [defaultTargetPlatform] directly. The
/// [dart:io.Platform] object should only be used directly when it's critical to
/// actually know the current platform, without any overrides possible (for
/// example, when a system API is about to be called).
///
/// In a test environment, the platform returned is [TargetPlatform.android]
/// regardless of the host platform. (Android was chosen because the tests were
/// originally written assuming Android-like behavior, and we added platform
/// adaptations for iOS later). Tests can check iOS behavior by using the
/// platform override APIs (such as [ThemeData.platform] in the material
/// library) or by setting [debugDefaultTargetPlatformOverride] in debug builds.
///
/// Tests can also create specific platform tests by and adding a `variant:`
/// argument to the test and using a [TargetPlatformVariant].
///
/// See also:
///
/// * [kIsWeb], a boolean which is true if the application is running on the
///   web, where [defaultTargetPlatform] returns which platform the browser is
///   running on.
//
// When adding support for a new platform (e.g. Windows Phone, Raspberry Pi),
// first create a new value on the [TargetPlatform] enum, then add a rule for
// selecting that platform in `_platform_io.dart` and `_platform_web.dart`.
//
// It would be incorrect to make a platform that isn't supported by
// [TargetPlatform] default to the behavior of another platform, because doing
// that would mean we'd be stuck with that platform forever emulating the other,
// and we'd never be able to introduce dedicated behavior for that platform
// (since doing so would be a big breaking change).
@pragma('vm:platform-const-if', !kDebugMode)
TargetPlatform get defaultTargetPlatform => platform.defaultTargetPlatform;

/// The platform that user interaction should adapt to target.
///
/// The [defaultTargetPlatform] getter returns the current platform.
///
/// When using the "flutter run" command, the "o" key will toggle between
/// values of this enum when updating [debugDefaultTargetPlatformOverride].
/// This lets one test how the application will work on various platforms
/// without having to switch emulators or physical devices.
//
// When you add values here, make sure to also add them to
// nextPlatform() in flutter_tools/lib/src/resident_runner.dart so that
// the tool can support the new platform for its "o" option.
enum TargetPlatform {
  /// Android: <https://www.android.com/>
  android,

  /// Fuchsia: <https://fuchsia.dev/fuchsia-src/concepts>
  fuchsia,

  /// iOS: <https://www.apple.com/ios/>
  iOS,

  /// Linux: <https://www.linux.org>
  linux,

  /// macOS: <https://www.apple.com/macos>
  macOS,

  /// Windows: <https://www.windows.com>
  windows,
}

/// Override the [defaultTargetPlatform] in debug builds.
///
/// Setting this to null returns the [defaultTargetPlatform] to its original
/// value (based on the actual current platform).
///
/// Generally speaking this override is only useful for tests. To change the
/// platform that widgets resemble, consider using the platform override APIs
/// (such as [ThemeData.platform] in the material library) instead.
///
/// Setting [debugDefaultTargetPlatformOverride] (as opposed to, say,
/// [ThemeData.platform]) will cause unexpected and undesirable effects. For
/// example, setting this to [TargetPlatform.iOS] when the application is
/// running on Android will cause the TalkBack accessibility tool on Android to
/// be confused because it would be receiving data intended for iOS VoiceOver.
/// Similarly, setting it to [TargetPlatform.android] while on iOS will cause
/// certainly widgets to work assuming the presence of a system-wide back
/// button, which will make those widgets unusable since iOS has no such button.
///
/// Attempting to override this property in non-debug builds causes an error.
TargetPlatform? get debugDefaultTargetPlatformOverride => _debugDefaultTargetPlatformOverride;

set debugDefaultTargetPlatformOverride(TargetPlatform? value) {
  if (!kDebugMode) {
    throw FlutterError('Cannot modify debugDefaultTargetPlatformOverride in non-debug builds.');
  }
  _debugDefaultTargetPlatformOverride = value;
}

TargetPlatform? _debugDefaultTargetPlatformOverride;
