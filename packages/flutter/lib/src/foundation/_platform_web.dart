// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui_web' as ui_web;

import 'platform.dart' as platform;

export 'platform.dart' show TargetPlatform;

/// The web implementation of [platform.defaultTargetPlatform].
platform.TargetPlatform get defaultTargetPlatform {
  // To get a better guess at the targetPlatform we need to be able to reference
  // the window, but that won't be available until we fix the platforms
  // configuration for Flutter.
  return platform.debugDefaultTargetPlatformOverride ?? _testPlatform ?? _browserPlatform;
}

// The TargetPlatform used on Web tests, unless overridden.
//
// Respects the `ui_web.browser.debugOperatingSystemOverride` value (when set).
platform.TargetPlatform? get _testPlatform {
  platform.TargetPlatform? testPlatform;
  assert(() {
    if (ui_web.debugEmulateFlutterTesterEnvironment) {
      // Return the overridden operatingSystem in tests, if any...
      if (ui_web.browser.debugOperatingSystemOverride != null) {
        testPlatform = _operatingSystemToTargetPlatform(ui_web.browser.operatingSystem);
      } else {
        // Fall back to `android` for tests.
        testPlatform = platform.TargetPlatform.android;
      }
    }
    return true;
  }());
  return testPlatform;
}

// Current browser platform.
//
// The computation of `operatingSystem` is cached in the ui_web package;
// this getter may be called dozens of times per frame.
//
// _browserPlatform is lazily initialized, and cached forever.
final platform.TargetPlatform _browserPlatform = _operatingSystemToTargetPlatform(
  ui_web.browser.operatingSystem,
);

// Converts an ui_web.OperatingSystem enum into a platform.TargetPlatform.
platform.TargetPlatform _operatingSystemToTargetPlatform(ui_web.OperatingSystem os) {
  return switch (os) {
    ui_web.OperatingSystem.android => platform.TargetPlatform.android,
    ui_web.OperatingSystem.iOs => platform.TargetPlatform.iOS,
    ui_web.OperatingSystem.linux => platform.TargetPlatform.linux,
    ui_web.OperatingSystem.macOs => platform.TargetPlatform.macOS,
    ui_web.OperatingSystem.windows => platform.TargetPlatform.windows,
    // Resolve 'unknown' OS values to `android`.
    ui_web.OperatingSystem.unknown => platform.TargetPlatform.android,
  };
}
