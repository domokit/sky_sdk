// Copyright 2013 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:ui/ui.dart' as ui;

import '../engine.dart';

class EngineFlutterDisplay extends ui.Display {
  EngineFlutterDisplay({required this.id, required this.size, required this.refreshRate});

  /// The single [EngineFlutterDisplay] that the web page is rendered on.
  static EngineFlutterDisplay get instance => _instance;
  static final EngineFlutterDisplay _instance = EngineFlutterDisplay(
    id: 0,
    size: ui.Size(domWindow.screen?.width ?? 0, domWindow.screen?.height ?? 0),
    refreshRate: 60,
  );

  @override
  final int id;

  // TODO(mdebbar): https://github.com/flutter/flutter/issues/133562
  // `size` and `refreshRate` should be kept up-to-date with the
  // browser. E.g. the window could be resized or moved to another display with
  // a different refresh rate.

  @override
  final ui.Size size;

  @override
  final double refreshRate;

  @override
  double get devicePixelRatio => _debugDevicePixelRatioOverride ?? browserDevicePixelRatio;

  /// The real device pixel ratio of the browser.
  ///
  /// This value cannot be overriden by tests, for example.
  double get browserDevicePixelRatio {
    double ratio = domWindow.devicePixelRatio;
    // Guard against WebOS returning 0.
    ratio = (ratio == 0.0) ? 1.0 : ratio;

    // The device pixel ratio is also affected by the scale factor of the
    // viewport. For example, on Chrome for Android, if the page is requested
    // with "Request Desktop Site" enabled, then the viewport size will be
    // very large, with the viewport scale less than 1.
    final double scale = domWindow.visualViewport?.scale ?? 1.0;
    return ratio * scale;
  }

  /// Overrides the default device pixel ratio.
  ///
  /// This is useful in tests to emulate screens of different dimensions.
  ///
  /// Passing `null` resets the device pixel ratio to the browser's default.
  void debugOverrideDevicePixelRatio(double? value) {
    _debugDevicePixelRatioOverride = value;
  }

  double? _debugDevicePixelRatioOverride;
}

/// Controls the screen orientation using the browser's screen orientation API.
class ScreenOrientation {
  const ScreenOrientation();

  static ScreenOrientation get instance => _instance;
  static const ScreenOrientation _instance = ScreenOrientation();

  static const String lockTypeAny = 'any';
  static const String lockTypeNatural = 'natural';
  static const String lockTypeLandscape = 'landscape';
  static const String lockTypePortrait = 'portrait';
  static const String lockTypePortraitPrimary = 'portrait-primary';
  static const String lockTypePortraitSecondary = 'portrait-secondary';
  static const String lockTypeLandscapePrimary = 'landscape-primary';
  static const String lockTypeLandscapeSecondary = 'landscape-secondary';

  /// Sets preferred screen orientation.
  ///
  /// Specifies the set of orientations the application interface can be
  /// displayed in.
  ///
  /// The [orientations] argument is a list of DeviceOrientation values.
  /// The empty list uses Screen unlock api and causes the application to
  /// defer to the operating system default.
  ///
  /// See w3c screen api: https://www.w3.org/TR/screen-orientation/
  Future<bool> setPreferredOrientation(List<dynamic> orientations) async {
    final DomScreen? screen = domWindow.screen;
    if (screen != null) {
      final DomScreenOrientation? screenOrientation = screen.orientation;
      if (screenOrientation != null) {
        if (orientations.isEmpty) {
          screenOrientation.unlock();
          return true;
        } else {
          final String? lockType = _deviceOrientationToLockType(orientations.first as String?);
          if (lockType != null) {
            try {
              await screenOrientation.lock(lockType);
              return true;
            } catch (_) {
              // On Chrome desktop an error with 'not supported on this device
              // error' is fired.
              return Future<bool>.value(false);
            }
          }
        }
      }
    }
    // API is not supported on this browser return false.
    return false;
  }

  // Converts device orientation to w3c OrientationLockType enum.
  //
  // See also: https://developer.mozilla.org/en-US/docs/Web/API/ScreenOrientation/lock
  static String? _deviceOrientationToLockType(String? deviceOrientation) {
    switch (deviceOrientation) {
      case 'DeviceOrientation.portraitUp':
        return lockTypePortraitPrimary;
      case 'DeviceOrientation.portraitDown':
        return lockTypePortraitSecondary;
      case 'DeviceOrientation.landscapeLeft':
        return lockTypeLandscapePrimary;
      case 'DeviceOrientation.landscapeRight':
        return lockTypeLandscapeSecondary;
      default:
        return null;
    }
  }
}
