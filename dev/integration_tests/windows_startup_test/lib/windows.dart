// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:typed_data';

import 'package:flutter/services.dart';

const MethodChannel _kMethodChannel = MethodChannel('tests.flutter.dev/windows_startup_test');

/// Returns true if the application's window is visible.
Future<bool> isWindowVisible() async {
  final bool? visible = await _kMethodChannel.invokeMethod<bool?>('isWindowVisible');
  if (visible == null) {
    throw 'Method channel unavailable';
  }

  return visible;
}

/// Returns true if the app's dark mode is enabled.
Future<bool> isAppDarkModeEnabled() async {
  final bool? enabled = await _kMethodChannel.invokeMethod<bool?>('isAppDarkModeEnabled');
  if (enabled == null) {
    throw 'Method channel unavailable';
  }

  return enabled;
}

/// Returns true if the operating system dark mode setting is enabled.
Future<bool> isSystemDarkModeEnabled() async {
  final bool? enabled = await _kMethodChannel.invokeMethod<bool?>('isSystemDarkModeEnabled');
  if (enabled == null) {
    throw 'Method channel unavailable';
  }

  return enabled;
}

/// Test conversion of a UTF16 string to UTF8 using the app template utils.
Future<String> testStringConversion(Int32List twoByteCodes) async {
  final String? converted = await _kMethodChannel.invokeMethod<String?>(
    'convertString',
    twoByteCodes,
  );
  if (converted == null) {
    throw 'Method channel unavailable.';
  }

  return converted;
}
