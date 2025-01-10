// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io' as io;

import 'package:flutter/foundation.dart';

/// Throws an [UnsupportedError] if the current platform is not Android.
void ensureAndroidDevice() {
  if (kIsWeb || !io.Platform.isAndroid) {
    throw UnsupportedError(
      'This app should only run on Android devices. It uses native Android '
      'plugins that are not developed for other platforms, and would need to '
      'be adapted to run on other platforms. See the README.md for details.',
    );
  }
}
