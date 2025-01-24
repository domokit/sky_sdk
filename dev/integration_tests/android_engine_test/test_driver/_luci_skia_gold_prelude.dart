// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:io' as io;

/// Whether the current environment is LUCI.
bool get isLuci => io.Platform.environment['LUCI_CI'] == 'True';

/// What golden suffix this test should use for different engine configurations.
String get goldenVariant {
  final String? variant = io.Platform.environment['ANDROID_ENGINE_TEST_GOLDEN_VARIANT'];
  if (variant == null) {
    return '';
  }
  return '.$variant';
}
