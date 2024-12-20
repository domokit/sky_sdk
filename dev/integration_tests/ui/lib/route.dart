// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui' as ui;

import 'package:flutter_driver/driver_extension.dart';

// To use this test: "flutter drive --route '/smuggle-it' lib/route.dart"

void main() {
  enableFlutterDriverExtension(
    handler: (String? message) async {
      return ui.PlatformDispatcher.instance.defaultRouteName;
    },
  );
}
