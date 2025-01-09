// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/widgets.dart';
import 'package:flutter_driver/driver_extension.dart';

import 'package:platform_views_layout/main.dart' as app;

void main() {
  enableFlutterDriverExtension();
  runApp(const app.PlatformViewApp());
}
