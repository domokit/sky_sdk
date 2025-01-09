// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui' as ui;
import 'package:flutter/services.dart';

void main() {
  BackgroundIsolateBinaryMessenger.ensureInitialized(ui.RootIsolateToken.instance!);
}
