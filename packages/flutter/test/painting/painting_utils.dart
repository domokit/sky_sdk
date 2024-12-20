// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:ui' as ui;

import 'package:flutter/foundation.dart';
import 'package:flutter/painting.dart';
import 'package:flutter/scheduler.dart';
import 'package:flutter/services.dart';

class PaintingBindingSpy extends BindingBase
    with SchedulerBinding, ServicesBinding, PaintingBinding {
  int counter = 0;
  int get instantiateImageCodecCalledCount => counter;

  @override
  Future<ui.Codec> instantiateImageCodecWithSize(
    ui.ImmutableBuffer buffer, {
    ui.TargetImageSizeCallback? getTargetSize,
  }) {
    counter++;
    return ui.instantiateImageCodecWithSize(buffer, getTargetSize: getTargetSize);
  }

  @override
  // ignore: must_call_super
  void initLicenses() {
    // Do not include any licenses, because we're a test, and the LICENSE file
    // doesn't get generated for tests.
  }
}
