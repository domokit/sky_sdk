// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:flutter/material.dart';
import 'package:flutter_driver/driver_extension.dart';

import 'package:macrobenchmarks/src/animated_image.dart';

/// This test is slightly different than most of the other tests in this
/// application, in that it directly instantiates the page we care about and
/// passes a callback. This way, we can make sure to consistently wait for a
/// set number of image frames to render.
Future<void> main() async {
  final Completer<void> waiter = Completer<void>();
  enableFlutterDriverExtension(
    handler: (String? request) async {
      if (request != 'waitForAnimation') {
        throw UnsupportedError('Unrecognized request $request');
      }
      await waiter.future;
      return 'done';
    },
  );
  runApp(
    MaterialApp(
      home: AnimatedImagePage(
        onFrame: (int frameNumber) {
          if (frameNumber == 250) {
            waiter.complete();
          }
        },
      ),
    ),
  );
}
