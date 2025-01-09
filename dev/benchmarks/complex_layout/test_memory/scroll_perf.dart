// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'dart:async';

import 'package:complex_layout/src/app.dart';
import 'package:flutter/material.dart';
import 'package:flutter/scheduler.dart';
import 'package:flutter_test/flutter_test.dart';

/// The speed, in pixels per second, that the drag gestures should end with.
const double speed = 1500.0;

/// The number of down drags and the number of up drags. The total number of
/// gestures is twice this number.
const int maxIterations = 4;

/// The time that is allowed between gestures for the fling effect to settle.
const Duration pauses = Duration(milliseconds: 500);

Future<void> main() async {
  final Completer<void> ready = Completer<void>();
  runApp(
    GestureDetector(
      onTap: () {
        debugPrint('==== MEMORY BENCHMARK ==== TAPPED ====');
        ready.complete();
      },
      behavior: HitTestBehavior.opaque,
      child: const IgnorePointer(child: ComplexLayoutApp()),
    ),
  );
  await SchedulerBinding.instance.endOfFrame;
  debugPrint('==== MEMORY BENCHMARK ==== READY ====');

  await ready.future; // waits for tap sent by devicelab task
  debugPrint('Continuing...');

  // Wait out any errant taps due to synchronization
  await Future<void>.delayed(const Duration(milliseconds: 200));

  // remove onTap handler, enable pointer events for app
  runApp(GestureDetector(child: const IgnorePointer(ignoring: false, child: ComplexLayoutApp())));
  await SchedulerBinding.instance.endOfFrame;

  final WidgetController controller = LiveWidgetController(WidgetsBinding.instance);

  // Scroll down
  for (int iteration = 0; iteration < maxIterations; iteration += 1) {
    debugPrint('Scroll down... $iteration/$maxIterations');
    await controller.fling(find.byType(ListView), const Offset(0.0, -700.0), speed);
    await Future<void>.delayed(pauses);
  }

  // Scroll up
  for (int iteration = 0; iteration < maxIterations; iteration += 1) {
    debugPrint('Scroll up... $iteration/$maxIterations');
    await controller.fling(find.byType(ListView), const Offset(0.0, 300.0), speed);
    await Future<void>.delayed(pauses);
  }

  debugPrint('==== MEMORY BENCHMARK ==== DONE ====');
}
