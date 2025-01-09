// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

// See //dev/devicelab/bin/tasks/flutter_gallery__memory_nav.dart

import 'dart:async';

import 'package:flutter/material.dart';
import 'package:flutter/scheduler.dart';
import 'package:flutter_gallery/gallery/app.dart' show GalleryApp;
import 'package:flutter_test/flutter_test.dart';

Future<void> endOfAnimation() async {
  do {
    await SchedulerBinding.instance.endOfFrame;
  } while (SchedulerBinding.instance.hasScheduledFrame);
}

Rect boundsFor(WidgetController controller, Finder item) {
  final RenderBox box = controller.renderObject<RenderBox>(item);
  return box.localToGlobal(Offset.zero) & box.size;
}

Future<void> main() async {
  final Completer<void> ready = Completer<void>();
  runApp(
    GestureDetector(
      onTap: () {
        debugPrint('==== MEMORY BENCHMARK ==== TAPPED ====');
        ready.complete();
      },
      behavior: HitTestBehavior.opaque,
      child: const IgnorePointer(child: GalleryApp(testMode: true)),
    ),
  );
  await SchedulerBinding.instance.endOfFrame;
  debugPrint('==== MEMORY BENCHMARK ==== READY ====');

  await ready.future;
  debugPrint('Continuing...');

  // Wait out any errant taps due to synchronization
  await Future<void>.delayed(const Duration(milliseconds: 200));

  // remove onTap handler, enable pointer events for app
  runApp(
    GestureDetector(child: const IgnorePointer(ignoring: false, child: GalleryApp(testMode: true))),
  );
  await SchedulerBinding.instance.endOfFrame;

  final WidgetController controller = LiveWidgetController(WidgetsBinding.instance);

  debugPrint('Navigating...');
  await controller.tap(find.text('Material'));
  await Future<void>.delayed(const Duration(milliseconds: 150));
  final Finder demoList = find.byKey(const Key('GalleryDemoList'));
  final Finder demoItem = find.text('Text fields');
  do {
    await controller.drag(demoList, const Offset(0.0, -300.0));
    await Future<void>.delayed(const Duration(milliseconds: 20));
  } while (!demoItem.tryEvaluate());

  // Ensure that the center of the "Text fields" item is visible
  // because that's where we're going to tap
  final Rect demoItemBounds = boundsFor(controller, demoItem);
  final Rect demoListBounds = boundsFor(controller, demoList);
  if (!demoListBounds.contains(demoItemBounds.center)) {
    await controller.drag(
      demoList,
      Offset(0.0, demoListBounds.center.dy - demoItemBounds.center.dy),
    );
    await endOfAnimation();
  }

  for (int iteration = 0; iteration < 15; iteration += 1) {
    debugPrint('Tapping... (iteration $iteration)');
    await controller.tap(demoItem);
    await endOfAnimation();
    debugPrint('Backing out...');
    await controller.tap(find.byTooltip('Back'));
    await endOfAnimation();
    await Future<void>.delayed(const Duration(milliseconds: 50));
  }

  debugPrint('==== MEMORY BENCHMARK ==== DONE ====');
}
