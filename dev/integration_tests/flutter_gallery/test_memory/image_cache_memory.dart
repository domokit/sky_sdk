// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/scheduler.dart';
// See //dev/devicelab/bin/tasks/flutter_gallery__image_cache_memory.dart

import 'package:flutter/widgets.dart';
import 'package:flutter_test/flutter_test.dart';

// Once we provide an option for images to be resized to
// fit the container, we should see a significant drop in
// the amount of memory consumed by this benchmark.
Future<void> main() async {
  const int numItems = 10;

  runApp(
    Directionality(
      textDirection: TextDirection.ltr,
      child: ListView.builder(
        key: const Key('ImageList'),
        itemCount: numItems,
        itemBuilder: (BuildContext context, int position) {
          return SizedBox(
            width: 200,
            height: 200,
            child: Center(
              child: Image.asset(
                'monochrome/red-square-1024x1024.png',
                package: 'flutter_gallery_assets',
                width: 100,
                height: 100,
                fit: BoxFit.contain,
                key: Key('image_$position'),
              ),
            ),
          );
        },
      ),
    ),
  );

  await SchedulerBinding.instance.endOfFrame;

  // We are waiting for the GPU to rasterize a frame here. This makes this
  // flaky, we can rely on a more deterministic source such as
  // PlatformDispatcher.onReportTimings once
  // https://github.com/flutter/flutter/issues/26154 is addressed.
  await Future<void>.delayed(const Duration(milliseconds: 50));
  debugPrint('==== MEMORY BENCHMARK ==== READY ====');

  final WidgetController controller = LiveWidgetController(WidgetsBinding.instance);

  debugPrint('Scrolling...');
  final Finder list = find.byKey(const Key('ImageList'));
  final Finder lastItem = find.byKey(const Key('image_${numItems - 1}'));
  do {
    await controller.drag(list, const Offset(0.0, -30.0));
    await Future<void>.delayed(const Duration(milliseconds: 20));
  } while (!lastItem.tryEvaluate());

  debugPrint('==== MEMORY BENCHMARK ==== DONE ====');
}
