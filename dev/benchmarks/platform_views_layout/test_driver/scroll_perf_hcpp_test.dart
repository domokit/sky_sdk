// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_driver/flutter_driver.dart';
import 'package:test/test.dart' hide TypeMatcher, isInstanceOf;

void main() {
  group('scrolling performance test', () {
    late FlutterDriver driver;

    setUpAll(() async {
      driver = await FlutterDriver.connect();

      await driver.waitUntilFirstFrameRasterized();
    });

    tearDownAll(() async {
      driver.close();
    });

    Future<void> testScrollPerf(String listKey, String summaryName) async {
      // The slight initial delay avoids starting the timing during a
      // period of increased load on the device. Without this delay, the
      // benchmark has greater noise.
      // See: https://github.com/flutter/flutter/issues/19434
      await Future<void>.delayed(const Duration(milliseconds: 250));

      await driver.forceGC();

      final Timeline timeline = await driver.traceAction(() async {
        // Find the scrollable stock list
        final SerializableFinder list = find.byValueKey(listKey);

        for (int j = 0; j < 5; j += 1) {
          // Scroll down
          for (int i = 0; i < 5; i += 1) {
            await driver.scroll(list, 0.0, -300.0, const Duration(milliseconds: 300));
            await Future<void>.delayed(const Duration(milliseconds: 500));
          }

          // Scroll up
          for (int i = 0; i < 5; i += 1) {
            await driver.scroll(list, 0.0, 300.0, const Duration(milliseconds: 300));
            await Future<void>.delayed(const Duration(milliseconds: 500));
          }
        }
      });

      final TimelineSummary summary = TimelineSummary.summarize(timeline);
      await summary.writeTimelineToFile(summaryName, pretty: true);
    }

    test('platform_views_scroll_perf', () async {
      // Disable frame sync, since there are ongoing animations.
      await driver.runUnsynchronized(() async {
        await testScrollPerf('platform-views-scroll', 'platform_views_hcpp_scroll_perf');
      });
    }, timeout: Timeout.none);
  });
}
