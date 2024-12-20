// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_driver/flutter_driver.dart';
import 'package:test/test.dart' hide TypeMatcher, isInstanceOf;

void main() {
  group('page transition performance test', () {
    late FlutterDriver driver;

    setUpAll(() async {
      driver = await FlutterDriver.connect();

      await driver.waitUntilFirstFrameRasterized();
    });

    tearDownAll(() async {
      driver.close();
    });

    test('measure', () async {
      final Timeline timeline = await driver.traceAction(() async {
        await driver.tap(find.text('Material'));

        for (int i = 0; i < 10; i++) {
          await driver.tap(find.text('Banner'));
          await Future<void>.delayed(const Duration(milliseconds: 500));
          await driver.waitFor(find.byTooltip('Back'));
          await driver.tap(find.byTooltip('Back'));
          await Future<void>.delayed(const Duration(milliseconds: 500));
        }
      }, retainPriorEvents: true);

      final TimelineSummary summary = TimelineSummary.summarize(timeline);
      await summary.writeTimelineToFile('page_transition_perf', pretty: true);
    }, timeout: Timeout.none);
  });
}
