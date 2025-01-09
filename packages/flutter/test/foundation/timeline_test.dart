// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter_test/flutter_test.dart';

// IMPORTANT: keep this in sync with the same constant defined
//            in foundation/timeline.dart
const int kSliceSize = 500;

void main() {
  setUp(() {
    FlutterTimeline.debugReset();
    FlutterTimeline.debugCollectionEnabled = false;
  });

  test('Does not collect when collection not enabled', () {
    FlutterTimeline.startSync('TEST');
    FlutterTimeline.finishSync();
    expect(() => FlutterTimeline.debugCollect(), throwsStateError);
  });

  test('Collects when collection is enabled', () {
    FlutterTimeline.debugCollectionEnabled = true;
    FlutterTimeline.startSync('TEST');
    FlutterTimeline.finishSync();
    final AggregatedTimings data = FlutterTimeline.debugCollect();
    expect(data.timedBlocks, hasLength(1));
    expect(data.aggregatedBlocks, hasLength(1));

    final AggregatedTimedBlock block = data.getAggregated('TEST');
    expect(block.name, 'TEST');
    expect(block.count, 1);

    // After collection the timeline is reset back to empty.
    final AggregatedTimings data2 = FlutterTimeline.debugCollect();
    expect(data2.timedBlocks, isEmpty);
    expect(data2.aggregatedBlocks, isEmpty);
  });

  test('Deletes old data when reset', () {
    FlutterTimeline.debugCollectionEnabled = true;
    FlutterTimeline.startSync('TEST');
    FlutterTimeline.finishSync();
    FlutterTimeline.debugReset();

    final AggregatedTimings data = FlutterTimeline.debugCollect();
    expect(data.timedBlocks, isEmpty);
    expect(data.aggregatedBlocks, isEmpty);
  });

  test('Reports zero aggregation when requested missing block', () {
    FlutterTimeline.debugCollectionEnabled = true;

    final AggregatedTimings data = FlutterTimeline.debugCollect();
    final AggregatedTimedBlock block = data.getAggregated('MISSING');
    expect(block.name, 'MISSING');
    expect(block.count, 0);
    expect(block.duration, 0);
  });

  test('Measures the runtime of a function', () {
    FlutterTimeline.debugCollectionEnabled = true;

    // The off-by-one values for `start` and `end` are for web's sake where
    // timer values are reported as float64 and toInt/toDouble conversions
    // are noops, so there's no value truncation happening, which makes it
    // a bit inconsistent with Stopwatch.
    final int start = FlutterTimeline.now - 1;
    FlutterTimeline.timeSync('TEST', () {
      final Stopwatch watch = Stopwatch()..start(); // flutter_ignore: stopwatch (see analyze.dart)
      // Ignore context: Used safely for benchmarking.
      while (watch.elapsedMilliseconds < 5) {}
      watch.stop();
    });
    final int end = FlutterTimeline.now + 1;

    final AggregatedTimings data = FlutterTimeline.debugCollect();
    expect(data.timedBlocks, hasLength(1));
    expect(data.aggregatedBlocks, hasLength(1));

    final TimedBlock block = data.timedBlocks.single;
    expect(block.name, 'TEST');
    expect(block.start, greaterThanOrEqualTo(start));
    expect(block.end, lessThanOrEqualTo(end));
    expect(block.duration, greaterThan(0));

    final AggregatedTimedBlock aggregated = data.getAggregated('TEST');
    expect(aggregated.name, 'TEST');
    expect(aggregated.count, 1);
    expect(aggregated.duration, block.duration);
  });

  test('FlutterTimeline.instanceSync does not collect anything', () {
    FlutterTimeline.debugCollectionEnabled = true;
    FlutterTimeline.instantSync('TEST');

    final AggregatedTimings data = FlutterTimeline.debugCollect();
    expect(data.timedBlocks, isEmpty);
    expect(data.aggregatedBlocks, isEmpty);
  });

  test('FlutterTimeline.now returns a value', () {
    FlutterTimeline.debugCollectionEnabled = true;
    expect(FlutterTimeline.now, isNotNull);
  });

  test('Can collect more than one slice of data', () {
    FlutterTimeline.debugCollectionEnabled = true;

    for (int i = 0; i < 10 * kSliceSize; i++) {
      FlutterTimeline.startSync('TEST');
      FlutterTimeline.finishSync();
    }
    final AggregatedTimings data = FlutterTimeline.debugCollect();
    expect(data.timedBlocks, hasLength(10 * kSliceSize));
    expect(data.aggregatedBlocks, hasLength(1));

    final AggregatedTimedBlock block = data.getAggregated('TEST');
    expect(block.name, 'TEST');
    expect(block.count, 10 * kSliceSize);
  });

  test('Collects blocks in a correct order', () {
    FlutterTimeline.debugCollectionEnabled = true;
    const int testCount = 7 * kSliceSize ~/ 2;

    for (int i = 0; i < testCount; i++) {
      FlutterTimeline.startSync('TEST$i');
      FlutterTimeline.finishSync();
    }

    final AggregatedTimings data = FlutterTimeline.debugCollect();
    expect(data.timedBlocks, hasLength(testCount));
    expect(
      data.timedBlocks.map<String>((TimedBlock block) => block.name).toList(),
      List<String>.generate(testCount, (int i) => 'TEST$i'),
    );
  });
}
