// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter/foundation.dart';
import 'package:flutter/gestures.dart';
import 'package:flutter/material.dart';
import 'package:flutter_test/flutter_test.dart';
import 'package:integration_test/integration_test.dart';
import 'package:macrobenchmarks/src/simple_scroll.dart';

void main() {
  final IntegrationTestWidgetsFlutterBinding binding =
      IntegrationTestWidgetsFlutterBinding.ensureInitialized();
  testWidgets('Frame Counter and Input Delay for benchmarkLive', (WidgetTester tester) async {
    await tester.pumpWidget(const MaterialApp(home: Scaffold(body: SimpleScroll())));
    await tester.pumpAndSettle();
    final Offset location = tester.getCenter(find.byType(ListView));
    int frameCount = 0;
    void frameCounter(Duration elapsed) {
      frameCount += 1;
    }

    tester.binding.addPersistentFrameCallback(frameCounter);

    const int timeInSecond = 1;
    const Duration totalTime = Duration(seconds: timeInSecond);
    const int moveEventNumber = timeInSecond * 120; // 120Hz
    const Offset movePerRun = Offset(0.0, -200.0 / moveEventNumber);
    final List<PointerEventRecord> records = <PointerEventRecord>[
      PointerEventRecord(Duration.zero, <PointerEvent>[
        PointerAddedEvent(position: location),
        PointerDownEvent(position: location, pointer: 1),
      ]),
      ...<PointerEventRecord>[
        for (int t = 0; t < moveEventNumber; t++)
          PointerEventRecord(totalTime * (t / moveEventNumber), <PointerEvent>[
            PointerMoveEvent(
              timeStamp: totalTime * (t / moveEventNumber),
              position: location + movePerRun * t.toDouble(),
              pointer: 1,
              delta: movePerRun,
            ),
          ]),
      ],
      PointerEventRecord(totalTime, <PointerEvent>[
        PointerUpEvent(
          // Deviate a little from integer number of frames to reduce flakiness
          timeStamp: totalTime - const Duration(milliseconds: 1),
          position: location + movePerRun * moveEventNumber.toDouble(),
          pointer: 1,
        ),
      ]),
    ];

    binding.framePolicy = LiveTestWidgetsFlutterBindingFramePolicy.benchmarkLive;
    List<Duration> delays = await tester.handlePointerEventRecord(records);
    await tester.pumpAndSettle();
    binding.reportData = <String, dynamic>{'benchmarkLive': _summarizeResult(frameCount, delays)};
    await tester.idle();
    await tester.binding.delayed(const Duration(milliseconds: 250));

    binding.framePolicy = LiveTestWidgetsFlutterBindingFramePolicy.fullyLive;
    frameCount = 0;
    delays = await tester.handlePointerEventRecord(records);
    await tester.pumpAndSettle();
    binding.reportData!['fullyLive'] = _summarizeResult(frameCount, delays);
    await tester.idle();
  });
}

Map<String, dynamic> _summarizeResult(final int frameCount, final List<Duration> delays) {
  assert(delays.length > 1);
  final List<int> delayedInMicro =
      delays.map<int>((Duration delay) => delay.inMicroseconds).toList();
  final List<int> delayedInMicroSorted = List<int>.from(delayedInMicro)..sort();
  final int index90th = (delayedInMicroSorted.length * 0.90).round();
  final int percentile90th = delayedInMicroSorted[index90th];
  final int sum = delayedInMicroSorted.reduce((int a, int b) => a + b);
  final double averageDelay = sum.toDouble() / delayedInMicroSorted.length;
  return <String, dynamic>{
    'frame_count': frameCount,
    'average_delay_millis': averageDelay / 1E3,
    '90th_percentile_delay_millis': percentile90th / 1E3,
    if (kDebugMode) 'delaysInMicro': delayedInMicro,
  };
}
