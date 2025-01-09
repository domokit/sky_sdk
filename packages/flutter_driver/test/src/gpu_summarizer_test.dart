// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_driver/flutter_driver.dart';
import 'package:flutter_driver/src/driver/gpu_sumarizer.dart';

import '../common.dart';

TimelineEvent newGPUTraceEvent(double ms) => TimelineEvent(<String, dynamic>{
  'name': 'GPUStart',
  'ph': 'b',
  'args': <String, String>{'FrameTimeMS': ms.toString()},
});

void main() {
  test('Can process GPU frame times.', () {
    final GpuSumarizer summarizer = GpuSumarizer(<TimelineEvent>[
      newGPUTraceEvent(4.233),
      newGPUTraceEvent(7.22),
      newGPUTraceEvent(9.1),
      newGPUTraceEvent(40.23),
    ]);

    expect(summarizer.computeAverageGPUTime(), closeTo(15.19, 0.1));
    expect(summarizer.computePercentileGPUTime(50.0), closeTo(9.1, 0.1));
    expect(summarizer.computeWorstGPUTime(), 40.23);
  });
}
