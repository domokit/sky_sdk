// Copyright 2014 The Flutter Authors. All rights reserved.
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

import 'package:flutter_driver/flutter_driver.dart';
import 'package:flutter_driver/src/driver/memory_summarizer.dart';

import '../common.dart';

TimelineEvent newGPUTraceEvent(double ms) => TimelineEvent(<String, dynamic>{
  'name': 'AllocatorVK',
  'ph': 'b',
  'args': <String, String>{'MemoryBudgetUsageMB': ms.toString()},
});

TimelineEvent newMetalGPUTraceEvent(double ms) => TimelineEvent(<String, dynamic>{
  'name': 'AllocatorMTL',
  'ph': 'b',
  'args': <String, String>{'MemoryBudgetUsageMB': ms.toString()},
});

void main() {
  test('Can process GPU memory usage times.', () {
    final GPUMemorySumarizer summarizer = GPUMemorySumarizer(<TimelineEvent>[
      newGPUTraceEvent(1024),
      newGPUTraceEvent(1024),
      newGPUTraceEvent(512),
      newGPUTraceEvent(2048),
    ]);

    expect(summarizer.computeAverageMemoryUsage(), closeTo(1152, 0.1));
    expect(summarizer.computePercentileMemoryUsage(50.0), closeTo(1024, 0.1));
    expect(summarizer.computeWorstMemoryUsage(), 2048);
  });

  test('Can process Metal GPU memory usage times.', () {
    final GPUMemorySumarizer summarizer = GPUMemorySumarizer(<TimelineEvent>[
      newMetalGPUTraceEvent(1024),
      newMetalGPUTraceEvent(1024),
      newMetalGPUTraceEvent(512),
      newMetalGPUTraceEvent(2048),
    ]);

    expect(summarizer.computeAverageMemoryUsage(), closeTo(1152, 0.1));
    expect(summarizer.computePercentileMemoryUsage(50.0), closeTo(1024, 0.1));
    expect(summarizer.computeWorstMemoryUsage(), 2048);
  });
}
